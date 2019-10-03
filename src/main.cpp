#if __MBED__
#include "mbed.h"
#endif

#include <stdio.h>
#include <string.h>

#define nil NULL

#if DEVICE_SERIAL
RawSerial io(USBTX, USBRX/*, 115200*/);

int poop = EOF;

int _getc() {
    if (poop != EOF) {
        int c = poop;
        poop = EOF;
        return c;
    }

    return io.getc();
}

void _ungetc(int c) { poop = c; }
void _putc(int c) { io.putc(c); }
void _flush() { }
#else
int _getc() { return fgetc(stdin); }
void _ungetc(int c) { ungetc(c, stdin); }
void _putc(int c) { fputc(c, stdout); }
void _flush() { fflush(stdout); }
#endif

void _putn(int n) {
    if (n < 0) {
        _putc('-');
        n = -n;
    }

    if (n < 10) {
        _putc('0' + n);
    } else {
        char buf[sizeof(int) * 3] = { 0 };

        int i = 0;

        for ( ; 0 < n; n = n / 10) {
            buf[i++] = '0' + (n % 10);
        }

        while (0 <= --i) {
            _putc(buf[i]);
        }
    }
}

void _puts(const char *s) {
    while (*s != '\0') {
        _putc(*s);
        s++;
    }
}

#if DEVICE_SLEEP
uint32_t ispr0, ispr1, ispr2, icsr;

#define __ZZZ() ({ \
    uint32_t primask = __get_PRIMASK(); \
    __disable_irq(); \
    ispr0 = NVIC->ISPR[0];  \
    ispr1 = NVIC->ISPR[1];  \
    ispr2 = NVIC->ISPR[2];  \
    icsr = SCB->ICSR;       \
    __set_PRIMASK(primask); \
})

void _sleep() {
    core_util_critical_section_enter();

#if DEVICE_USTICKER
    us_ticker_disable_interrupt();
    us_ticker_clear_interrupt();
#endif
#if DEVICE_LPTICKER
    lp_ticker_disable_interrupt();
    lp_ticker_clear_interrupt();
#endif

    if (/*sleep_manager_can_deep_sleep()*/false) {
        hal_deepsleep();
    } else {
        hal_sleep();
    }

    core_util_critical_section_exit();
}
#endif

#if DEVICE_ANALOGIN
AnalogIn ai[] = {
    AnalogIn(A0), AnalogIn(A1), AnalogIn(A2), AnalogIn(A3), AnalogIn(A4), AnalogIn(A5), AnalogIn(D0), AnalogIn(D1),
  /*AnalogIn(ADC_TEMP),*/ AnalogIn(ADC_VREF)/*, AnalogIn(ADC_VBAT)*/
};
#endif

#if FEATURE_BLE
#include "ble/BLE.h"
#include "events/mbed_events.h"

EventQueue event_queue(16 * EVENTS_EVENT_SIZE);

const UUID UART_UUID   ("6E40" "0001" "-B5A3-F393-E0A9-E50E24DCCA9E");
const UUID UART_TX_UUID("6E40" "0002" "-B5A3-F393-E0A9-E50E24DCCA9E");
const UUID UART_RX_UUID("6E40" "0003" "-B5A3-F393-E0A9-E50E24DCCA9E");

class BleuArt : private mbed::NonCopyable<BleuArt>, public ble::Gap::EventHandler {
public:
    static const unsigned BLE_UART_SERVICE_MAX_DATA_LEN = (BLE_GATT_MTU_SIZE_DEFAULT - 3);

private:
    BLE &_ble;
    events::EventQueue &_event_queue;

    DigitalOut _led1;
    DigitalOut _led2;
    DigitalOut _led3;

    uint8_t rxBuffer[BLE_UART_SERVICE_MAX_DATA_LEN];
    uint8_t txBuffer[BLE_UART_SERVICE_MAX_DATA_LEN];
    uint8_t txIndex = 0;
    uint8_t rxTotal = 0;
    uint8_t rxIndex = 0;

    GattCharacteristic txCharacteristic;
    GattCharacteristic rxCharacteristic;

    bool _connected = false;
    int _blinker = 0;

public:
    BleuArt(BLE &ble, events::EventQueue &event_queue) :
        _ble(ble),
        _event_queue(event_queue),
        _led1(LED1, 0),
        _led2(LED2, 0),
        _led3(LED3, 0),
        txCharacteristic(UART_TX_UUID, rxBuffer, 1, BLE_UART_SERVICE_MAX_DATA_LEN, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE |
                                                                                   GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE),
        rxCharacteristic(UART_RX_UUID, txBuffer, 1, BLE_UART_SERVICE_MAX_DATA_LEN, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY) {
        _ble.gap().setEventHandler(this);
        _puts("uart constructed\n"); // 
    }

    void run() {
        if (!_ble.hasInitialized()) {
            _ble.init(this, &BleuArt::on_init_complete);
        } else {
            _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
            _puts("advertising started\n"); // 
        }

        if (_blinker == 0) {
            _blinker = _event_queue.call_every(200, this, &BleuArt::blink);
            _puts("blinker started\n"); // 
        }

        _puts("dispatch forever\n"); // 
        _event_queue.dispatch_forever();
        _puts("dispatch broken\n"); // 
    }

    ~BleuArt() {
        if (_blinker != 0) {
            _event_queue.cancel(_blinker);
            _puts("blinker stopped\n"); // 
        }

        if (_ble.hasInitialized()) {
            _ble.shutdown();
            _puts("ble shutdown complete\n"); // 
        }

        _ble.gap().setEventHandler(nil);
        _led3 = _led2 = _led1 = 0;
        _puts("uart destructed\n"); // 
    }

    uint16_t txHandle() { return txCharacteristic.getValueAttribute().getHandle(); }
    uint16_t rxHandle() { return rxCharacteristic.getValueAttribute().getHandle(); }

    int __getc() { return (rxIndex < rxTotal) ? rxBuffer[rxIndex++] : EOF; }

    size_t write(const void *_buffer, size_t _length) {
        if (_connected) {
            const uint8_t *buffer = static_cast<const uint8_t *>(_buffer);
            size_t         length = _length;
            unsigned       index  = 0;

            while (length) {
                unsigned m = BLE_UART_SERVICE_MAX_DATA_LEN - txIndex;
                unsigned n = (length < m) ? length : m;

                memcpy(&txBuffer[txIndex], &buffer[index], n);
                length  -= n;
                txIndex += n;
                index   += n;

                if (txIndex == BLE_UART_SERVICE_MAX_DATA_LEN || txBuffer[txIndex - 1] == '\n') {
                    _ble.gattServer().write(rxHandle(), txBuffer, txIndex);
                    txIndex = 0;
                }
            }
        }

        return _length;
    }

    int __putc(int c) { return (write(&c, 1) == 1) ? 1 : EOF; }

    void __putn(int n) {
        if (n < 0) {
            __putc('-');
            n = -n;
        }

        if (n < 10) {
            __putc('0' + n);
        } else {
            char buf[sizeof(int) * 3] = { 0 };

            int i = 0;

            for ( ; 0 < n; n = n / 10) {
                buf[i++] = '0' + (n % 10);
            }

            while (0 <= --i) {
                __putc(buf[i]);
            }
        }
    }

    size_t __puts(const char *str) { return write(str, strlen(str)); }

    void flush() {
        if (_connected) {
            if (txIndex != 0) {
                _ble.gattServer().write(rxHandle(), txBuffer, txIndex);
                txIndex = 0;
            }
        }
    }

protected:
    void blink(void) {
        _led2 = !_led2;
#if DEVICE_ANALOGIN
        _led1 = (0.3f < ai[0]) ? 1 : 0; // 

        int m = sizeof(ai) / sizeof(*ai);
        int r = (1 << 12);

        __putn(0); _putn(0); // 
        __putc(' '); _putc(' '); // 

        for (int i = 0; i < m; i++) {
            int v = ai[i] * r;
            char c = ' '; //  (i < m - 1) ? ' ' : '\n';

            __putn(v); _putn(v); //
            __putc(c); _putc(c); //
        }

        __putn(r - 1); _putn(r - 1); // 
        __putc('\n'); _putc('\n'); // 
#else
        const char *s = _led2 ? "p1ng\n" : "p0ng\n"; __puts(s); _puts(s); // 
#endif
    }

    void onDataWritten(const GattWriteCallbackParams *params) {
        _led3 = !_led3; // 
        if (params->handle == txHandle()) {
            uint16_t len = params->len;
            if (len <= BLE_UART_SERVICE_MAX_DATA_LEN) {
                rxTotal = len;
                rxIndex = 0;
                memcpy(rxBuffer, params->data, rxTotal);
                _putc('"'); for (uint8_t i = rxIndex; i < rxTotal; i++) { _putc(rxBuffer[i]); } _puts("\" received\n"); // 
            }
        }
    }

private:
    void on_init_complete(BLE::InitializationCompleteCallbackContext *context) {
        _puts("ble init complete\n"); // 
        BLE &ble = context->ble;

        {
            GattCharacteristic *table[] = { &txCharacteristic, &rxCharacteristic };
            GattService service(UART_UUID, table, sizeof(table) / sizeof(*table));

            ble.gattServer().addService(service);
            ble.gattServer().onDataWritten(this, &BleuArt::onDataWritten);
        }

        {
            ble::AdvertisingParameters aps(
                ble::advertising_type_t::CONNECTABLE_UNDIRECTED,
                ble::adv_interval_t(ble::millisecond_t(1000))
            );

            ble.gap().setAdvertisingParameters(ble::LEGACY_ADVERTISING_HANDLE, aps);
        }

        {
            uint8_t adv_buffer[ble::LEGACY_ADVERTISING_MAX_SIZE];
            ble::AdvertisingDataBuilder adb(adv_buffer);

            adb.setFlags();
            adb.setName("Charm Cortex-ζ");
            adb.setLocalServiceList(mbed::make_Span(&UART_UUID, 1));

            ble.gap().setAdvertisingPayload(ble::LEGACY_ADVERTISING_HANDLE, adb.getAdvertisingData());
        }

        ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
        _puts("advertising started\n"); // 
    }

    void onConnectionComplete(const ble::ConnectionCompleteEvent &e) {
        _puts("connection complete\n"); // 
        _connected = true;
    }

    void onDisconnectionComplete(const ble::DisconnectionCompleteEvent &e) {
        _puts("disconnection complete\n"); // 
        _connected = false;
     // _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
        _event_queue.break_dispatch(); // 
    }
};

void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    event_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
}

void bleuart() {
    BLE &ble = BLE::Instance();
    ble.onEventsToProcess(schedule_ble_events);
    {
        BleuArt uart(ble, event_queue);
        uart.run();
    }
    ble.onEventsToProcess(nil);
}
#endif

#if DEVICE_ANALOGIN
#include "PeripheralPins.h"

PinName _pins[] = { A0, A1, A2, A3, A4, A5, D0, D1 };
const int _n = sizeof(_pins) / sizeof(*_pins);
const int _m = 4000 * _n;

class ZooLog {
public:
    ZooLog() {
        _led1 = 0;

        _values = new uint16_t[_m];

        lock();
        init();
        unlock();
    }

    void init() {
        for (int i = 0; i < _n; i++) {
            uint32_t function = (uint32_t)NC;

            if (_pins[i] < 0xF0 || 0x100 <= _pins[i]) {
                _handle.Instance = (ADC_TypeDef *)pinmap_peripheral(_pins[i], PinMap_ADC);
                function = pinmap_function(_pins[i], PinMap_ADC);
                pinmap_pinout(_pins[i], PinMap_ADC);
            } else {
                _handle.Instance = (ADC_TypeDef *)pinmap_peripheral(_pins[i], PinMap_ADC_Internal);
                function = pinmap_function(_pins[i], PinMap_ADC_Internal);
            }
            MBED_ASSERT(_handle.Instance != (ADC_TypeDef *)NC);
            MBED_ASSERT(function != (uint32_t)NC);

            _channels[i] = STM_PIN_CHANNEL(function);
        }

        _handle.State = HAL_ADC_STATE_RESET;
        _handle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV4;
        _handle.Init.Resolution            = ADC_RESOLUTION_12B;
        _handle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
        _handle.Init.ScanConvMode          = ADC_SCAN_ENABLE;
        _handle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
        _handle.Init.LowPowerAutoWait      = ENABLE; // 
        _handle.Init.ContinuousConvMode    = ENABLE;
        _handle.Init.NbrOfConversion       = _n;
        _handle.Init.DiscontinuousConvMode = DISABLE;
        _handle.Init.NbrOfDiscConversion   = 1;
        _handle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
        _handle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
        _handle.Init.DMAContinuousRequests = DISABLE;
        _handle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
        _handle.Init.OversamplingMode      = DISABLE;

        __HAL_RCC_ADC_CLK_ENABLE();

        __HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_SYSCLK);

        if (HAL_ADC_Init(&_handle) != HAL_OK) {
            error("ADC init failed\r\n");
        }

        if (!HAL_ADCEx_Calibration_GetValue(&_handle, ADC_SINGLE_ENDED)) {
            HAL_ADCEx_Calibration_Start(&_handle, ADC_SINGLE_ENDED);
        }

        HAL_NVIC_SetPriority(COMP_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(COMP_IRQn);

        _hcomp1.Instance = COMP1;
        _hcomp1.State = HAL_COMP_STATE_RESET;
        _hcomp1.Init.InputMinus = COMP_INPUT_MINUS_VREFINT;
        _hcomp1.Init.InputPlus = COMP_INPUT_PLUS_IO3;
        _hcomp1.Init.OutputPol = COMP_OUTPUTPOL_INVERTED;
        _hcomp1.Init.Hysteresis = COMP_HYSTERESIS_LOW;
        _hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
        _hcomp1.Init.Mode = COMP_POWERMODE_MEDIUMSPEED;
        _hcomp1.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
        _hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING_FALLING;

        if (HAL_COMP_Init(&_hcomp1) != HAL_OK) {
            debug("COMP init failed\r\n");    
        }
    }

    void run() {
        lock();
        read();
        unlock();

        for (int i = 0; i < _m; i++) {
            _putn(_values[i]);
            bool more = (i % _n < _n - 1);
            _putc(more ? ' ' : '\n');
            if (!more) {
                i += _m / 40 - _n;
            }
        }
    }

    void read() {
        ADC_ChannelConfTypeDef sConfig = {0};

        for (int i = 0; i < _n; i++) {
            switch (i) {
                case 0: sConfig.Rank = ADC_REGULAR_RANK_1; break;
                case 1: sConfig.Rank = ADC_REGULAR_RANK_2; break;
                case 2: sConfig.Rank = ADC_REGULAR_RANK_3; break;
                case 3: sConfig.Rank = ADC_REGULAR_RANK_4; break;
                case 4: sConfig.Rank = ADC_REGULAR_RANK_5; break;
                case 5: sConfig.Rank = ADC_REGULAR_RANK_6; break;
                case 6: sConfig.Rank = ADC_REGULAR_RANK_7; break;
                case 7: sConfig.Rank = ADC_REGULAR_RANK_8; break;
                case 8: sConfig.Rank = ADC_REGULAR_RANK_9; break;
                case 9: sConfig.Rank = ADC_REGULAR_RANK_10; break;
                case 10: sConfig.Rank = ADC_REGULAR_RANK_11; break;
                case 11: sConfig.Rank = ADC_REGULAR_RANK_12; break;
                case 12: sConfig.Rank = ADC_REGULAR_RANK_13; break;
                case 13: sConfig.Rank = ADC_REGULAR_RANK_14; break;
                case 14: sConfig.Rank = ADC_REGULAR_RANK_15; break;
                case 15: sConfig.Rank = ADC_REGULAR_RANK_16; break;
                default:
                    MBED_ASSERT(false);
                    break;
            }

            sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
            sConfig.SingleDiff   = ADC_SINGLE_ENDED;
            sConfig.OffsetNumber = ADC_OFFSET_NONE;
            sConfig.Offset       = 0;

            switch (_channels[i]) {
                case 0:
                    sConfig.Channel = ADC_CHANNEL_VREFINT;
                    sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
                    break;
                case 1: sConfig.Channel = ADC_CHANNEL_1; break;
                case 2: sConfig.Channel = ADC_CHANNEL_2; break;
                case 3: sConfig.Channel = ADC_CHANNEL_3; break;
                case 4: sConfig.Channel = ADC_CHANNEL_4; break;
                case 5: sConfig.Channel = ADC_CHANNEL_5; break;
                case 6: sConfig.Channel = ADC_CHANNEL_6; break;
                case 7: sConfig.Channel = ADC_CHANNEL_7; break;
                case 8: sConfig.Channel = ADC_CHANNEL_8; break;
                case 9: sConfig.Channel = ADC_CHANNEL_9; break;
                case 10: sConfig.Channel = ADC_CHANNEL_10; break;
                case 11: sConfig.Channel = ADC_CHANNEL_11; break;
                case 12: sConfig.Channel = ADC_CHANNEL_12; break;
                case 13: sConfig.Channel = ADC_CHANNEL_13; break;
                case 14: sConfig.Channel = ADC_CHANNEL_14; break;
                case 15: sConfig.Channel = ADC_CHANNEL_15; break;
                case 16: sConfig.Channel = ADC_CHANNEL_16; break;
                case 17:
                     sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
                     sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
                     break;
                case 18:
                     sConfig.Channel = ADC_CHANNEL_VBAT;
                     sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
                     break;
                default:
                     MBED_ASSERT(false);
                     break;
            }

            if (HAL_ADC_ConfigChannel(&_handle, &sConfig) != HAL_OK) {
                debug("ADC channel configuration failed\r\n");
            }
        }

        if (HAL_COMP_Start(&_hcomp1) != HAL_OK) {
            debug("COMP start failed\r\n");    
        }

_puts("\nzzz...\n"); // 
        _sleep();

        if (HAL_ADC_Start(&_handle) != HAL_OK) {
            debug("ADC start of conversion failed\r\n");
        }

        for (int i = 0; i < _m; i++) {
            uint16_t value = 0;
            if (HAL_ADC_PollForConversion(&_handle, 128) == HAL_OK) {
                value = (uint16_t)HAL_ADC_GetValue(&_handle);
            }
            _values[i] = value;
        }

        if (HAL_ADC_Stop(&_handle) != HAL_OK) {
            debug("ADC stop of conversion failed\r\n");
        }

        if (HAL_COMP_Stop(&_hcomp1) != HAL_OK) {
            debug("COMP stop failed\r\n");    
        }

        LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE((&_handle)->Instance), LL_ADC_PATH_INTERNAL_NONE);
    }

    void deinit() {
        if (HAL_COMP_DeInit(&_hcomp1) != HAL_OK) {
            debug("COMP deinit failed\r\n");    
        }

        HAL_NVIC_DisableIRQ(COMP_IRQn);

        if (HAL_ADC_DeInit(&_handle) != HAL_OK) {
            error("ADC deinit failed\r\n");
        }

        __HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_NONE);

        __HAL_RCC_ADC_CLK_DISABLE();
    }

    virtual ~ZooLog() {
        lock();
        deinit();
        unlock();

        delete _values;

        _led1 = 0;
    }

protected:
    virtual void lock() { _mutex->lock(); }
    virtual void unlock() { _mutex->unlock(); }

    static SingletonPtr<PlatformMutex> _mutex;

    ADC_HandleTypeDef _handle;
    uint8_t _channels[_n];
    uint16_t *_values;

public:
    static COMP_HandleTypeDef _hcomp1;
    static DigitalOut _led1;
};

SingletonPtr<PlatformMutex> ZooLog::_mutex;

COMP_HandleTypeDef ZooLog::_hcomp1 = {0};
DigitalOut ZooLog::_led1(LED1, 0);

#ifdef __cplusplus
extern "C" {
#endif

void COMP_IRQHandler(void) {
    HAL_COMP_IRQHandler(&ZooLog::_hcomp1);
}

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp) {
    ZooLog::_led1 = !ZooLog::_led1;
}

#ifdef __cplusplus
}
#endif

void zoolog() {
    ZooLog ai;
    ai.run();
}
#endif

typedef enum { EOT = -1, ERR, QUOTED, LPAREN, RPAREN, ALPHA, DIGIT, EOL } token_t;

enum {
    LIST, NUMBER, SYMBOL, VAR, QUOTE, NIL, T, COND, DEFUN, FSETQ, NULLP, FUNCALL, PROG, GO, RETRN, LABL, FREPLACA, FREPLACD, FAPPLY, FLIST,
#if FEATURE_BLE
    BLEUART,
#endif
#if DEVICE_ANALOGIN
    ZOOLOG,
#endif
    FUSER, FADD1, FSUB1, FPLUS, FDIFF, FTIMES, FQUOT, LESSP, EQP, GREATERP, ZEROP, NUMBERP, FAND, FOR, FNOT, FCONS, FCAR, FCDR, FREAD, FEVAL, FPRINT, FATOM
};

int isfunc(int t) { return (FUSER <= t); }

struct Cons {
    int type;
    union {
        int number;
        const char *symbol;
    } u;
    Cons *car;
    Cons *cdr;

    Cons(Cons *car, Cons *cdr) : type(LIST), car(car), cdr(cdr) { }
};

Cons *cons(Cons *car, Cons *cdr) { return new Cons(car, cdr); }

Cons *car(Cons *p) { return (p == nil) ? nil : p->car; }
Cons *cdr(Cons *p) { return (p == nil) ? nil : p->cdr; }

int _type(Cons *p) { return p->type; }
int _number(Cons *p) { return p->u.number; }
const char *_symbol(Cons *p) { return (p == nil) ? nil : p->u.symbol; }

void rplaca(Cons *p, Cons *q) { p->car = q; }
void rplacd(Cons *p, Cons *q) { p->cdr = q; }
void rplact(Cons *p, int t) { p->type = t; }

Cons *number(int n) {
    Cons *p = cons(nil, nil);
    p->u.number = n;
    rplact(p, NUMBER);

    return p;
}

Cons *ENV;
Cons *TRUE;

Cons *declare(const char *name) {
    Cons *p = cons(nil, nil);
    p->u.symbol = name;
    rplact(p, VAR);

    ENV = cons(p, ENV);

    return p;
}

Cons *eval(Cons *x, Cons *env);

Cons *evalcond(Cons *expr, Cons *env) {
    if (expr == nil) {
        return nil;
    }

    if (eval(car(car(expr)), env) != nil) {
        return eval(car(cdr(car(expr))), env);
    }

    return evalcond(cdr(expr), env);
}

Cons *pairargs(Cons *pars, Cons *args, Cons *env, bool prog) {
    if (pars == nil) {
        return env;
    }

    Cons *p = cons(nil, car(args)); // value of param is corresponding arg
    p->u.symbol = _symbol(car(car(pars)));
    rplact(p, VAR);

    return cons(p, pairargs(cdr(pars), prog ? cons(nil, nil) : cdr(args), env, prog));
}

Cons *evalargs(Cons *args, Cons *env) {
    if (args == nil) {
        return nil;
    }

    return cons(eval(car(args), env), evalargs(cdr(args), env));
}

// find_labels - change the type of all labels in a PROG to LABL
void find_labels(Cons *p) {
    for ( ; p != nil; p = cdr(p)) {
        if (_type(car(p)) == VAR) {
            rplact(car(p), LABL);        // change the type to LABL
            rplacd(car(car(p)), cdr(p)); // label points to next statement
        }
    }
}

bool progon;

Cons *evalprog(Cons *p, Cons *env) {
    Cons *x = nil;

    // set up parameters as locals
    env = pairargs(car(cdr(p)), cons(nil, nil), env, true);
    progon = true;
    p = cdr(cdr(p)); // p now points to the statement list
    find_labels(p);  // set up all labels in the prog

    while (p != nil && progon) {
        x = eval(car(p), env);
        if (_type(car(car(p))) == GO) {
            p = x;      // GO returned the next statement to go to
        } else {
            p = cdr(p); // just follow regular chain of statements

        }
    }

    progon = true; // in case of nested progs
    return x;
}

Cons *lookup(Cons *env, const char *name) {
    Cons *p = env;

    while (p != nil && strcmp(_symbol(car(p)), name) != 0) {
        p = cdr(p);
    }

    return (p == nil) ? nil : car(p);
}

Cons *arith(Cons *op, Cons *x, Cons *y) {
    int t = _type(op);

    if (t == LESSP) {
        return (_number(x) < _number(y)) ? TRUE : nil;
    }
    if (t == GREATERP) {
        return (_number(x) > _number(y)) ? TRUE : nil;
    }

    int n = 0;

    switch (t) {
        case FPLUS: n = _number(x) + _number(y); break;
        case FDIFF: n = _number(x) - _number(y); break;
        case FTIMES: n = _number(x) * _number(y); break;
        case FQUOT: n = _number(x) / _number(y); break;
        case FADD1: n = _number(x) + 1; break;
        case FSUB1: n = _number(x) - 1; break;
    }

    return number(n);
}

bool _isalpha(int c) {
    return ('A' <= c && c <= 'Z') || ('a' <= c && c <= 'z') || strchr("!*+-/<=>?_", c) != nil; // 
}

bool _isdigit(int c) {
    return ('0' <= c && c <= '9');
}

Cons *read_number() {
    int c;
    int n = 0;

    while (true) {
        c = _getc();
        if (!_isdigit(c)) {
            break;
        }
        n = n * 10 + c - '0';
    }

    _ungetc(c);

    return number(n);
}

Cons *read_symbol() {
    char inbuf[32 + 1];
    char *s = inbuf;

    {
        int c = _getc();
        *s = c;
        s++;
        if (c != '\'') {
            while ((unsigned)(s - inbuf) < sizeof(inbuf) - 1) {
                c = _getc();
                if (!_isalpha(c) && !_isdigit(c)) {
                    _ungetc(c);
                    break;
                }
                *s = c;
                s++;
            }
        }
        *s = '\0';
    }

    Cons *q = lookup(ENV, inbuf);
    if (q == nil) {
        s = new char[strlen(inbuf) + 1];
        strcpy(s, inbuf);
        q = declare(s);
    }

    Cons *p = cons(q, nil);
    rplact(p, _type(q));

    return p;
}

int advance() {
    int c;
    while (true) {
        c = _getc();
        if (c == EOF) {
            break;
        }
        if (strchr(" \t\r,", c) == nil) { // 
            break;
        }
    }
    _ungetc(c);
    return c;
}

token_t read_token() {
    int c = advance();

    if (_isalpha(c)) {
        return ALPHA;
    }
    if (_isdigit(c)) {
        return DIGIT;
    }

    switch (c) {
        case '(': case '[': return LPAREN;
        case ')': case ']': return RPAREN;
        case '\'': return QUOTED;
        case '\n': return EOL;
        case EOF: return EOT;
    }

    return ERR;
}

Cons *eq(Cons *x, Cons *y) {
    if (x == nil || y == nil) {
        if (x == y) {
            return TRUE;
        }
    } else if (_type(x) == SYMBOL && _type(y) == SYMBOL && car(x) == car(y)) {
        return TRUE;
    }

    return nil;
}

Cons *atom(Cons *x) {
    if (x == nil) {
        return TRUE;
    }
    int typ = _type(x);
    if (typ == NUMBER || typ == SYMBOL) {
         return TRUE;
    }
    return nil;
}

Cons *_and(Cons *x) {
    for (Cons *p = cdr(x); p != nil; p = cdr(p)) {
        if (eval(car(p), nil) == nil) {
            return nil;
        }
    }
    return TRUE;
}

Cons *_or(Cons *x) {
    for (Cons *p = cdr(x); p != nil; p = cdr(p)) {
        if (eval(car(p), nil) != nil) {
            return TRUE;
        }
    }
    return nil;
}

Cons *_not(Cons *x) {
    return (eval(cdr(x), nil) == nil) ? TRUE : nil;
}

Cons *_list(Cons *x) {
    Cons *q = nil;

    for (Cons *p = cdr(x); p != nil; p = cdr(p)) {
        q = cons(q, car(p));
    }

    return q;
}

void var_to_user(Cons *p) {
    if (p != nil) {
        int t = _type(p);

        if (t == VAR) {
            if (_type(car(p)) == FUSER) {
                rplact(p, FUSER);
            }
        } else if (t == LIST) {
            var_to_user(car(p));
            var_to_user(cdr(p));
        }
    }
}

void var_to_atom(Cons *p) {
    if (p != nil) {
        int t = _type(p);

        if ((t != LIST && !isfunc(t)) || t == FUSER) {
            rplact(p, SYMBOL);
        } else {
            var_to_atom(car(p));
            var_to_atom(cdr(p));
        }
    }
}

Cons *read() {
    switch (read_token()) {
        case LPAREN: {
            _getc();
            Cons *p = cons(read(), read());
            rplact(p, LIST);
            return p;
        }

        case ALPHA:
            return cons(read_symbol(), read());

        case QUOTED: {
            Cons *p = cons(read_symbol(), read());
            rplaca(p, cons(car(p), cons(car(cdr(p)), nil)));
            rplacd(p, cdr(cdr(p)));
            return p;
        }

        case DIGIT:
            return cons(read_number(), read());

        case RPAREN:
            _getc();
            return nil;

        case EOL:
        case EOT:
        case ERR:
            _getc();
            return nil;
    }
}

void print(Cons *p) {
    if (p == nil) {
        return;
    }

    int t = _type(p);

    if (t == NUMBER) {
        _putn(_number(p));
    } else if (t == SYMBOL) {
        _puts(_symbol(car(p)));
    } else if (_type(car(p)) == LIST) {
        _putc('(');
        print(car(p));
        _putc(')');
        print(cdr(p));
    } else if (t == LIST) {
        print(car(p));
        print(cdr(p));
    } else {
        _puts("\033[31m" "?" "\033[0m");
    }
}

Cons *eval(Cons *x, Cons *env) {
    if (x == nil) {
        return nil;
    }

    int t = _type(x);

    if (t == VAR) {
        return cdr(lookup(env, _symbol(car(x))));
    }
    if (t == NUMBER) {
        return x;
    }
    if (t == LABL) {
        return nil;
    }

    switch (_type(car(x))) {
        case T:
            return TRUE;

        case NIL:
            return nil;

        case QUOTE:
            var_to_atom(car(cdr(x)));
            return car(cdr(x));

        case FCAR: return car(eval(cdr(x), env));
        case FCDR: return cdr(eval(cdr(x), env));

        case FATOM:
            return atom(eval(cdr(x), env));

        case EQP: return eq(eval(car(cdr(x)), env), eval(cdr(cdr(x)), env));
        case NULLP: return eq(eval(car(cdr(x)), env), nil);

        case FCONS:
            return cons(eval(car(cdr(x)), env), eval(cdr(cdr(x)), env));

        case FLIST:
            return _list(x);

        case COND:
            return evalcond(cdr(x), env);

        case FSETQ: {
            Cons *p = eval(cdr(cdr(x)), env);
            rplacd(lookup(env, _symbol(car(car(cdr(x))))), p);
            return p;
        }

        case DEFUN: {
            Cons *p = car(car(cdr(x)));
            rplact(p, FUSER);
            rplacd(p, cdr(cdr(x)));
            var_to_user(cdr(cdr(cdr(x))));
            return nil;
        }

        case FUSER: {
            Cons *p = cdr(car(car(x))); // p is statement list
            return eval(car(cdr(p)), pairargs(car(p), evalargs(cdr(x), env), env, false));
        }

        case FAPPLY:
        case FUNCALL: {
            Cons *p = eval(car(cdr(x)), env); // func name
            t = _type(car(p));
            if (isfunc(t)) {
                p = cons(p, cdr(cdr(x)));
                if (t == FUSER) {
                    rplact(car(p), FUSER);
                }
                Cons *q = eval(p, env);
                rplact(car(p), t);
                return q;
            }
            return nil;
        }

        case FEVAL: {
            Cons *p = eval(cdr(x), env);
            if (_type(p) == SYMBOL) {
                return cdr(lookup(env, _symbol(car(p))));
            }
            return eval(p, env);
        }

        case FPRINT:
            print(eval(car(cdr(x)), env));
            _putc('\n');
            return nil;

        case FREAD:
            return read();

        case FAND: return _and(x);
        case FOR: return _or(x);
        case FNOT: return _not(x);

        case FPLUS:
        case FDIFF:
        case FTIMES:
        case FQUOT:
        case GREATERP:
        case LESSP:
            return arith(car(x), eval(car(cdr(x)), env), eval(cdr(cdr(x)), env));

        case FADD1:
        case FSUB1:
            return arith(car(x), eval(car(cdr(x)), env), nil);

        case ZEROP: {
            Cons *p = eval(car(cdr(x)), env);
            return (_number(p) == 0) ? TRUE : nil;
        }

        case NUMBERP: {
            t = _type(eval(car(cdr(x)), env));
            return (t == NUMBER) ? TRUE : nil;
        }

        case PROG:
            return evalprog(x, env);

        case GO:
            return cdr(car(car(cdr(x))));

        case RETRN:
            progon = false;
            return eval(cdr(x), env);

        case LIST:
            if (cdr(x) == nil) {
                return eval(car(x), env);
            }
            return cons(eval(car(x), env), eval(cdr(x), env));

        case VAR:
            return cdr(lookup(env, _symbol(car(car(x)))));

        case NUMBER:
            return car(x);

#if FEATURE_BLE
        case BLEUART:
            bleuart();
            return nil;
#endif

#if DEVICE_ANALOGIN
        case ZOOLOG:
            zoolog();
            return nil;
#endif
    }

    return nil;
}

void repl() {
    Cons *p = nil;
    bool cont = false;
    bool oops = false;

    while (true) {
        if (!cont && !oops) {
            _puts("\033[31m" "ζ " "\033[32m" "=> " "\033[0m");
            _flush();
        }

        switch (read_token()) {
            case LPAREN:
                _getc();
                p = eval(read(), ENV);
                cont = true;
                break;

            case ALPHA:
                p = cdr(car(read_symbol()));
                cont = true;
                break;

            case QUOTED:
            case RPAREN:
            case DIGIT:
            case ERR:
                _getc();
                _puts("\033[33m" "oops!" "\033[0m" "\n");
                p = nil;
                oops = true;
                break;

            case EOL:
                _getc();
                if (!oops) {
                    if (p == nil) {
                        _puts("nil");
                    } else {
                        print(cons(p, nil));
                    }
                    _putc('\n');
                }
                p = nil;
                cont = false;
                oops = false;
                break;

            case EOT:
                _getc();
                return;
        }
    }
}

Cons *def(const char *name, int t) {
    Cons *p = declare(name);
    rplact(p, t);
    return p;
}

int main() {
    TRUE = cons(def("t", T), nil);
    rplact(TRUE, SYMBOL);

    def("'", QUOTE); def("quote", QUOTE);
    def("add1", FADD1); def("inc", FADD1);
    def("and", FAND);
    def("apply", FAPPLY);
    def("atom", FATOM);
    def("car", FCAR); def("first", FCAR);
    def("cdr", FCDR); def("next", FCDR);
    def("cond", COND);
    def("cons", FCONS);
    def("defun", DEFUN); def("defn", DEFUN);
    def("diff", FDIFF); def("-", FDIFF);
    def("eq", EQP); def("=", EQP);
    def("eval", FEVAL);
    def("funcall", FUNCALL);
    def("go", GO);
    def("greaterp", GREATERP); def(">", GREATERP);
    def("lessp", LESSP); def("<", LESSP);
    def("nil", NIL);
    def("not", FNOT);
    def("null", NULLP); def("nil?", NULLP);
    def("numberp", NUMBERP); def("number?", NUMBERP);
    def("or", FOR);
    def("plus", FPLUS); def("+", FPLUS);
    def("print", FPRINT);
    def("prog", PROG);
    def("quot", FQUOT); def("/", FQUOT);
    def("read", FREAD);
    def("return", RETRN);
    def("rplaca", FREPLACA);
    def("rplacd", FREPLACD);
    def("setq", FSETQ);
    def("sub1", FSUB1); def("dec", FSUB1);
    def("times", FTIMES); def("*", FTIMES);
    def("zerop", ZEROP); def("zero?", ZEROP);

#if FEATURE_BLE
    def("bleuart", BLEUART);
#endif
#if DEVICE_ANALOGIN
    def("zoolog", ZOOLOG);
#endif

    repl();

    return 0;
}
