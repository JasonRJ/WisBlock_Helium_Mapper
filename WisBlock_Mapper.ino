#include "SX126x-RAK4630.h"
#include "LoRaWan-RAK4630.h"
#include "SparkFun_Ublox_Arduino_Library.h"

#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE // Maximum size of scheduler events
#define SCHED_QUEUE_SIZE 60                                       // Maximum number of events in the scheduler queue

#define LORAWAN_APP_DATA_BUFF_SIZE 128  // Size of the data to be transmitted
#define LORAWAN_APP_TX_DUTYCYCLE 7500   // Data transmission cycle (value in ms)
#define APP_TX_DUTYCYCLE_RND 500        // Random delay for application data transmission cycle (value in ms)
#define JOINREQ_NBTRIALS 20             // Number of trys to join LoRa network

static void lorawan_has_joined_handler(void);

static void lorawan_rx_handler(lmh_app_data_t *app_data);

static void lorawan_confirm_class_handler(DeviceClass_t Class);

static void send_lora_frame(void);

static uint32_t timers_init(void);

TimerEvent_t appTimer;                                                        // tranfer timer instance
static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];            // application data buffer
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0}; // application data structure
static lmh_param_t lora_param_init = {LORAWAN_ADR_OFF, LORAWAN_DEFAULT_DATARATE, LORAWAN_PUBLIC_NETWORK,
                                      JOINREQ_NBTRIALS, LORAWAN_DEFAULT_TX_POWER, LORAWAN_DUTYCYCLE_OFF};
static lmh_callback_t lora_callbacks = {BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
                                        lorawan_rx_handler, lorawan_has_joined_handler, lorawan_confirm_class_handler};

#include <bluefruit.h>

void initBLE();

extern bool bleUARTisConnected;
extern BLEUart bleuart;

// Configure the three OTAA keys here or in an external file and #include that file
//uint8_t nodeDeviceEUI[8] = SECRET_NODEDEVICEEUI;
//uint8_t nodeAppEUI[8] = SECRET_NODEAPPEUI;
//uint8_t nodeAppKey[16] = SECRET_NODEAPPKEY;
//uint8_t nodeDeviceEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//uint8_t nodeAppEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//uint8_t nodeAppKey[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// Set OTAA keys outside of the project in my case in the directory called OTAA_Keys where the Arduino sketchbooks are stored
//#include "../../../OTAA_Keys/myMapperOTAAkeys"
#include "../../../OTAA_Keys/WisBlock-B997"
//#include "../../../OTAA_Keys/WisBlock-B99E"

const byte greenLEDGPIO = 35; // RAK5005-O Baseboard with RAK4631 Core has 2 user-definable LEDs, Green on GPIO 35, Blue on GPIO 36
volatile byte greenLEDstate = LOW;
const byte blueLEDGPIO = 36; // RAK5005-O Baseboard with RAK4631 Core has 2 user-definable LEDs, Green on GPIO 35, Blue on GPIO 36
volatile byte blueLEDstate = LOW;
const byte PPSinterruptGPIO = 17; // RAK1910 GNSS Module (u-blox MAX-7Q) 1PPS on GPIO 17
const byte GNSSpowerGPIO = 34; // RAK1910 GNSS Module (u-blox MAX-7Q) VCC 3.3V via 3V3_S Powered by GPIO 34

const bool OTAA = true;
static uint32_t err_code = 0;
static uint32_t send_total = 0;
static uint32_t send_success = 0;
static uint32_t send_failure = 0;
SFE_UBLOX_GPS GNSS;


void blink() {
    blueLEDstate = !blueLEDstate;
    digitalWrite(blueLEDGPIO, blueLEDstate);
}

void setup() {
    Serial.begin(115200);
    time_t serialStart = millis();
    while (!Serial) {
        if ((millis() - serialStart) > 6000) {
            break;
        }
    }
    Serial.println("==========================================================================");
    Serial.println("RAK WisBlock LoRaWAN OTAA Mapper");
    Serial.println("-  WisBlock Base Board - RAK5005-O");
    Serial.println("-  WisBlock Core MCU/LPWAN Module - RAK4631");
    Serial.println("   - Nordic nRF52840 MCU with BLE 5.0 and Semtech SX126x LoRa® transceiver");
    Serial.println("-  WisBlock GNSS Sensor Module - RAK1910 (U-Blox MAX-7Q Engine)");
    Serial.println("");
    Serial.println("-  Using SparkFun u-blox UBX GNSS Library");
    Serial.println("==========================================================================");

    static uint8_t BoardUniqueId[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    BoardGetUniqueId(BoardUniqueId);
    Serial.printf("Board Unique Id %X%X%X%X%X%X%X%X\n", BoardUniqueId[0], BoardUniqueId[1], BoardUniqueId[2],
                  BoardUniqueId[3], BoardUniqueId[4], BoardUniqueId[5], BoardUniqueId[6], BoardUniqueId[7]);

    // RAK5005-O Baseboard with RAK4631 Core has 2 user-definable LEDs, Green on GPIO 35, Blue on GPIO 36
    pinMode(greenLEDGPIO, OUTPUT);
    digitalWrite(greenLEDGPIO, greenLEDstate);
    pinMode(blueLEDGPIO, OUTPUT);
    digitalWrite(blueLEDGPIO, blueLEDstate);

    // RAK1910 GNSS Module (u-blox MAX-7Q) 1PPS on GPIO 17
    pinMode(PPSinterruptGPIO, INPUT);
    attachInterrupt(digitalPinToInterrupt(PPSinterruptGPIO), blink, RISING);

    // RAK1910 GNSS Module (u-blox MAX-7Q) VCC 3.3V via 3V3_S Powered by GPIO 34
    pinMode(GNSSpowerGPIO, OUTPUT);
    //digitalWrite(GNSSpowerGPIO, 0);
    //delay(125);
    digitalWrite(GNSSpowerGPIO, 1);
    //delay(125);

    //GNSS.enableDebugging(Serial, false);

    Serial1.begin(9600);
    while (!Serial1);
    GNSS.begin(Serial1);
    Serial.println("GNSS UART Initalized");

    //GNSS.factoryDefault();
    //GNSS.saveConfiguration();
    //GNSS.factoryReset();
    //GNSS.saveConfiguration();
    //GNSS.hardReset();
    //GNSS.setSerialRate(38400);

    GNSS.setPortOutput(COM_PORT_UART1, COM_TYPE_UBX);
    GNSS.setPortInput(COM_PORT_UART1, COM_TYPE_UBX);
    GNSS.setUART1Output(COM_TYPE_UBX);

    //GNSS.enableMessage(UBX_CLASS_NAV, UBX_NAV_PVT, COM_PORT_UART1, 1);
    //GNSS.configureMessage(UBX_CLASS_NAV, UBX_NAV_PVT, COM_PORT_UART1, 1);
    //GNSS.disableMessage(UBX_CLASS_NAV, UBX_NAV_PVT, COM_PORT_UART1);

    GNSS.saveConfiguration();

    pinMode(30, OUTPUT);
    digitalWrite(30, HIGH);
    initBLE();

    // Initialize Scheduler and timer
    err_code = timers_init();
    if (err_code != 0) {
        Serial.printf("timers_init failed - % d\n", err_code);
    }

    // Initialize LoRa chip.
    err_code = lora_rak4630_init();
    if (err_code != 0) {
        Serial.printf("lora_hardware_init failed - % d\n", err_code);
    }

    // Setup the EUIs and Key
    lmh_setDevEui(nodeDeviceEUI);
    lmh_setAppEui(nodeAppEUI);
    lmh_setAppKey(nodeAppKey);

    // Initialize LoRaWan
    err_code = lmh_init(&lora_callbacks, lora_param_init, OTAA);
    if (err_code != 0) {
        Serial.printf("lmh_init failed - % d\n", err_code);
    }

    lmh_datarate_set(DR_1, LORAWAN_ADR_OFF);
    lmh_tx_power_set(TX_POWER_0);

    // Start Join procedure
    digitalWrite(greenLEDGPIO, HIGH);
    lmh_join();
}

void loop() {
    Radio.IrqProcess();
}

static void lorawan_has_joined_handler(void) {
    Serial.printf("Joined Helium Network in OTAA Mode with DevAddr %X\n", __builtin_bswap32(LoRaMacGetOTAADevId()));
    digitalWrite(greenLEDGPIO, LOW);
    lmh_class_request(CLASS_A);
    lorawan_confirm_class_handler(CLASS_A);
    TimerSetValue(&appTimer, LORAWAN_APP_TX_DUTYCYCLE);
    TimerStart(&appTimer);
}

static void lorawan_rx_handler(lmh_app_data_t *app_data) {
    Serial.printf("LoRa Packet received on port %d, size: %d, rssi: %d, snr: %d, data: %s\n",
                  app_data->port, app_data->buffsize, app_data->rssi, app_data->snr, app_data->buffer);

    switch (app_data->port) {
        case 11:
            // Send a packet to port 11 to toggle the green LED
            Serial.println("Data received on port 11");
            if (app_data->buffsize > 0) {
                Serial.println("Toggle Green LED State");
                greenLEDstate = !greenLEDstate;
                digitalWrite(greenLEDGPIO, greenLEDstate);
            }
            break;

        case 111:
            // Send a 1 character message on port 111 to reset u-blox GNSS
            Serial.println("Data received on port 111");
            if (app_data->buffsize == 1) {
                Serial.printf("Single character received '%s', %x", app_data->buffer[0], app_data->buffer[0]);
                Serial.println();
                switch (app_data->buffer[0]) {
                    case 0x30: // character 0
                        Serial.println("hardReset");
                        GNSS.hardReset();
                        break;

                    case 0x31: // character 1
                        Serial.println("factoryReset");
                        GNSS.factoryDefault();
                        GNSS.saveConfiguration();
                        GNSS.factoryReset();
                        GNSS.saveConfiguration();
                        break;

                    default:
                        break;
                }
            }
            break;

        default:
            break;
    }
}

static void lorawan_confirm_class_handler(DeviceClass_t Class) {
    Serial.printf("Switch to Class %c Complete\n", "ABC"[Class]);
    m_lora_app_data.buffsize = 0;
    m_lora_app_data.port = LORAWAN_APP_PORT;
    lmh_send(&m_lora_app_data, LMH_UNCONFIRMED_MSG);
}

static void send_lora_frame(void) {
    if (lmh_join_status_get() != LMH_SET) {
        Serial.println("Not joined to a network, skip sending frame");
        digitalWrite(greenLEDGPIO, HIGH);
        lmh_join(); // try to join network
        return;
    }
    digitalWrite(greenLEDGPIO, HIGH);
    uint32_t i = 0;
    int32_t data = 0;
    m_lora_app_data.port = 1;
    if (GNSS.getTimeValid()) {
        data = (int32_t)(GNSS.latitude);
        Serial.print("Location: ");
        Serial.print(data);
        m_lora_app_data.buffer[i++] = data >> 24;
        m_lora_app_data.buffer[i++] = data >> 16;
        m_lora_app_data.buffer[i++] = data >> 8;
        m_lora_app_data.buffer[i++] = data;
        data = (int32_t)(GNSS.longitude);
        Serial.print(", ");
        Serial.print(data);
        m_lora_app_data.buffer[i++] = data >> 24;
        m_lora_app_data.buffer[i++] = data >> 16;
        m_lora_app_data.buffer[i++] = data >> 8;
        m_lora_app_data.buffer[i++] = data;
        data = (int32_t)(GNSS.altitudeMSL);
        Serial.print(", Altitude: ");
        Serial.print(data);
        m_lora_app_data.buffer[i++] = data >> 24;
        m_lora_app_data.buffer[i++] = data >> 16;
        m_lora_app_data.buffer[i++] = data >> 8;
        m_lora_app_data.buffer[i++] = data;
        data = (uint32_t)(GNSS.horizontalAccuracy);
        Serial.print(", Accuracy: ");
        Serial.print(data);
        m_lora_app_data.buffer[i++] = data >> 24;
        m_lora_app_data.buffer[i++] = data >> 16;
        m_lora_app_data.buffer[i++] = data >> 8;
        m_lora_app_data.buffer[i++] = data;
        data = (uint8_t)(GNSS.SIV);
        Serial.print(", Satellites: ");
        Serial.print(data);
        m_lora_app_data.buffer[i++] = data;
        data = (uint16_t)(GNSS.pDOP);
        Serial.print(", HDOP: ");
        Serial.print(data);
        m_lora_app_data.buffer[i++] = data >> 8;
        m_lora_app_data.buffer[i++] = data;
        data = (int32_t)(GNSS.groundSpeed);
        Serial.print(", Speed: ");
        Serial.print(data);
        m_lora_app_data.buffer[i++] = data >> 24;
        m_lora_app_data.buffer[i++] = data >> 16;
        m_lora_app_data.buffer[i++] = data >> 8;
        m_lora_app_data.buffer[i++] = data;
        Serial.println();
    }
    m_lora_app_data.buffsize = i;

    lmh_datarate_set(DR_1, LORAWAN_ADR_OFF);
    lmh_tx_power_set(TX_POWER_0);
    lmh_error_status error = lmh_send(&m_lora_app_data, LMH_UNCONFIRMED_MSG);
    if (error == LMH_SUCCESS) {
        send_success++;
    } else {
        send_failure++;
    }
    send_total++;
    Serial.printf("lmh_send return code % d, success count % d, fail count % d, total count % d\n", error, send_success,
                  send_failure, send_total);
    digitalWrite(greenLEDGPIO, LOW);
}

static void tx_lora_periodic_handler(void) {
    TimerSetValue(&appTimer, LORAWAN_APP_TX_DUTYCYCLE);
    TimerStart(&appTimer);
    Serial.println("Sending frame ...  ");
    send_lora_frame();
}

static uint32_t timers_init(void) {
    appTimer.timerNum = 3;
    TimerInit(&appTimer, tx_lora_periodic_handler);
    return 0;
}
