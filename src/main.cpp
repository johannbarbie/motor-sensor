#include <Arduino.h>
#include <ActisenseReader.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <N2kMessages.h>
#include <NMEA2000_esp32.h>
#include <ReactESP.h>
#include <Wire.h>
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>
#include "elapsedMillis.h"

using namespace reactesp;

ReactESP app;

// GPIO Pin Definitions
#define MEASURE_PIN 15
#define DISCHARGE_PIN 33
#define CHARGE_PIN 5
#define CAN_RX_PIN GPIO_NUM_34
#define CAN_TX_PIN GPIO_NUM_32
#define SDA_PIN 16
#define SCL_PIN 17
#define OLED_RESET -1  // Reset pin # (or -1 if sharing Arduino reset pin)

#define TIMEOUT 5000000  // Timeout in microseconds (5 seconds)
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define RECOVERY_RETRY_MS 1000  // How long to attempt CAN bus recovery

TwoWire *i2c;

Stream *read_stream = &Serial;
Stream *forward_stream = &Serial;

tActisenseReader actisense_reader;

Adafruit_SSD1306 *display;

tNMEA2000 *nmea2000;

int num_n2k_messages = 0;
int num_actisense_messages = 0;
double engine_temp = 0;
elapsedMillis time_since_last_can_rx = 0;

// Time after which we should reboot if we haven't received any CAN messages
#define MAX_RX_WAIT_TIME_MS 30000

void ToggleLed() {
  static bool led_state = false;
  digitalWrite(LED_BUILTIN, led_state);
  led_state = !led_state;
}

void HandleStreamN2kMsg(const tN2kMsg &message) {
  // N2kMsg.Print(&Serial);
  num_n2k_messages++;
  time_since_last_can_rx = 0;
  ToggleLed();
}

void HandleStreamActisenseMsg(const tN2kMsg &message) {
  // N2kMsg.Print(&Serial);
  num_actisense_messages++;
  ToggleLed();
  nmea2000->SendMsg(message);
}

String can_state;

void RecoverFromCANBusOff() {
  // This recovery routine first discussed in
  // https://www.esp32.com/viewtopic.php?t=5010 and also implemented in
  // https://github.com/wellenvogel/esp32-nmea2000
  static bool recovery_in_progress = false;
  static elapsedMillis recovery_timer;
  if (recovery_in_progress && recovery_timer < RECOVERY_RETRY_MS) {
    return;
  }
  recovery_in_progress = true;
  recovery_timer = 0;
  // Abort transmission
  MODULE_CAN->CMR.B.AT = 1;
  // read SR after write to CMR to settle register changes
  (void)MODULE_CAN->SR.U;

  // Reset error counters
  MODULE_CAN->TXERR.U = 127;
  MODULE_CAN->RXERR.U = 0;
  // Release Reset mode
  MODULE_CAN->MOD.B.RM = 0;
}

void PollCANStatus() {
  // CAN controller registers are SJA1000 compatible.
  // Bus status value 0 indicates bus-on; value 1 indicates bus-off.
  unsigned int bus_status = MODULE_CAN->SR.B.BS;

  switch (bus_status) {
    case 0:
      can_state = "RUNNING";
      break;
    case 1:
      can_state = "BUS-OFF";
      // try to automatically recover
      RecoverFromCANBusOff();
      break;
  }
}


double measure_temperature() {
    digitalWrite(CHARGE_PIN, LOW);
    digitalWrite(DISCHARGE_PIN, HIGH);
    delay(100);  // Ensure capacitor is fully discharged
    digitalWrite(DISCHARGE_PIN, LOW);

    digitalWrite(CHARGE_PIN, HIGH);
    uint64_t startTime = esp_timer_get_time();
    while (digitalRead(MEASURE_PIN) == 0) {
        if ((esp_timer_get_time() - startTime) > TIMEOUT) {
            Serial.println("Timeout reached!");
            return -1.0;  // Return -1 on timeout
        }
    }
    uint64_t endTime = esp_timer_get_time();
    uint64_t elapsedTime = endTime - startTime;

    double temperature = 0.00894 * elapsedTime - 71.85;
    return temperature;
}

void sendNMEA2000Temperature(double temperature) {
    tN2kMsg N2kMsg;
    SetN2kEngineDynamicParam(N2kMsg,
                            0,  // instance of a single engine is always 0
                            N2kDoubleNA,  // oil pressure
                            temperature, 
                            N2kDoubleNA,
                            N2kDoubleNA,  // alternator voltage
                            N2kDoubleNA,  // fuel rate
                            N2kDoubleNA,  // engine hours
                            N2kDoubleNA,  // engine coolant pressure
                            N2kDoubleNA,  // engine fuel pressure
                            N2kInt8NA,    // engine load
                            N2kInt8NA,    // engine torque
                            (tN2kEngineDiscreteStatus1)0,
                            (tN2kEngineDiscreteStatus2)0);
    nmea2000->SendMsg(N2kMsg);
    // Serial.print("Sent temperature: ");
    // Serial.println(temperature);
}

void setup() {
    // setup serial output
    Serial.begin(115200);
    delay(100);

    // toggle the LED pin at rate of 1 Hz
    pinMode(LED_BUILTIN, OUTPUT);
    app.onRepeatMicros(1e6 / 1, []() { ToggleLed(); });

    // setup meoasurement
    pinMode(MEASURE_PIN, INPUT);
    pinMode(DISCHARGE_PIN, OUTPUT);
    pinMode(CHARGE_PIN, OUTPUT);
    digitalWrite(DISCHARGE_PIN, LOW);
    digitalWrite(CHARGE_PIN, HIGH);

    // instantiate the NMEA2000 object
    nmea2000 = new tNMEA2000_esp32(CAN_TX_PIN, CAN_RX_PIN);

    // input the NMEA 2000 messages

    // Reserve enough buffer for sending all messages. This does not work on small
    // memory devices like Uno or Mega
    nmea2000->SetN2kCANSendFrameBufSize(250);
    nmea2000->SetN2kCANReceiveFrameBufSize(250);

    // Set Product information
    nmea2000->SetProductInformation(
        "20210331",  // Manufacturer's Model serial code (max 32 chars)
        103,         // Manufacturer's product code
        "SH-ESP32 NMEA 2000 USB GW",  // Manufacturer's Model ID (max 33 chars)
        "0.1.0.0 (2021-03-31)",  // Manufacturer's Software version code (max 40
                                // chars)
        "0.0.3.1 (2021-03-07)"   // Manufacturer's Model version (max 24 chars)
    );
    // Set device information
    nmea2000->SetDeviceInformation(
        1,    // Unique number. Use e.g. Serial number.
        130,  // Device function=Analog to NMEA 2000 Gateway. See codes on
                // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
        25,   // Device class=Inter/Intranetwork Device. See codes on
            // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
        2046  // Just choosen free from code list on
                // http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
    );

    nmea2000->SetForwardStream(forward_stream);
    nmea2000->SetMode(tNMEA2000::N2km_ListenAndNode);
    // nmea2000->SetForwardType(tNMEA2000::fwdt_Text); // Show bus data in clear
    // text
    nmea2000->SetForwardOwnMessages(false);  // do not echo own messages.
    nmea2000->SetMsgHandler(HandleStreamN2kMsg);
    nmea2000->Open();

    // No need to parse the messages at every single loop iteration; 1 ms will do
    app.onRepeat(1, []() {
        PollCANStatus();
        nmea2000->ParseMessages();
        actisense_reader.ParseMessages();
    });

    app.onRepeat(100, []() {
        if (time_since_last_can_rx > MAX_RX_WAIT_TIME_MS) {
        // No CAN messages received in a while; reboot
        esp_task_wdt_init(1, true);
        esp_task_wdt_add(NULL);
        while (true) {
            // wait for watchdog to trigger
        }
        }
    });

    // initialize the display
    i2c = new TwoWire(0);
    i2c->begin(SDA_PIN, SCL_PIN);
    display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, i2c, -1);
    if (!display->begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 allocation failed"));
    }
    delay(100);
    display->setRotation(2);
    display->clearDisplay();
    display->display();

    // update results

    app.onRepeat(1000, []() {
        display->clearDisplay();
        display->setTextSize(1);
        display->setCursor(0, 0);
        display->setTextColor(SSD1306_WHITE);
        display->printf("SH-ESP32 N2K USB GW\n");
        display->printf("CAN: %s\n", can_state.c_str());
        display->printf("Uptime: %lu\n", millis() / 1000);
        display->printf("RX: %d\n", num_n2k_messages);
        display->printf("TX: %d\n", num_actisense_messages);
        display->printf("Engine temp: %f\n", engine_temp);

        display->display();

        num_n2k_messages = 0;
        num_actisense_messages = 0;
    });

    app.onRepeat(5000, []() {
        double temperature = measure_temperature();
        engine_temp = temperature;
        if (temperature != -1) {  // Valid measurement
            sendNMEA2000Temperature(temperature + 273.15);  // Convert to Kelvin
        } else {
            Serial.println("Failed to measure temperature.");
        }
    });

    Serial.println("Setup complete");
}


void loop() {
    app.tick();
}
