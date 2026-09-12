// ------------------------------------------------------------
// OUTDOOR ESP32-C3 WEATHER NODE
// Wind Speed (GPIO9), Rain Gauge (GPIO3), Wind Vane ADC (GPIO0)
// LoRa RYLR890 UART: RXD=GPIO20, TXD=GPIO21
// Long-range LoRa: SF12, BW125, CR4/5
// Sends <OUT> packets to Mega every 5 seconds
// ------------------------------------------------------------

#include <Arduino.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

// ---------------- PIN DEFINITIONS ----------------
#define PIN_WIND     10    // Wind speed reed switch (interrupt)
#define PIN_RAIN     3     // Rain gauge reed switch (interrupt)
#define PIN_VANE     0     // Wind vane ADC input

#define PIN_LORA_TX  4     // ESP32 TX → LoRa RXD
#define PIN_LORA_RX  5     // ESP32 RX → LoRa TXD

// ---------------- WIND VARIABLES ----------------
volatile uint32_t windPulses = 0;
float windSpeedMPH = 0.0;
float windSpeedBuffer[10] = {0};
int windSpeedIndex = 0;
const float MPH_PER_PULSE = 2.25;

// Gust buffer (16 samples)
float gustBuffer[16];
int gustIndex = 0;
float windGustMPH = 0.0;

// ---------------- RAIN VARIABLES ----------------
volatile uint32_t rainTips = 0;
float rainRate = 0.0;
float dailyRain = 0.0;
uint32_t rainHistory[15] = {0};
int rainIndex = 0;
uint32_t lastRainMinute = 0;
uint32_t tipsOneMinuteAgo = 0;

const float INCHES_PER_TIP = 0.01;

// ---------------- LORA SERIAL ----------------
HardwareSerial LoRaSerial(1);

// ------------- LORA flag ---------------
bool sendStatusReply = false;

// ---------------- INTERRUPTS ----------------
void IRAM_ATTR windISR() {
    windPulses = windPulses + 1;
}

volatile uint32_t lastRainInterrupt = 0;

void IRAM_ATTR rainISR() {

    uint32_t now = millis();

    if (now - lastRainInterrupt > 500) {  // debounce
        rainTips = rainTips + 1;
        lastRainInterrupt = now;
    }
}

int mapWindDirection(int adc)
{
    if (adc < 600)   return 315;  // NW
    if (adc < 1000)  return 0;    // N
    if (adc < 1600)  return 45;   // NE
    if (adc < 2350)  return 270;  // W
    if (adc < 3100)  return 90;   // E
    if (adc < 3750)  return 225;  // SW
    if (adc < 4090)  return 180;  // S
    return 135;                   // SE
}

// ---------------- LORA COMMAND SENDER ----------------
void sendLoRaCmd(String cmd) {
    LoRaSerial.println(cmd);
    delay(50);

    while (LoRaSerial.available()) {
        String resp = LoRaSerial.readString();
        Serial.print("LoRa Response: ");
        Serial.println(resp);
    }
}

// ---------------- LORA INITIALIZATION ----------------
void initLoRa() {
    sendLoRaCmd("AT+IPR=115200");
    delay(200);

    sendLoRaCmd("AT");
    sendLoRaCmd("AT+RESET");
    delay(500);

    sendLoRaCmd("AT+MODE=0");

    sendLoRaCmd("AT+ADDRESS=0");
    sendLoRaCmd("AT+NETWORKID=0");
    sendLoRaCmd("AT+BAND=915000000");
    sendLoRaCmd("AT+PARAMETER=12,7,1,4");

    Serial.println("LoRa RYLR890 initialized.");
}

// ---------------- SETUP ----------------
// void setup() {
//     Serial.begin(115200);
//     delay(200);

//     // LoRa UART
//     LoRaSerial.begin(115200, SERIAL_8N1, PIN_LORA_RX, PIN_LORA_TX);

//     initLoRa();

//     // Wind speed interrupt
//     pinMode(PIN_WIND, INPUT_PULLUP);
//     attachInterrupt(
//         digitalPinToInterrupt(PIN_WIND),
//         windISR,
//         FALLING
//     );

//     // Rain gauge interrupt
//     pinMode(PIN_RAIN, INPUT_PULLUP);
//     attachInterrupt(
//         digitalPinToInterrupt(PIN_RAIN),
//         rainISR,
//         FALLING
//     );

//     // Wind vane ADC
//     pinMode(PIN_VANE, INPUT);

//     Serial.println("Outdoor ESP32-C3 Weather Node Ready.");
// }

#include "esp_system.h"

void setup()
{
    Serial.begin(115200);

    delay(3000);

    Serial.println();
    Serial.println();
    Serial.println("*** BOOTED ***");    Serial.printf("\n\n===== BOOT =====\n");

    Serial.printf("Reset reason = %d\n", esp_reset_reason());

    Serial.println("STEP 1 -- before LoRaSerial.begin()");

    // LoRa UART
    LoRaSerial.begin(115200, SERIAL_8N1, PIN_LORA_RX, PIN_LORA_TX);

    Serial.println("STEP 2 -- before calling initLoRa()");

    initLoRa();

    Serial.println("STEP 3 -- after calling initLoRa()");

    pinMode(PIN_WIND, INPUT_PULLUP);

    pinMode(PIN_RAIN, INPUT_PULLUP);
    Serial.println("STEP 4");

    attachInterrupt(
        digitalPinToInterrupt(PIN_WIND),
        windISR,
        FALLING
    );
    Serial.println("STEP 5");

    attachInterrupt(
        digitalPinToInterrupt(PIN_RAIN),
        rainISR,
        FALLING
    );

    // Set wind calculation buffers:
    for (int i = 0; i < 10; i++)
    {
        windSpeedBuffer[i] = 0.0;
    }

    for (int i = 0; i < 16; i++)
    {
        gustBuffer[i] = 0.0;
    }

    // Set up the rain history array:
    for (int i = 0; i < 15; i++)
    {
        rainHistory[i] = rainTips;
    }

    // Watchdog init
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 30000,
        .idle_core_mask = 0,
        .trigger_panic = true
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);

    Serial.println("Watchdog enabled (30 sec)");

    Serial.println("STEP 6");
    Serial.println("SETUP COMPLETE");
    Serial.println("Outdoor ESP32-C3 Weather Node Ready.");
}

// ---------------- MAIN LOOP ----------------
void loop() {
    static bool firstPass = true;

    if (firstPass)
    {
        Serial.println("LOOP STARTED");
        firstPass = false;
    }
    
    static uint32_t lastCalc = 0;
    static uint32_t lastSend = 0;

    uint32_t now = millis();

    // Watchdog reset in each loop...
    esp_task_wdt_reset();

    // Get packets SENT FROM THE MEGA:
    while (LoRaSerial.available())
    {
        String line = LoRaSerial.readStringUntil('\n');
        line.trim();

        if (line.length())
        {
            Serial.println();
            Serial.print("LoRa RX: ");
            Serial.println(line);
            Serial.println();

            if (line.indexOf("<STATUS?>") >= 0)
            {
                sendStatusReply = true;
            }

            if (line.indexOf("<REBOOT>") >= 0)
            {
                Serial.println();
                Serial.println("Remote reboot requested");

                // Tell the Mega we're about to reboot
                String reply = "<REBOOTING>";

                String cmd =
                    "AT+SEND=0," +
                    String(reply.length()) +
                    "," +
                    reply;

                sendLoRaCmd(cmd);

                Serial.println("REBOOTING reply sent");

                Serial.flush();

                delay(500);

                ESP.restart();
            }
        }
    }

    if (sendStatusReply)
    {
        sendStatusReply = false;

        uint32_t uptimeSec = millis() / 1000;

        String reply =
            "<STATUS>," +
            String(uptimeSec);

        String cmd =
            "AT+SEND=0," +
            String(reply.length()) +
            "," +
            reply;

        sendLoRaCmd(cmd);

        Serial.println();
        Serial.print("STATUS reply sent: ");
        Serial.println(reply);
        Serial.println();
    }
    
    // ---------------- WIND & RAIN CALC EVERY 1 SECOND ----------------
    if (now - lastCalc >= 1000) {
        lastCalc = now;

        // Wind speed
        uint32_t pulses = windPulses;
        windPulses = 0;

        // Serial.print("PIN_WIND state = ");
        // Serial.println(digitalRead(PIN_WIND));

        float currentWindMPH = pulses * MPH_PER_PULSE;

        if (currentWindMPH > 100.0)
        {
            Serial.print("INVALID WIND SAMPLE: ");
            Serial.print(currentWindMPH);
            Serial.print(" MPH (");
            Serial.print(pulses);
            Serial.println(" pulses)");

            currentWindMPH = 0.0;
        }

        if (pulses > 50)
        {
            Serial.print("LARGE PULSE COUNT: ");
            Serial.println(pulses);
        }

        // Store newest sample
        windSpeedBuffer[windSpeedIndex] = currentWindMPH;
        windSpeedIndex = (windSpeedIndex + 1) % 10;

        // Compute 10-second average
        float totalWind = 0.0;

        for (int i = 0; i < 10; i++)
        {
            totalWind += windSpeedBuffer[i];
        }

        windSpeedMPH = totalWind / 10.0;

        // Gust buffer update
        gustBuffer[gustIndex] = currentWindMPH;
        gustIndex = (gustIndex + 1) % 16;

        // Compute gust
        windGustMPH = 0.0;
        for (int i = 0; i < 16; i++) {
            if (gustBuffer[i] > windGustMPH)
                windGustMPH = gustBuffer[i];
        }

Serial.print("Pulses = ");
Serial.println(pulses);

Serial.print("Current Wind = ");
Serial.println(currentWindMPH);

Serial.print("Average Wind = ");
Serial.println(windSpeedMPH);

Serial.print("Gust = ");
Serial.println(windGustMPH);

        // Rain calculations

        // Serial.print("PIN_RAIN state = ");
        // Serial.println(digitalRead(PIN_RAIN));

        // Serial.print("Rain tips = ");
        // Serial.println(rainTips);

        uint32_t tipsNow = rainTips;

        // Daily rain accumulation
        dailyRain = tipsNow * INCHES_PER_TIP;

        // Update rain-rate history once per minute
        if (now - lastRainMinute >= 60000UL)
        {
            lastRainMinute = now;

            rainHistory[rainIndex] = tipsNow;

            rainIndex = (rainIndex + 1) % 15;

            uint32_t oldestTips = rainHistory[rainIndex];

            float rainLast15Min =
                (tipsNow - oldestTips) * INCHES_PER_TIP;
            float rainLastMinute =
                (tipsNow - oldestTips) * INCHES_PER_TIP;

            rainRate = rainLast15Min * 4.0;   // 15 min -> in/hr
        }

        // Serial.print("Rain Rate: ");
        // Serial.println(rainRate, 2);

        // Serial.print("Daily Rain: ");
        // Serial.println(dailyRain, 2);

        // Wind direction
        int adc = analogRead(PIN_VANE);
        int windDirDeg = mapWindDirection(adc);
    }

    // ---------------- SEND LORA PACKET EVERY 5 SECONDS ----------------
    if (now - lastSend >= 5000) {
        lastSend = now;

        int adc = analogRead(PIN_VANE);
        int windDirDeg = mapWindDirection(adc);

        String packet = "<OUT>";

        packet += String(windSpeedMPH, 2);
        packet += ",";

        packet += String(windGustMPH, 2);
        packet += ",";

        packet += String(windDirDeg);
        packet += ",";

        packet += String(rainRate, 3);
        packet += ",";

        packet += String(dailyRain, 3);
        packet += ",";

        packet += String(rainTips);

        uint32_t uptimeSec = millis() / 1000;
        packet += ",";
        packet += String(uptimeSec);

        packet += "</OUT>";
        //Serial.print("Payload length = ");
        //Serial.println(packet.length());

        //String cmd = "AT+SEND=0," + String(packet.length()) + ",\"" + packet + "\"";
        String cmd = "AT+SEND=0," + String(packet.length()) + "," + packet;

        //Serial.print("CMD length = ");
        //Serial.println(cmd.length());
        //Serial.println(cmd);
        Serial.println();
        Serial.print("UptimeSec = ");
        Serial.println(uptimeSec);

        sendLoRaCmd(cmd);

        //Serial.print("Millis: ");
        //Serial.println(millis());

        //Serial.println("Sent LoRa Packet:");
        //Serial.println(packet);
    }
}
