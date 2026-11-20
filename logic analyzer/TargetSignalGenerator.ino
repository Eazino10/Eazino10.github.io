#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_ADDR 0x3C
#define SDA_PIN      2      // I2C SDA
#define SCL_PIN      4      // I2C SCL
#define PWM_PIN     18      // PWM Output
#define TX_PIN       1      // UART TX0

// Simulated SPI Pins (Non-conflicting)
#define SPI_CS_PIN   13
#define SPI_SCK_PIN  19
#define SPI_MOSI_PIN 27

// KY-040 Rotary Encoder Pins
#define ENC_CLK_PIN  5
#define ENC_DT_PIN   15
#define ENC_SW_PIN   23

Adafruit_SSD1306 oled(128, 64, &Wire, -1);

// Configurable Parameters adjustable via Rotary Encoder
const uint32_t BAUD_OPTIONS[] = { 4800, 9600, 19200, 38400, 57600, 115200 };
uint8_t baudIndex = 1; // Default 9600

const uint32_t PWM_FREQ_OPTIONS[] = { 500, 1000, 2000, 5000 };
uint8_t pwmIndex = 1;  // Default 1 kHz

const uint8_t PWM_DUTY_OPTIONS[] = { 10, 25, 50, 75, 90 };
uint8_t dutyIndex = 2; // Default 50%

uint8_t menuSelection = 0; // 0 = Baud Rate, 1 = PWM Freq, 2 = PWM Duty Cycle
uint32_t frameCounter = 0;

// Encoder state tracking
int lastClkState;
unsigned long lastButtonPress = 0;
unsigned long lastTaskTime = 0;
const unsigned long TASK_INTERVAL = 200;

void setupEncoder() {
    pinMode(ENC_CLK_PIN, INPUT_PULLUP);
    pinMode(ENC_DT_PIN, INPUT_PULLUP);
    pinMode(ENC_SW_PIN, INPUT_PULLUP);
    lastClkState = digitalRead(ENC_CLK_PIN);
}

void setupSPI() {
    pinMode(SPI_CS_PIN, OUTPUT);
    pinMode(SPI_SCK_PIN, OUTPUT);
    pinMode(SPI_MOSI_PIN, OUTPUT);
    digitalWrite(SPI_CS_PIN, HIGH);
    digitalWrite(SPI_SCK_PIN, LOW);
    digitalWrite(SPI_MOSI_PIN, LOW);
}

void updatePWM() {
    uint32_t freq = PWM_FREQ_OPTIONS[pwmIndex];
    uint8_t dutyPct = PWM_DUTY_OPTIONS[dutyIndex];
    // Map percentage (0-100) to 8-bit resolution (0-255)
    uint32_t dutyVal = (dutyPct * 255) / 100;

    ledcAttach(PWM_PIN, freq, 8);
    ledcWrite(PWM_PIN, dutyVal);
}

void transmitSPISensorBurst() {
    digitalWrite(SPI_CS_PIN, LOW);
    delayMicroseconds(5);

    uint8_t cmd = 0x80 | 0x2A;
    for (int i = 7; i >= 0; i--) {
        digitalWrite(SPI_MOSI_PIN, (cmd >> i) & 0x01);
        digitalWrite(SPI_SCK_PIN, HIGH);
        delayMicroseconds(2);
        digitalWrite(SPI_SCK_PIN, LOW);
        delayMicroseconds(2);
    }

    uint8_t dummyResponse = 0xA5;
    for (int i = 7; i >= 0; i--) {
        digitalWrite(SPI_MOSI_PIN, (dummyResponse >> i) & 0x01);
        digitalWrite(SPI_SCK_PIN, HIGH);
        delayMicroseconds(2);
        digitalWrite(SPI_SCK_PIN, LOW);
        delayMicroseconds(2);
    }

    digitalWrite(SPI_CS_PIN, HIGH);
}

void updateDisplay() {
    oled.clearDisplay();
    oled.setCursor(0, 0);
    oled.setTextSize(1);
    oled.println(F("ESP32 WORKBENCH GEN"));
    oled.drawFastHLine(0, 9, 128, SSD1306_WHITE);

    oled.setCursor(0, 12);
    oled.printf("Frame: #%lu\n", frameCounter);

    // Menu Item 0: Baud Rate
    oled.setCursor(0, 24);
    if (menuSelection == 0) oled.print(F(">")); else oled.print(F(" "));
    oled.printf(" Baud: %lu\n", BAUD_OPTIONS[baudIndex]);

    // Menu Item 1: PWM Frequency
    oled.setCursor(0, 36);
    if (menuSelection == 1) oled.print(F(">")); else oled.print(F(" "));
    oled.printf(" Freq: %u Hz\n", PWM_FREQ_OPTIONS[pwmIndex]);

    // Menu Item 2: PWM Duty Cycle
    oled.setCursor(0, 48);
    if (menuSelection == 2) oled.print(F(">")); else oled.print(F(" "));
    oled.printf(" Duty: %u%%\n", PWM_DUTY_OPTIONS[dutyIndex]);

    oled.display();
}

void handleEncoder() {
    int currentClkState = digitalRead(ENC_CLK_PIN);

    if (currentClkState != lastClkState && currentClkState == LOW) {
        if (digitalRead(ENC_DT_PIN) != currentClkState) {
            // Clockwise turn
            if (menuSelection == 0) baudIndex = (baudIndex + 1) % 6;
            else if (menuSelection == 1) pwmIndex = (pwmIndex + 1) % 4;
            else dutyIndex = (dutyIndex + 1) % 5;
        } else {
            // Counter-clockwise turn
            if (menuSelection == 0) baudIndex = (baudIndex == 0) ? 5 : baudIndex - 1;
            else if (menuSelection == 1) pwmIndex = (pwmIndex == 0) ? 3 : pwmIndex - 1;
            else dutyIndex = (dutyIndex == 0) ? 4 : dutyIndex - 1;
        }
        
        updatePWM();
        updateDisplay(); // Refresh display instantly on turn
    }
    lastClkState = currentClkState;

    // Detect push button press to cycle menu focus (0 -> 1 -> 2 -> 0)
    if (digitalRead(ENC_SW_PIN) == LOW) {
        if (millis() - lastButtonPress > 300) {
            menuSelection = (menuSelection + 1) % 3;
            lastButtonPress = millis();
            updateDisplay();
        }
    }
}

void setup() {
    setupEncoder();
    setupSPI();

    Serial.begin(BAUD_OPTIONS[baudIndex]);
    updatePWM();

    Wire.begin(SDA_PIN, SCL_PIN);
    oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
    oled.setTextColor(SSD1306_WHITE);

    updateDisplay();
}

void loop() {
    // 1. Instantly poll encoder rotation and clicks with zero delay blocking
    handleEncoder();

    // 2. Background periodic tasks (serial burst, SPI simulation, frame count increment)
    if (millis() - lastTaskTime >= TASK_INTERVAL) {
        lastTaskTime = millis();
        frameCounter++;

        static uint32_t lastBaud = 0;
        if (BAUD_OPTIONS[baudIndex] != lastBaud) {
            Serial.end();
            Serial.begin(BAUD_OPTIONS[baudIndex]);
            lastBaud = BAUD_OPTIONS[baudIndex];
        }

        Serial.printf("MSG:%lu\n", frameCounter);
        Serial.flush();

        transmitSPISensorBurst();
        updateDisplay();
    }
}
