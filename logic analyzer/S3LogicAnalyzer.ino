#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <esp_timer.h>
#include <soc/gpio_struct.h>

// 1.44" ST7735 Hardware SPI Pins
#define TFT_CS    10
#define TFT_MOSI  11
#define TFT_SCLK  12
#define TFT_DC    13
#define TFT_RST   14

// 4-Channel Simultaneous Protocol Probe Pins (Contiguous GPIOs 4, 5, 6, 7)
#define CH0_PIN   4   // Target I2C SDA (GPIO 2)
#define CH1_PIN   5   // Target I2C SCL (GPIO 4)
#define CH2_PIN   6   // Target PWM (GPIO 18)
#define CH3_PIN   7   // Target UART TX0 (GPIO 1)
#define CTRL_BTN  0   // Onboard BOOT button: Short Tap=HOLD/LIVE, Long Press=Zoom

#define BUFFER_SIZE 512
#define REFRESH_INTERVAL_MS 500  // Balanced 0.5s pace: steady, smooth, and readable

struct LogicSample {
    uint32_t timestampUs;
    uint8_t pinState; // bit0=CH0, bit1=CH1, bit2=CH2, bit3=CH3
};

LogicSample captureBuffer[BUFFER_SIZE];
uint16_t sampleCount = 0;
uint32_t totalSpanUs = 0;

// Simultaneous Telemetry State
int16_t detectedI2CAddr = -1;
uint32_t detectedBaud = 0;
char detectedUARTText[10] = "";
float pwmFreqHz = 0.0f;
uint8_t pwmDutyPct = 0;

bool isPaused = false;
uint8_t zoomIndex = 0;
const uint8_t ZOOM_LEVELS[3] = {1, 4, 16};
unsigned long lastFrameTime = 0;

Adafruit_ST7735 tft = Adafruit_ST7735(&SPI, TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 canvas(128, 128);

const uint16_t CH_COLORS[4] = {
    ST77XX_YELLOW,  // CH0: I2C SDA
    ST77XX_CYAN,    // CH1: I2C SCL
    ST77XX_GREEN,   // CH2: PWM
    ST77XX_MAGENTA  // CH3: UART TX0
};

// Atomic single-cycle read of GPIO 4, 5, 6, and 7 simultaneously
inline uint8_t readAllChannelsFast() {
    return (uint8_t)((GPIO.in >> 4) & 0x0F);
}

// Reconstruct CH3 (bit 3) logic state at any microsecond timestamp
uint8_t getCH3AtTime(uint32_t targetUs) {
    uint8_t state = (captureBuffer[0].pinState >> 3) & 0x01;
    for (uint16_t i = 1; i < sampleCount; i++) {
        if (captureBuffer[i].timestampUs > targetUs) break;
        state = (captureBuffer[i].pinState >> 3) & 0x01;
    }
    return state;
}

void decodeUARTFromCH3() {
    detectedBaud = 0;
    detectedUARTText[0] = '\0';
    if (sampleCount < 10) return;

    uint32_t minPulseUs = 0xFFFFFFFF;
    uint32_t lastEdgeUs = captureBuffer[0].timestampUs;
    uint8_t prevBit = (captureBuffer[0].pinState >> 3) & 0x01;

    for (uint16_t i = 1; i < sampleCount; i++) {
        uint8_t currBit = (captureBuffer[i].pinState >> 3) & 0x01;
        if (currBit != prevBit) {
            uint32_t dt = captureBuffer[i].timestampUs - lastEdgeUs;
            if (dt >= 6 && dt < minPulseUs) {
                minPulseUs = dt;
            }
            lastEdgeUs = captureBuffer[i].timestampUs;
            prevBit = currBit;
        }
    }

    uint32_t bitUs = 0;
    if (minPulseUs >= 6 && minPulseUs <= 12) {
        detectedBaud = 115200; bitUs = 9;
    } else if (minPulseUs >= 14 && minPulseUs <= 22) {
        detectedBaud = 57600;  bitUs = 17;
    } else if (minPulseUs >= 23 && minPulseUs <= 32) {
        detectedBaud = 38400;  bitUs = 26;
    } else if (minPulseUs >= 44 && minPulseUs <= 62) {
        detectedBaud = 19200;  bitUs = 52;
    } else if (minPulseUs >= 88 && minPulseUs <= 120) {
        detectedBaud = 9600;   bitUs = 104;
    } else if (minPulseUs >= 180 && minPulseUs <= 240) {
        detectedBaud = 4800;   bitUs = 208;
    } else {
        return;
    }

    uint8_t charIdx = 0;
    uint32_t nextSearchUs = 0;

    for (uint16_t i = 1; i < sampleCount && charIdx < 7; i++) {
        uint32_t tEdge = captureBuffer[i].timestampUs;
        if (tEdge < nextSearchUs) continue;

        uint8_t pBit = (captureBuffer[i - 1].pinState >> 3) & 0x01;
        uint8_t cBit = (captureBuffer[i].pinState >> 3) & 0x01;

        if (pBit == 1 && cBit == 0) {
            uint32_t stopSampleUs = tEdge + (bitUs * 9) + (bitUs / 2);
            if (stopSampleUs > totalSpanUs) break;

            if (getCH3AtTime(tEdge + (bitUs / 2)) == 0 && getCH3AtTime(stopSampleUs) == 1) {
                uint8_t rxByte = 0;
                for (uint8_t b = 0; b < 8; b++) {
                    uint32_t sampleT = tEdge + (bitUs * (b + 1)) + (bitUs / 2);
                    if (getCH3AtTime(sampleT)) {
                        rxByte |= (1 << b);
                    }
                }
                if (rxByte >= 0x20 && rxByte <= 0x7E) {
                    detectedUARTText[charIdx++] = (char)rxByte;
                    detectedUARTText[charIdx] = '\0';
                }
                nextSearchUs = tEdge + (bitUs * 9);
            }
        }
    }

    if (charIdx == 0) {
        detectedBaud = 0;
    }
}

void analyzeCapturedSignals() {
    detectedI2CAddr = -1;
    detectedBaud = 0;
    detectedUARTText[0] = '\0';
    pwmFreqHz = 0.0f;
    pwmDutyPct = 0;

    if (sampleCount < 6) return;

    // 1. Decode I2C Address on CH0 (SDA) & CH1 (SCL)
    bool inFrame = false;
    uint8_t bitIndex = 0;
    uint8_t currentByte = 0;

    for (uint16_t i = 1; i < sampleCount; i++) {
        uint8_t prevSDA = captureBuffer[i - 1].pinState & 0x01;
        uint8_t prevSCL = (captureBuffer[i - 1].pinState >> 1) & 0x01;
        uint8_t currSDA = captureBuffer[i].pinState & 0x01;
        uint8_t currSCL = (captureBuffer[i].pinState >> 1) & 0x01;

        if (prevSCL == 1 && currSCL == 1 && prevSDA == 1 && currSDA == 0) {
            inFrame = true;
            bitIndex = 0;
            currentByte = 0;
        } else if (prevSCL == 1 && currSCL == 1 && prevSDA == 0 && currSDA == 1) {
            inFrame = false;
        } else if (inFrame && prevSCL == 0 && currSCL == 1) {
            if (bitIndex < 8) {
                currentByte = (currentByte << 1) | currSDA;
                bitIndex++;
            } else {
                if (detectedI2CAddr < 0) detectedI2CAddr = (currentByte >> 1);
                bitIndex = 0;
                currentByte = 0;
            }
        }
    }

    // 2. Decode UART ASCII on CH3
    decodeUARTFromCH3();

    // 3. Measure PWM Frequency & Duty Cycle on CH2
    uint32_t lastRiseUs = 0;
    uint32_t highStartUs = 0;
    uint64_t sumPeriodUs = 0;
    uint64_t sumHighUs = 0;
    uint16_t fullCycles = 0;

    for (uint16_t i = 1; i < sampleCount; i++) {
        uint8_t prevC2 = (captureBuffer[i - 1].pinState >> 2) & 0x01;
        uint8_t currC2 = (captureBuffer[i].pinState >> 2) & 0x01;
        uint32_t t = captureBuffer[i].timestampUs;

        if (prevC2 == 0 && currC2 == 1) {
            if (lastRiseUs > 0 && highStartUs > lastRiseUs) {
                uint32_t period = t - lastRiseUs;
                uint32_t highTime = highStartUs - lastRiseUs;
                if (period > 0) {
                    sumPeriodUs += period;
                    sumHighUs += highTime;
                    fullCycles++;
                }
            }
            lastRiseUs = t;
        } else if (prevC2 == 1 && currC2 == 0) {
            highStartUs = t;
        }
    }

    if (fullCycles >= 2 && sumPeriodUs > 0) {
        float avgPeriodUs = (float)sumPeriodUs / fullCycles;
        pwmFreqHz = 1000000.0f / avgPeriodUs;
        pwmDutyPct = (uint8_t)((sumHighUs * 100ULL) / sumPeriodUs);
    }
}

void renderFrameToTFT(const char* statusText, uint16_t badgeColor) {
    canvas.fillScreen(ST77XX_BLACK);

    // Top Header Bar
    canvas.fillRect(0, 0, 128, 15, 0x18E3);
    canvas.setTextSize(1);

    // Status Badge (LIVE / HOLD)
    canvas.fillRoundRect(2, 2, 31, 11, 2, badgeColor);
    canvas.setCursor(4, 4);
    canvas.setTextColor(ST77XX_BLACK);
    canvas.print(statusText);

    // Zoom Multiplier Badge
    uint8_t zoom = ZOOM_LEVELS[zoomIndex];
    canvas.fillRoundRect(35, 2, 23, 11, 2, 0x39E7);
    canvas.setCursor(38, 4);
    canvas.setTextColor(ST77XX_WHITE);
    canvas.printf("%dx", zoom);

    // Top-Right Protocol Readout
    bool hasI2C = (detectedI2CAddr >= 0);
    bool hasUART = (detectedBaud > 0 && detectedUARTText[0] != '\0');

    if (hasI2C && hasUART) {
        canvas.setCursor(60, 4);
        canvas.setTextColor(ST77XX_GREEN);
        canvas.printf("%02X", detectedI2CAddr);
        canvas.setCursor(74, 4);
        canvas.setTextColor(ST77XX_MAGENTA);
        canvas.printf(" %s", detectedUARTText);
    } else if (hasI2C) {
        canvas.setCursor(68, 4);
        canvas.setTextColor(ST77XX_GREEN);
        canvas.printf("I2C:0x%02X", detectedI2CAddr);
    } else if (hasUART) {
        canvas.setCursor(64, 4);
        canvas.setTextColor(ST77XX_MAGENTA);
        canvas.printf("U:%s", detectedUARTText);
    } else {
        canvas.setCursor(72, 4);
        canvas.setTextColor(0x8410);
        canvas.print("NO PROTO");
    }

    // Vertical Grid
    for (int x = 22; x <= 126; x += 26) {
        for (int y = 18; y < 114; y += 4) {
            canvas.drawPixel(x, y, 0x3186);
        }
    }

    // Channel Labels & Dividers
    for (int ch = 0; ch < 4; ch++) {
        int yTop = 18 + (ch * 24);
        canvas.drawFastHLine(0, yTop + 22, 128, 0x2104);
        canvas.setTextColor(CH_COLORS[ch]);
        canvas.setCursor(2, yTop + 7);
        canvas.printf("C%d", ch);
    }

    // Plot Digital Waveforms
    if (sampleCount >= 2 && totalSpanUs > 0) {
        const int xOrigin = 22;
        const int xWidth = 104;
        uint32_t viewSpanUs = totalSpanUs / zoom;
        if (viewSpanUs < 40) viewSpanUs = 40;

        for (int ch = 0; ch < 4; ch++) {
            int yHigh = 21 + (ch * 24);
            int yLow  = yHigh + 15;
            uint16_t color = CH_COLORS[ch];

            uint8_t prevBit = (captureBuffer[0].pinState >> ch) & 1;
            int prevX = xOrigin;

            for (uint16_t i = 1; i < sampleCount; i++) {
                uint32_t t = captureBuffer[i].timestampUs;
                int currX = xOrigin + (int)(((uint64_t)t * xWidth) / viewSpanUs);
                if (currX > xOrigin + xWidth) currX = xOrigin + xWidth;

                int yLevel = prevBit ? yHigh : yLow;
                if (currX > prevX) {
                    canvas.drawFastHLine(prevX, yLevel, currX - prevX, color);
                }

                uint8_t currBit = (captureBuffer[i].pinState >> ch) & 1;
                if (currBit != prevBit) {
                    canvas.drawFastVLine(currX, yHigh, yLow - yHigh + 1, color);
                    prevBit = currBit;
                }
                prevX = currX;
                if (currX >= xOrigin + xWidth) break;
            }

            if (prevX < xOrigin + xWidth) {
                int yFinal = prevBit ? yHigh : yLow;
                canvas.drawFastHLine(prevX, yFinal, (xOrigin + xWidth) - prevX, color);
            }
        }
    } else {
        canvas.setCursor(28, 62);
        canvas.setTextColor(ST77XX_RED);
        canvas.print("NO SIGNAL EDGE");
    }

    // Bottom Telemetry Footer
    canvas.fillRect(0, 115, 128, 13, 0x1082);
    canvas.setTextColor(ST77XX_WHITE);
    canvas.setCursor(2, 118);

    uint32_t shownSpan = (totalSpanUs > 0) ? (totalSpanUs / zoom) : 0;
    if (pwmFreqHz > 0.0f && detectedBaud > 0) {
        canvas.printf("%.1fms|%dHz|%luk", shownSpan / 1000.0f, (int)pwmFreqHz, detectedBaud / 1000UL);
    } else if (detectedBaud > 0) {
        canvas.printf("%.1fms | %lubd", shownSpan / 1000.0f, detectedBaud);
    } else if (pwmFreqHz >= 1000.0f) {
        canvas.printf("%.1fms | %.1fkHz %d%%", shownSpan / 1000.0f, pwmFreqHz / 1000.0f, pwmDutyPct);
    } else if (pwmFreqHz > 0.0f) {
        canvas.printf("%.1fms | %dHz %d%%", shownSpan / 1000.0f, (int)pwmFreqHz, pwmDutyPct);
    } else if (shownSpan < 1000) {
        canvas.printf("%luus | %d pts", shownSpan, sampleCount);
    } else {
        canvas.printf("%.1fms | %d pts", shownSpan / 1000.0f, sampleCount);
    }

    tft.drawRGBBitmap(0, 0, canvas.getBuffer(), 128, 128);
}

void captureSweep() {
    sampleCount = 0;

    // Free-running immediate oscilloscope capture
    uint8_t lastState = readAllChannelsFast();
    uint32_t t0 = (uint32_t)esp_timer_get_time();
    captureBuffer[0].timestampUs = 0;
    captureBuffer[0].pinState = lastState;
    sampleCount = 1;

    while (sampleCount < BUFFER_SIZE) {
        uint8_t currentState = readAllChannelsFast();
        uint32_t elapsed = (uint32_t)esp_timer_get_time() - t0;

        if (currentState != lastState) {
            captureBuffer[sampleCount].timestampUs = elapsed;
            captureBuffer[sampleCount].pinState = currentState;
            lastState = currentState;
            sampleCount++;
        }
        if (elapsed - captureBuffer[sampleCount - 1].timestampUs > 5000) {
            break;
        }
    }

    if (sampleCount < 2) {
        totalSpanUs = 0;
        detectedI2CAddr = -1;
        detectedBaud = 0;
        detectedUARTText[0] = '\0';
        pwmFreqHz = 0.0f;
        renderFrameToTFT("IDLE", ST77XX_RED);
        return;
    }

    totalSpanUs = captureBuffer[sampleCount - 1].timestampUs;
    analyzeCapturedSignals();
    renderFrameToTFT("LIVE", ST77XX_GREEN);
}

void setup() {
    Serial.begin(115200);

    pinMode(CH0_PIN, INPUT);
    pinMode(CH1_PIN, INPUT);
    pinMode(CH2_PIN, INPUT);
    pinMode(CH3_PIN, INPUT);
    pinMode(CTRL_BTN, INPUT_PULLUP);

    SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);
    tft.initR(INITR_144GREENTAB);
    tft.setSPISpeed(40000000);
    tft.setRotation(1);

    renderFrameToTFT("LIVE", ST77XX_GREEN);
}

void loop() {
    if (digitalRead(CTRL_BTN) == LOW) {
        unsigned long pressStart = millis();
        while (digitalRead(CTRL_BTN) == LOW) {
            delay(10);
        }
        unsigned long pressDuration = millis() - pressStart;

        if (pressDuration >= 450) {
            zoomIndex = (zoomIndex + 1) % 3;
        } else if (pressDuration >= 25) {
            isPaused = !isPaused;
            if (!isPaused) {
                lastFrameTime = 0;
            }
        }

        if (isPaused) {
            renderFrameToTFT("HOLD", ST77XX_YELLOW);
        } else {
            renderFrameToTFT("LIVE", ST77XX_GREEN);
        }
    }

    if (!isPaused && (millis() - lastFrameTime >= REFRESH_INTERVAL_MS)) {
        lastFrameTime = millis();
        captureSweep();
    }

    delay(5);
}
