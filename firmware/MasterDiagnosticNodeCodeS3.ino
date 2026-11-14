#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Target Transponder (ESP32-C3) MAC Address
uint8_t targetPeerMac[] = {0x70, 0x4B, 0xCA, 0x4A, 0x3B, 0x64};

enum PacketType : uint8_t {
    PING_REQ = 0x01,
    PING_ACK = 0x02,
    FAULT_ALERT = 0x0E
};

#pragma pack(push, 1)
struct DiagnosticPayload {
    uint8_t packetType;
    uint32_t sequenceNumber;
    uint32_t timestampMs;
    uint32_t senderFreeHeap;
    int8_t rssi;
    uint8_t statusFlags;
};
#pragma pack(pop)

// Dynamic 10-Packet Rolling Window Diagnostics
class NetworkDiagnostics {
private:
    uint32_t seqCounter;
    uint32_t lastRttMs;
    float avgRttMs;
    unsigned long lastAckTimestamp;
    
    // Rolling window buffer (1 = received, 0 = lost)
    static const int WINDOW_SIZE = 10;
    uint8_t window[WINDOW_SIZE];
    int windowIndex;

public:
    NetworkDiagnostics() 
        : seqCounter(0), lastRttMs(0), avgRttMs(0.0f), lastAckTimestamp(0), windowIndex(0) {
        for (int i = 0; i < WINDOW_SIZE; i++) window[i] = 0;
    }

    DiagnosticPayload createPingRequest(uint32_t currentHeap) {
        DiagnosticPayload packet;
        packet.packetType = PING_REQ;
        packet.sequenceNumber = ++seqCounter;
        packet.timestampMs = millis();
        packet.senderFreeHeap = currentHeap;
        packet.rssi = WiFi.RSSI();
        packet.statusFlags = 0x00;
        
        // Record pending packet (will be overwritten to 1 if ACK arrives)
        window[windowIndex] = 0;
        windowIndex = (windowIndex + 1) % WINDOW_SIZE;

        return packet;
    }

    void processAck(const DiagnosticPayload& ackPacket) {
        unsigned long now = millis();
        lastAckTimestamp = now;
        lastRttMs = now - ackPacket.timestampMs;
        
        if (avgRttMs == 0.0f) avgRttMs = (float)lastRttMs;
        else avgRttMs = (avgRttMs * 0.8f) + (lastRttMs * 0.2f);

        // Mark current window slot as successful
        int lastSlot = (windowIndex - 1 + WINDOW_SIZE) % WINDOW_SIZE;
        window[lastSlot] = 1;
    }

    bool isLinkActive() const {
        return (millis() - lastAckTimestamp < 2500) && (lastAckTimestamp > 0);
    }

    float getSlidingLossRate() const {
        if (!isLinkActive()) return 100.0f;
        
        int successCount = 0;
        for (int i = 0; i < WINDOW_SIZE; i++) {
            successCount += window[i];
        }
        return ((float)(WINDOW_SIZE - successCount) / WINDOW_SIZE) * 100.0f;
    }

    uint32_t getLastRtt() const { return isLinkActive() ? lastRttMs : 0; }
    float getAvgRtt() const { return isLinkActive() ? avgRttMs : 0.0f; }
};

NetworkDiagnostics netDiag;
unsigned long lastPingTime = 0;
const unsigned long PING_INTERVAL = 1000;
bool oledOK = false;

void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
    if (len == sizeof(DiagnosticPayload)) {
        DiagnosticPayload ack;
        memcpy(&ack, incomingData, sizeof(ack));
        if (ack.packetType == PING_ACK) {
            netDiag.processAck(ack);
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.println("\n=== GBB MASTER NODE STARTUP ===");

    // OLED Hardware Init (SDA: GPIO 8, SCL: GPIO 9)
    Wire.begin(8, 9);
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("[WARNING] OLED hardware not found at 0x3C.");
        oledOK = false;
    } else {
        Serial.println("[OK] OLED Display Active.");
        oledOK = true;
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("GBB DIAGNOSTIC");
        display.println("Starting WiFi...");
        display.display();
    }

    // Lock Wi-Fi to Channel 1
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    delay(100);

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("[FAIL] ESP-NOW Init Failed");
        return;
    }

    esp_now_register_recv_cb(OnDataRecv);

    // Register C3 Peer on Channel 1
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, targetPeerMac, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;

    esp_now_del_peer(targetPeerMac);
    esp_now_add_peer(&peerInfo);
}

void loop() {
    unsigned long currentMillis = millis();

    if (currentMillis - lastPingTime >= PING_INTERVAL) {
        lastPingTime = currentMillis;

        uint32_t freeHeap = ESP.getFreeHeap();
        DiagnosticPayload ping = netDiag.createPingRequest(freeHeap);

        esp_now_send(targetPeerMac, (uint8_t *)&ping, sizeof(ping));

        bool active = netDiag.isLinkActive();
        float lossRate = netDiag.getSlidingLossRate();
        uint32_t rtt = netDiag.getLastRtt();

        if (oledOK) {
            display.clearDisplay();
            display.setCursor(0, 0);
            display.println("== GBB NODE DIAGNOSTIC ==");
            
            if (active) {
                display.print("RTT:       "); display.print(rtt); display.println(" ms");
                display.print("Avg RTT:   "); display.print(netDiag.getAvgRtt(), 1); display.println(" ms");
                display.print("Loss Rate: "); display.print(lossRate, 1); display.println(" %");
                display.print("Free Heap: "); display.print(freeHeap / 1024); display.println(" KB");
                display.println("STATUS: [OK] LINK ACTIVE");
            } else {
                display.println("RTT:       0 ms");
                display.println("Avg RTT:   0.0 ms");
                display.println("Loss Rate: 100.0 %");
                display.print("Free Heap: "); display.print(freeHeap / 1024); display.println(" KB");
                display.println("STATUS: [FAIL] DISCONNECT");
            }
            display.display();
        }

        Serial.printf("Active: %d | RTT: %d ms | Loss: %.1f%%\n", active, rtt, lossRate);
    }
    
    delay(10);
}
