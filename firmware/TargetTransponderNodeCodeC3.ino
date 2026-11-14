#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

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

void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
    if (len == sizeof(DiagnosticPayload)) {
        DiagnosticPayload packet;
        memcpy(&packet, incomingData, sizeof(packet));

        if (packet.packetType == PING_REQ) {
            // Auto-register Master Node as a peer if not already present
            if (!esp_now_is_peer_exist(info->src_addr)) {
                esp_now_peer_info_t peerInfo = {};
                memcpy(peerInfo.peer_addr, info->src_addr, 6);
                peerInfo.channel = 1; // Match Wi-Fi Channel 1
                peerInfo.encrypt = false;
                
                esp_err_t addErr = esp_now_add_peer(&peerInfo);
                if (addErr != ESP_OK) {
                    Serial.printf("[WARN] Auto-add peer failed: %d\n", addErr);
                }
            }

            // Prepare ACK Response
            packet.packetType = PING_ACK;
            packet.senderFreeHeap = ESP.getFreeHeap();
            packet.rssi = WiFi.RSSI();

            // Send ACK reply back to Master's MAC address
            esp_err_t sendStatus = esp_now_send(info->src_addr, (uint8_t *)&packet, sizeof(packet));
            Serial.printf("[RX PING #%d] ACK Send Status: %d\n", packet.sequenceNumber, sendStatus);
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== ESP32-C3 TRANSPONDER ACTIVE ===");

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    
    // Lock explicitly to Wi-Fi Channel 1
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

    Serial.print("C3 MAC Address: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        Serial.println("[FAIL] ESP-NOW Init Failed");
        return;
    }

    esp_now_register_recv_cb(OnDataRecv);
    Serial.println("[OK] C3 Listening and ready on Channel 1...");
}

void loop() {
    delay(100);
}
