#include "WiFi.h"

#define TX2 17
#define RX2 16

// [설정] 스캔할 채널 목록
const int targetChannels[] = {1, 3, 6, 9, 11};
const int channelCount = sizeof(targetChannels) / sizeof(targetChannels[0]);

// 상태 관리 변수
int currentListIndex = 0; 
bool isScanning = false; // 현재 스캔 중인지 여부 확인

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RX2, TX2);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setScanMethod(WIFI_FAST_SCAN);

  Serial.println("\n[ESP32] Ready. Waiting for 'GETRSSI' command...");
  
  // setup에서는 스캔을 시작하지 않습니다. (명령 대기)
}

void loop() {
  // 1. UART 명령 수신 확인 (스캔 중이 아닐 때만 명령 받음)
  if (!isScanning && Serial2.available()) {
    String input = Serial2.readStringUntil('\n'); // 줄바꿈 문자까지 읽기
    input.trim(); // \r, \n, 공백 제거 (중요)

    if (input == "GETRSSI") {
      Serial.println("Command Received: GETRSSI -> Starting Scan Cycle");
      
      // 상태 초기화 및 스캔 시작
      isScanning = true;
      currentListIndex = 0;
      
      // 첫 번째 채널 스캔 시작
      WiFi.scanNetworks(true, false, false, 100, targetChannels[0]);
    }
  }

  // 2. 스캔 상태 처리 (스캔 중일 때만 동작)
  if (isScanning) {
    int n = WiFi.scanComplete();

    if (n >= 0) { // 스캔 완료됨
      // 데이터 전송
      for (int i = 0; i < n; ++i) {
        String payload = "SSID:" + WiFi.SSID(i) + 
                         ",RSSI:" + String(WiFi.RSSI(i)) + 
                         ",Ch:" + String(WiFi.channel(i)) + "\n";
        Serial2.print(payload);
        Serial.print(payload);
      }
      
      WiFi.scanDelete(); // 메모리 정리

      // 다음 채널로 넘어갈 준비
      currentListIndex++;

      // [판단] 다음 채널이 남았는가? vs 끝났는가?
      if (currentListIndex < channelCount) {
        // 아직 남음 -> 다음 채널 스캔 (이어달리기)
        int nextCh = targetChannels[currentListIndex];
        WiFi.scanNetworks(true, false, false, 100, nextCh);
      } 
      else {
        // 모든 채널 스캔 끝 -> 종료 처리
        isScanning = false;
        Serial.println("--- Cycle Finished. Waiting for next command ---");
        
        // (선택사항) 라즈베리 파이에게 끝났음을 알리는 신호가 필요하면 아래 주석 해제
        // Serial2.println("SCAN_COMPLETE"); 
      }
    }
  }
}