#include "WiFi.h"

#define TX2 17
#define RX2 16

// [설정] 스캔할 채널 목록 (원하는 채널만 넣으세요)
const int targetChannels[] = {1, 3, 6, 9, 11};
// 배열의 크기 계산 (자동)
const int channelCount = sizeof(targetChannels) / sizeof(targetChannels[0]);

int currentListIndex = 0; // 현재 스캔 중인 리스트의 순번 (0 ~ 4)

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RX2, TX2);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  // [속도 향상 1] Active Scan 모드 (공유기를 빨리 찾음)
  WiFi.setScanMethod(WIFI_FAST_SCAN);

  Serial.println("\n[ESP32] 5-Channel Hopping Mode Started (1,3,6,9,11)");
  
  // 첫 번째 채널(1번) 스캔 시작
  // scanNetworks(async, show_hidden, passive, max_ms_per_chan, channel)
  int firstCh = targetChannels[currentListIndex];
  WiFi.scanNetworks(true, false, false, 100, firstCh); 
}

void loop() {
  // 스캔 상태 확인
  int n = WiFi.scanComplete();

  // n >= 0 이면 스캔이 완료된 것임
  if (n >= 0) {
    // Serial.printf("\n--- Scan Finished (Ch %d): %d networks found ---\n", targetChannels[currentListIndex], n);

    for (int i = 0; i < n; ++i) {
      // [수정] 채널 번호(Ch)도 같이 보내면 디버깅에 좋습니다. 필요 없으면 지우세요.
      String payload = "SSID:" + WiFi.SSID(i) + 
                       ",RSSI:" + String(WiFi.RSSI(i)) + 
                       ",Ch:" + String(WiFi.channel(i)) + "\n";
      
      Serial2.print(payload); // 라즈베리 파이 드라이버로 전송
      Serial.print(payload);  // PC 시리얼 모니터 확인용
      
      // delay(2); // 필요시 주석 해제
    }

    // 스캔 결과 삭제 (메모리 확보)
    WiFi.scanDelete();

    // [핵심 로직] 다음 채널 번호 선택
    currentListIndex++;
    // 목록 끝에 도달하면 다시 처음(0번 인덱스)으로
    if (currentListIndex >= channelCount) {
      currentListIndex = 0;
    }

    // 다음 타겟 채널 가져오기
    int nextCh = targetChannels[currentListIndex];

    // 지체 없이 즉시 다음 채널 스캔 시작! (채널당 최대 100ms)
    WiFi.scanNetworks(true, false, false, 100, nextCh);
    
    // Serial.printf(">> Next scan started for Ch %d...\n", nextCh);
  }

  // 다른 작업 가능
}