#include <WiFi.h>

const int LED_PIN = 23;   // 네가 연결한 LED 핀 (없으면 -1로 두고 관련 코드 지워도 됨)

void setup() {
  Serial.begin(115200);
  delay(1000);            // 시리얼 안정화 대기

  Serial.println();
  Serial.println("=== ESP32 WiFi RSSI Scanner ===");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // WiFi를 STA 모드로 설정하고, 기존 연결 끊기
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(500);
}

void loop() {
  Serial.println();
  Serial.println("Scanning WiFi networks...");

  digitalWrite(LED_PIN, HIGH);  // 스캔 시작 표시 (LED ON)

  // 주변 AP 스캔 (블로킹, 몇 초 걸릴 수 있음)
  int n = WiFi.scanNetworks();
  digitalWrite(LED_PIN, LOW);   // 스캔 끝 (LED OFF)

  Serial.print("Scan done. Found ");
  Serial.print(n);
  Serial.println(" networks.");

  if (n <= 0) {
    Serial.println("No networks found :(");
  } else {
    for (int i = 0; i < n; ++i) {
      Serial.print(i + 1);
      Serial.print(") SSID: ");
      Serial.print(WiFi.SSID(i));     // AP 이름

      Serial.print("  RSSI: ");
      Serial.print(WiFi.RSSI(i));     // 신호 세기(dBm, 0에 가까울수록 세다)
      Serial.print(" dBm");

      Serial.print("  CH: ");
      Serial.print(WiFi.channel(i));  // 채널 번호

      Serial.print("  ENC: ");
      Serial.println(WiFi.encryptionType(i)); // 암호화 타입 (숫자로 나옴)
      
      delay(10); // 출력 정리용
    }
  }

  // 메모리 정리 (선택 사항이지만 습관처럼 써 두면 좋음)
  WiFi.scanDelete();

  // 5초마다 한 번씩 스캔
  delay(5000);
}
