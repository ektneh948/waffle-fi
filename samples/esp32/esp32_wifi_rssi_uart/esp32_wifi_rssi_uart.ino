#include <WiFi.h>

// 라즈베리파이에 연결할 UART2
HardwareSerial SerialPi(2);

const int LED_PIN    = 23;  // 네가 연결한 LED 핀
const int UART2_RX   = 16;  // ESP32 GPIO16 = RX2
const int UART2_TX   = 17;  // ESP32 GPIO17 = TX2

void setup() {
  Serial.begin(115200);                           // PC(USB)용 시리얼
  SerialPi.begin(115200, SERIAL_8N1, UART2_RX, UART2_TX); // 라즈베리용 UART2
  delay(1000);

  Serial.println();
  Serial.println("=== ESP32 WiFi RSSI Scanner ===");
  SerialPi.println("=== ESP32 WiFi RSSI Scanner ===");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(500);
}

void loop() {
  Serial.println();
  Serial.println("Scanning WiFi networks...");

  SerialPi.println();
  SerialPi.println("SCAN_START");

  digitalWrite(LED_PIN, HIGH);  // 스캔 시작 표시

  int n = WiFi.scanNetworks();

  digitalWrite(LED_PIN, LOW);   // 스캔 끝

  Serial.print("Scan done. Found ");
  Serial.print(n);
  Serial.println(" networks.");

  // 라즈베리 쪽에는 간단한 형식으로 전송
  // 형식:
  // FOUND,<개수>
  // AP,<index>,<ssid>,<rssi>,<channel>,<enc>
  SerialPi.print("FOUND,");
  SerialPi.println(n);

  if (n <= 0) {
    Serial.println("No networks found :(");
    SerialPi.println("NO_NETWORK");
  } else {
    for (int i = 0; i < n; ++i) {
      // PC 모니터용 (보기 편하게)
      Serial.print(i + 1);
      Serial.print(") SSID: ");
      Serial.print(WiFi.SSID(i));
      Serial.print("  RSSI: ");
      Serial.print(WiFi.RSSI(i));
      Serial.print(" dBm  CH: ");
      Serial.print(WiFi.channel(i));
      Serial.print("  ENC: ");
      Serial.println(WiFi.encryptionType(i));

      // 라즈베리용: 파싱 쉬운 한 줄짜리
      // 예) AP,0,mywifi,-45,1,3
      SerialPi.print("AP,");
      SerialPi.print(i);
      SerialPi.print(",");
      SerialPi.print(WiFi.SSID(i));
      SerialPi.print(",");
      SerialPi.print(WiFi.RSSI(i));
      SerialPi.print(",");
      SerialPi.print(WiFi.channel(i));
      SerialPi.print(",");
      SerialPi.println(WiFi.encryptionType(i));

      delay(10);
    }
  }

  SerialPi.println("SCAN_END");

  WiFi.scanDelete();
  delay(5000);  // 5초마다 스캔
}
