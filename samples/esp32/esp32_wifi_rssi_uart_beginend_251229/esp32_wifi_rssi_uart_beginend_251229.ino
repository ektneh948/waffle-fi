#include "WiFi.h"

#define TX2 17
#define RX2 16

const int targetChannels[] = {1, 3, 6, 9, 11};
const int channelCount = sizeof(targetChannels) / sizeof(targetChannels[0]);

int currentListIndex = 0;

static void start_scan(int ch) {
  WiFi.scanDelete();
  WiFi.scanNetworks(true, false, false, 100, ch);  // async scan
}

static void print_both(const String& s) {
  Serial.print(s);   // PC Serial Monitor
  Serial2.print(s);  // Pi(UART)
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RX2, TX2);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(200);

  // ✅ 라운드 시작
  print_both("BEGIN\n");

  start_scan(targetChannels[currentListIndex]);
}

void loop() {
  int n = WiFi.scanComplete();

  if (n == -1) return;  // scanning...

  if (n < 0) {          // 실패(-2 등) -> 같은 채널 재시작
    start_scan(targetChannels[currentListIndex]);
    return;
  }

  // ✅ SSID + RSSI만 전송 (채널/scan_id 없음)
  for (int i = 0; i < n; ++i) {
    String line = "SSID:" + WiFi.SSID(i) +
                  ",RSSI:" + String(WiFi.RSSI(i)) + "\n";
    print_both(line);
  }

  WiFi.scanDelete();

  currentListIndex++;

  // ✅ 5개 채널 라운드 끝나면 END 1번, 다음 라운드 BEGIN 1번
  if (currentListIndex >= channelCount) {
    print_both("END\n");
    currentListIndex = 0;
    print_both("BEGIN\n");
  }

  start_scan(targetChannels[currentListIndex]);
}
