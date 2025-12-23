#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

int main() {
    int fd;
    struct termios options;
    char buf[1024];

    // 1. UART0 포트 열기 (라즈베리 파이 4 기본 UART)
    fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY);
    if (fd == -1) {
        perror("포트를 열 수 없습니다. /dev/ttyAMA0 확인 및 sudo 권한 필요");
        return -1;
    }

    // 2. 통신 환경 설정 (115200 Baudrate)
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    
    options.c_cflag |= (CLOCAL | CREAD); // 수신 가능하도록 설정
    options.c_cflag &= ~PARENB;          // 패리티 비트 없음
    options.c_cflag &= ~CSTOPB;          // 스톱 비트 1개
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;               // 데이터 비트 8개
    
    // 중요: Raw 모드 설정 (입력 데이터 가공 방지 및 에코 방지)
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // 소프트웨어 흐름 제어 해제
    options.c_oflag &= ~OPOST; // 출력 가공 해제
    
    tcsetattr(fd, TCSANOW, &options);

    printf("--- [시작] ESP32 데이터 수신 대기 중 (Ctrl+C로 종료) ---\n");

    // 3. 무한 루프 수신
    while (1) {
        int n = read(fd, buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = '\0'; // 문자열 끝 처리
            printf("%s", buf); // 화면 출력
            fflush(stdout);    // 버퍼 즉시 비우기
        }
        usleep(100); // CPU 부하 방지용 미세 대기
    }

    close(fd);
    return 0;
}
