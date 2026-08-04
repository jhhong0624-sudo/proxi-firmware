/*
 * ============================================================
 *  ESP32-S3  Voltage/Current Sensor Logger  — v21.6
 * ============================================================
 *
 *  [v20.4 → v21.0 변경 내역]
 *  - 핀맵 변경: DETECT_EN(IO35), PRT_EN(IO36), CNT_TMZ(IO37), SW(IO3), TFT_RST(IO7)
 *  - TURBO 기능 제거 (단일 출력부로 통합)
 *  - DETECT/PREVENT 단일 출력부 통합: 동시 동작 불가, 전환 시 3초 딜레이
 *  - 로깅은 DETECT 모드에서만
 *  - RTC(RV-8263-C8, I2C 0x51) 추가: 컴파일타임 KST 기준 초기화
 *    WiFi/NTP 없어도 항상 정확한 타임스탬프 보장
 *  - MCP4651(I2C 0x28) 디지털 포텐셔미터 추가: VDC/Vpp 원격 조절
 *    VPP = VDC × 2 자동 계산, MQTT VPSET 명령으로 제어
 *  - SD NAND(CSNP1GCR01-BOW): 핀 동일, SD.h 그대로 사용
 *  - 더블리셋 MSC: 타임아웃 10s→5s, 1번 리셋 시 빨강 LED 피드백
 *  - 대시보드: index_v2.html (기존 index.html 유지)
 *
 *  [Arduino IDE — Tools 설정]
 *  Board              : ESP32S3 Dev Module
 *  USB CDC On Boot    : Enabled
 *  CPU Frequency      : 240MHz (WiFi)
 *  Core Debug Level   : None
 *  USB DFU On Boot    : Disabled
 *  Erase All Flash    : Disabled (첫 플래시 시에만 Enabled)
 *  Events Run On      : Core 1
 *  Flash Mode         : QIO 80MHz
 *  Flash Size         : 4MB (32Mb)
 *  JTAG Adapter       : Disabled
 *  Arduino Runs On    : Core 1
 *  USB Firmware MSC   : Disabled
 *  Partition Scheme   : Default 4MB with spiffs (1.2MB APP/1.5MB SPIFFS)
 *  PSRAM              : Disabled
 *  Upload Mode        : UART0 / Hardware CDC
 *  Upload Speed       : 921600
 *  USB Mode           : Hardware CDC and JTAG
 *
 *  [필요 라이브러리]
 *  - PubSubClient      (MQTT)
 *  - 내장: WiFi, WebServer, SD, SPIFFS, Wire, SPI,
 *          HTTPClient, ESPmDNS, Update, Preferences, USB, USBMSC
 *  ※ car_v2: Adafruit GFX/ST7789(TFT) 제거됨 (디스플레이 미사용)
 *
 *  [핀 배치 — car_v2]
 *  SD NAND   : CS=10, MOSI=11, SCK=12, MISO=13
 *  A/C 신호  : IO17 (HIGH=A/C ON, 딥슬립 ext0 wakeup) ★구 TFT_SCL
 *  LED       : R=4, G=5, B=6
 *  (해방된 핀: IO7/14/15/16/18 — 구 TFT, 여유)
 *  SW      : IO3 (더블리셋용)
 *  DETECT  : IO35(DETECT_EN)
 *  PREVENT : IO36(PRT_EN), IO37(CNT_TMZ)
 *  I2C     : SDA=8, SCL=9
 *            ADS1115(0x48), MCP4725-A(0x60), MCP4725-B(0x61)
 *            RV-8263 RTC(0x51), MCP4651 포텐셔미터(0x28)
 * ============================================================
 */

#include <Arduino.h>
#include <nvs_flash.h>
#include <esp_mac.h>
#include "esp_task_wdt.h"   // 워치독 제어 (MSC 모드 무한 재부팅 방지)
#include "soc/rtc_cntl_reg.h"  // 유선 업데이트: ROM 다운로드 모드 강제 진입
#include "esp_sleep.h"        // 완전 꺼짐: 딥슬립
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <time.h>
#include <Preferences.h>
#include <Update.h>
#include <vector>
#include <algorithm>
#include "USB.h"
#include "USBMSC.h"
#include "driver/rtc_io.h"   // 딥슬립 ext0 wakeup용 RTC GPIO 풀다운

// ════════════════════════════════════════════════════════════
//  핀 정의
// ════════════════════════════════════════════════════════════
constexpr int SD_CS_PIN    = 10;
constexpr int SD_MOSI_PIN  = 11;
constexpr int SD_SCK_PIN   = 12;
constexpr int SD_MISO_PIN  = 13;
// ★ car_v2: ST7789(TFT) 제거 — IO7/14/15/16/17/18 해방
constexpr int AC_SIGNAL_PIN = 17;  // ★ 자동차 A/C 신호 입력 (구 TFT_SCL, RTC GPIO)
                                    //   A/C ON=HIGH / OFF=LOW, 딥슬립 ext0 wakeup 핀
constexpr int PIN_DETECT_EN = 35;  // DETECT 출력 활성화 (Q3, Q4 게이트)
constexpr int PIN_PRT_EN    = 36;  // PREVENT 출력 활성화 (Q1, Q2 게이트)
constexpr int PIN_CNT_TMZ   = 37;  // PREVENT TMZ 출력
constexpr int PIN_SW        = 3;   // 스위치 (더블리셋)
constexpr int LED_R_PIN     = 4;
constexpr int LED_G_PIN     = 5;
constexpr int LED_B_PIN     = 6;

// ════════════════════════════════════════════════════════════
//  SPI  (TFT 제거됨 — car_v2)
// ════════════════════════════════════════════════════════════
SPIClass sdSpi(FSPI);

// ════════════════════════════════════════════════════════════
//  I2C 주소
// ════════════════════════════════════════════════════════════
constexpr uint8_t ADS1115_ADDR  = 0x48;
constexpr uint8_t DAC_A_ADDR    = 0x60;
constexpr uint8_t DAC_B_ADDR    = 0x61;
constexpr uint8_t RV8263_ADDR   = 0x51;  // RTC
constexpr uint8_t MCP4651_ADDR  = 0x28;  // 디지털 포텐셔미터

// ════════════════════════════════════════════════════════════
//  MQTT 설정
// ════════════════════════════════════════════════════════════
const char* MQTT_HOST = "0787e7c51f3849acbc9b6fb026816a97.s1.eu.hivemq.cloud";
const char* MQTT_USER = "PROXIHEALTH";
const char* MQTT_PASS = "!Jack0190";
constexpr uint16_t MQTT_PORT       = 8883;
constexpr uint32_t MQTT_PUBLISH_MS = 1000;
constexpr uint32_t MQTT_RECONN_MS  = 10000;

WiFiClientSecure mqttWifiClient;
PubSubClient     mqttClient(mqttWifiClient);

char g_deviceId[16]    = "";
char g_deviceName[32]  = "";
char g_topicData[64]   = "";
char g_topicStatus[64] = "";
char g_topicCmd[64]    = "";
char g_topicName[64]   = "";

uint32_t lastMqttPublishMs = 0;
uint32_t lastMqttReconnMs  = 0;

// ════════════════════════════════════════════════════════════
//  동작 모드 — DETECT / PREVENT 단일 출력, 동시 동작 불가
// ════════════════════════════════════════════════════════════
enum Mode { MODE_IDLE, MODE_DETECT, MODE_PREVENT, MODE_TRANSITION };
Mode     g_mode          = MODE_IDLE;
Mode     g_nextMode      = MODE_IDLE;
uint32_t g_transitionMs  = 0;
constexpr uint32_t TRANSITION_DELAY_MS = 2000;   // ★ car_v4: 3000→2000 (동작 전환 딜레이 2초)

bool g_detectActive  = false;
bool g_preventActive = false;

// ════════════════════════════════════════════════════════════
//  독립 타이머
// ════════════════════════════════════════════════════════════
uint32_t g_detectIntervalHr   = 1;
uint32_t g_detectDurationMin  = 1;
uint32_t g_preventIntervalHr  = 1;
uint32_t g_preventDurationMin = 10;

uint32_t g_detectCycleMs  = 0;
uint32_t g_detectStartMs  = 0;
uint32_t g_preventCycleMs = 0;
uint32_t g_preventStartMs = 0;

// ── NAS 자동 업로드 설정 ──────────────────────────────────────
bool     g_autoUpload     = true;   // (일반 라인용, car는 미사용)
uint32_t g_autoUploadHour = 0;

// ── A/C 연동 설정 (car_v4) ────────────────────────────────────
//  동작코드: 0=정지(IDLE), 1=감지(DETECT), 2=방지(PREVENT)
//  [A/C ON 시퀀스]  g_acOnAction 을 g_acOnMin분 동작 → g_acOnNext 로 전환(유지)
//  [A/C OFF 시퀀스] (ON→OFF 전환) g_acOffAction 을 g_acOffMin분 동작 → 완전꺼짐(딥슬립)
uint8_t  g_acOnAction    = 2;       // A/C ON 시 동작 (기본 방지)
uint32_t g_acOnMin       = 10;      // A/C ON 동작 지속 시간(분)
uint8_t  g_acOnNext      = 1;       // ON 동작 종료 후 전환할 동작 (기본 감지)
uint8_t  g_acOffAction   = 1;       // A/C OFF 후 동작 (기본 감지)
uint32_t g_acOffMin      = 20;      // A/C OFF 동작 지속 시간(분) → 종료 시 딥슬립
// A/C 신호 상태머신 런타임 변수
bool     g_acPrevHigh    = false;   // 직전 A/C 신호(HIGH=ON)
bool     g_acOnTimerOn   = false;   // ON 동작 타이머 진행 중
uint32_t g_acOnTimerMs   = 0;       // ON 동작 타이머 시작 시각
bool     g_acOffTimerOn  = false;   // OFF 동작 타이머 진행 중
uint32_t g_acOffTimerMs  = 0;       // OFF 동작 타이머 시작 시각
// 로그 파일명 태그: A/C ON 단계 감지="POL"(pollution), A/C OFF 단계 감지="DRY", 수동/기타=""
char     g_logTag[8]     = "";

inline uint32_t detectIntervalMs()  { return g_detectIntervalHr  * 3600000UL; }
inline uint32_t detectDurationMs()  { return g_detectDurationMin * 60000UL;   }
inline uint32_t preventIntervalMs() { return g_preventIntervalHr * 3600000UL; }
inline uint32_t preventDurationMs() { return g_preventDurationMin * 60000UL;  }

// ════════════════════════════════════════════════════════════
//  수동 모드 플래그
// ════════════════════════════════════════════════════════════
bool g_detectManualOff  = false;
bool g_preventManualOff = false;
bool g_detectManualOn   = false;
bool g_preventManualOn  = false;

// ════════════════════════════════════════════════════════════
//  동작 시간표 (매일 반복, 시계 기준) — v22.6
//  각 이벤트: 시각(분 단위 0~1439) + 동작(0=IDLE,1=DETECT,2=PREVENT)
//  그 시각이 되면 해당 동작으로 전환, 다음 이벤트 시각까지 유지.
//  자정 넘김(wrap)도 처리: 현재 시각 이전의 마지막 이벤트가 활성.
// ════════════════════════════════════════════════════════════
struct SchedEvent { uint16_t minOfDay; uint8_t action; };
SchedEvent g_sched[10];
uint8_t    g_schedCount = 0;
char       g_schedStr[160] = "";   // 원본 문자열("1300=D,1301=P,1311=I") 저장/표시용
int        g_schedLastApplied = -2; // 마지막 적용 인덱스 (변경 감지용, -2=미적용)

// 동작 코드: 0=IDLE(정지), 1=DETECT(기록), 2=PREVENT(방지), 3=DETECT(출력만/무기록)
inline uint8_t schedActionFromChar(char c){
  if(c=='D'||c=='d') return 1;
  if(c=='P'||c=='p') return 2;
  if(c=='O'||c=='o') return 3;   // O = 감지 출력만(로그 미기록)
  return 0;  // I
}
inline char schedCharFromAction(uint8_t a){
  return a==1?'D':(a==2?'P':(a==3?'O':'I'));
}

// DETECT 중 CSV 로그 기록 여부 (false=기록, true=출력만)
bool g_detectNoLog = false;

// 완전 꺼짐(소프트 오프) 상태 — 출력 정지+시간표 중단, Wi-Fi 유지 (RAM 비저장)
bool g_powerOff = false;

// ════════════════════════════════════════════════════════════
//  MCP4651 포텐셔미터 설정 (PREVENT 출력 레벨)
//  부품: MCP4651T-503E/ST → "-503" = 50kΩ full scale (★ 10kΩ 아님!)
//  P1(Wiper1/R48): VDC 제어  — 기준: step50 ≈ 0.7V
//  P0(Wiper0/R51): Vpp 제어  — 기준: step29 ≈ 1.5V
//  VPP = VDC × 2 자동 계산
// ════════════════════════════════════════════════════════════
// ★ 포텐셔미터 full-scale 저항(Ω). MCP4651-503 = 50kΩ.
constexpr float POT_FULL_OHM = 50000.0f;
constexpr float POT_STEPS    = 256.0f;

// ★ v22.5: 실측 결과 와이퍼가 반대로 배선됨(A–W 방식).
//   step↑ → 실제 저항↓  →  R_pot = (256 - step)/256 × 50kΩ
//   측정 검증: set0.1V→1.86V, set0.7V→1.718V, set1.5V→1.166V (설정↑ 측정↓)
//   역산 R_pot ≈ 46.5k/39.1k/19.7k, 풀스케일 ≈50kΩ → A–W 반전 확정
constexpr bool POT_INVERTED = true;   // false = 정방향(W–B) 배선용

float    g_vdcSet    = 0.7f;   // 사용자 설정 VDC (V)
float    g_vppSet    = 1.4f;   // Vpp = VDC × 2 (V)

// 필요저항(Ω) → MCP4651 step (배선 반전 반영)
uint8_t rwToStep(float rw) {
  int s = (int)round(rw * POT_STEPS / POT_FULL_OHM);   // 정방향 step
  if (POT_INVERTED) s = 256 - s;                       // A–W 배선 → 반전
  return (uint8_t)constrain(s, 0, 255);
}

// ── VDC 스텝 변환 ─────────────────────────────────────────────
// 회로: 3.3V → 36kΩ(R47) → R_pot(R48_IN↔R48_OUT) → GND, VDC = 중간 노드
//   VDC = 3.3 × R_pot / (36000 + R_pot)
//   역산: R_pot = 36000 × VDC / (3.3 - VDC)
// 검증(반전 반영): VDC=0.7V → rw≈9.7kΩ → step=256-50=206
//                 VDC=1.5V → rw=30kΩ   → step=256-154=102
uint8_t vdcToStep(float vdc) {
  vdc = constrain(vdc, 0.01f, 1.90f);           // 회로 최대 ≈1.92V
  float rw = 36000.0f * vdc / (3.3f - vdc);     // 필요 저항(Ω)
  return rwToStep(rw);
}

// ── Vpp 스텝 변환 ─────────────────────────────────────────────
// 회로: 고정전압 Vin → op-amp 비반전 증폭 → Vpp
//   op-amp: Rin=1kΩ(고정), Rf=R_pot → gain = 1 + Rf/1000, Vpp = Vin×gain
// 역산: Rf = (Vpp/Vin - 1) × 1000
// ※ Vpp 포텐셔미터(R51)도 동일 칩이라 같은 반전(rwToStep) 적용.
//    실측 검증 필요 — 반대로 나오면 Vpp용 반전 플래그 별도 분리 예정.
constexpr float VPP_VIN = 1.5f / 6.6f;          // ≈0.2273V (op-amp 고정 입력)
uint8_t vppToStep(float vpp) {
  float rw = (vpp <= VPP_VIN) ? 0.0f : (vpp / VPP_VIN - 1.0f) * 1000.0f;  // 필요 Rf(Ω)
  return rwToStep(rw);
}

// ════════════════════════════════════════════════════════════
//  펌웨어 버전
// ════════════════════════════════════════════════════════════
#define FW_VERSION "car_v4"

// ── 더블리셋 전방 선언 (mqttOnMessage에서 사용, 실제 정의는 파일 하단) ──
constexpr uint32_t RST_MAGIC = 0xDEAD5678;
extern RTC_NOINIT_ATTR uint32_t rtc_magic;
extern RTC_NOINIT_ATTR uint32_t rtc_cnt;

// ── 유선(USB) 펌웨어 다운로드 모드 진입 ──────────────────────
//  케이스에 BOOT/RESET 버튼이 없어 소프트웨어로 ROM 다운로드 모드 강제 진입.
//  진입 후 PC에서 Arduino IDE / esptool 로 USB 케이블 통해 .bin 굽기.
//  취소/복귀: 전원 OFF → ON (또는 플래시 완료 후 자동 정상 부팅).
inline void enterDownloadMode(){
  Serial.println("[DL] forcing ROM download(boot) mode... flash via USB now.");
  Serial.flush();
  delay(100);
  // RTC OPTION1 레지스터에 강제 다운로드 부트 비트 세팅 후 리셋
  REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
  esp_restart();   // 리셋 후 ROM이 다운로드 모드로 대기 (절대 리턴 안 함)
}

// ════════════════════════════════════════════════════════════
//  전역 상태
// ════════════════════════════════════════════════════════════
bool useSD     = false;
bool useSPIFFS = false;
File     logFile;
char     currentLogPath[64] = "/log.csv";
uint32_t lastFlushMs        = 0;
bool     g_isApMode         = false;

// ════════════════════════════════════════════════════════════
//  설정 상수
// ════════════════════════════════════════════════════════════
char g_nasBase[128] = "";
char g_nasDir[128]  = "";
char g_nasUser[64]  = "";
char g_nasPass[64]  = "";
#define WEBDAV_BASE g_nasBase
#define WEBDAV_DIR  g_nasDir
#define WEBDAV_USER g_nasUser
#define WEBDAV_PASS g_nasPass
const char* UPDATE_USER = "admin";
const char* UPDATE_PASS = "admin1234";

char g_wifiSsid[64] = "";
char g_wifiPass[64] = "";

constexpr bool     SERIAL_DEBUG   = false;
constexpr uint32_t WIFI_RETRY_MS  = 15000;
constexpr uint32_t LOG_FLUSH_MS   = 1000;

constexpr uint32_t TRI_PERIOD_MS = 20000;
constexpr float    VREF_DAC      = 3.3f;
constexpr float    TRI_OFFSET_V  = 1.65f;
constexpr float    TRI_AMP_V     = 1.5f;
constexpr uint16_t DAC_MAX_CODE  = 4095;
constexpr uint16_t DAC_MID_CODE  = (uint16_t)((TRI_OFFSET_V/VREF_DAC)*DAC_MAX_CODE+0.5f);
constexpr uint16_t DAC_AMP_CODE  = (uint16_t)((TRI_AMP_V  /VREF_DAC)*DAC_MAX_CODE+0.5f);
constexpr uint32_t LOG_INTERVAL_MS = 200;

constexpr uint16_t ADS1115_CFG_BASE = (0b001<<9)|(1<<8)|(0b100<<5)|0b11;
constexpr uint16_t ADS1115_CFG_AIN0 = 0x8000|(0b100<<12)|ADS1115_CFG_BASE;
constexpr uint16_t ADS1115_CFG_AIN1 = 0x8000|(0b101<<12)|ADS1115_CFG_BASE;
constexpr float    ADS1115_LSB      = 4.096f/32768.0f;

constexpr float R_SHUNT        = 10.0f;
constexpr float INA_GAIN       = 200.0f;
constexpr float V_OFFSET       = 1.65f;
constexpr float VDIFF_CENTER_V = 1.65f;
constexpr float VDIFF_GAIN     = 2.50f;

float g_vOffsetCal = V_OFFSET;

constexpr int TZ_OFFSET_SEC  = 9*3600;
constexpr int DST_OFFSET_SEC = 0;
const char*   NTP1 = "pool.ntp.org";
const char*   NTP2 = "time.nist.gov";
bool timeSynced = false;

uint32_t triStartMs    = 0;
uint32_t lastLogMs     = 0;
uint32_t lastWifiTryMs = 0;

struct Latest {
  uint32_t nowMs=0;
  float triNorm=NAN,vCurrentRaw=NAN,vVoltageRaw=NAN,vDiff=NAN,currentA=NAN;
  bool detecting=false;
} latest;

// ── 감지(DETECT) 중 주기별 전류 피크 ─────────────────────────
//  삼각파 1주기(TRI_PERIOD_MS=20s)마다 그 구간의 It 최대값을 확정해 발행.
//  대시보드가 it_pk_n(시퀀스) 증가를 감지해 꺾은선 그래프에 점을 추가.
float    g_itPeakCur  = 0.0f;   // 진행 중 주기의 누적 최대 It
float    g_itPeakLast = 0.0f;   // 직전 완료 주기의 최대 It (발행값)
uint32_t g_itPeakSeq  = 0;      // 주기 완료 시퀀스 (단조 증가)
uint32_t g_prevPeriodMs = 0;    // 현재 주기 시작 시각

static char g_latestJson[1100];
static char g_dt[24];

WebServer   server(80);
Preferences prefs;

// ════════════════════════════════════════════════════════════
//  USB MSC — 전역 선언
// ════════════════════════════════════════════════════════════
static USBMSC    msc;
static uint32_t  s_mscSectors = 0;
static bool      s_mscReady   = false;

// ════════════════════════════════════════════════════════════
//  그래프 버퍼 — car_v2: TFT 제거로 미사용 (그래프 표시 없음)
// ════════════════════════════════════════════════════════════
inline void graphClear(){}   // 호환용 빈 구현

// ════════════════════════════════════════════════════════════
//  LED — Active Low
// ════════════════════════════════════════════════════════════
void setRgbLed(bool r,bool g,bool b){
  digitalWrite(LED_R_PIN,r?LOW:HIGH);
  digitalWrite(LED_G_PIN,g?LOW:HIGH);
  digitalWrite(LED_B_PIN,b?LOW:HIGH);
}
void ledOff()     { setRgbLed(0,0,0); }
void ledRed()     { setRgbLed(1,0,0); }
void ledGreen()   { setRgbLed(0,1,0); }
void ledBlue()    { setRgbLed(0,0,1); }
void ledYellow()  { setRgbLed(1,1,0); }
void ledPurple()  { setRgbLed(1,0,1); }  // 보라색 (R+B)
void ledCyan()    { setRgbLed(0,1,1); }  // 시안색 (G+B)
void ledWhite()   { setRgbLed(1,1,1); }  // 하얀색 (R+G+B)
void ledBlink(int n,bool r,bool g,bool b,uint32_t ms=500){
  for(int i=0;i<n;i++){ setRgbLed(r,g,b); delay(ms); ledOff(); delay(ms); }
}

// ── LED 상태 정의 ──────────────────────────────────────────
//  DETECT 중   : 노란색 유지
//  PREVENT 중  : 파란색 유지
//  TRANSITION  : 시안색 유지 (전환 중)
//  IDLE (정지) : 하얀색 유지
//  MSC 모드    : 빨간색 유지  (runUsbMscMode 직접 설정)
//  부팅 Wi-Fi  : 보라색 5번 점멸 → setup()에서 설정
//  부팅 AP     : 녹색   5번 점멸 → setup()에서 설정
void updateLed(){
  if(g_powerOff){ ledOff(); return; }            // 완전 꺼짐 → LED OFF
  switch(g_mode){
    case MODE_DETECT:     ledYellow();   break;  // 노란색
    case MODE_PREVENT:    ledBlue();     break;  // 파란색
    case MODE_TRANSITION: ledCyan();     break;  // 시안색
    default:              ledWhite();    break;  // 하얀색 (정지)
  }
}

// ════════════════════════════════════════════════════════════
//  출력 핀 제어
// ════════════════════════════════════════════════════════════
void applyOutputs(){
  digitalWrite(PIN_DETECT_EN, g_mode==MODE_DETECT  ? HIGH : LOW);
  digitalWrite(PIN_PRT_EN,    g_mode==MODE_PREVENT ? HIGH : LOW);
  digitalWrite(PIN_CNT_TMZ,   g_mode==MODE_PREVENT ? HIGH : LOW);
}

// ════════════════════════════════════════════════════════════
//  RTC — RV-8263-C8 (I2C 0x51) 드라이버
// ════════════════════════════════════════════════════════════
static uint8_t bcd2dec(uint8_t b){ return (b>>4)*10+(b&0x0F); }
static uint8_t dec2bcd(uint8_t d){ return ((d/10)<<4)|(d%10); }

bool rtcRead(struct tm& t){
  Wire.beginTransmission(RV8263_ADDR);
  Wire.write(0x04);  // Seconds 레지스터부터
  if(Wire.endTransmission(false)!=0) return false;
  Wire.requestFrom(RV8263_ADDR,(uint8_t)7);
  if(Wire.available()<7) return false;
  uint8_t sec  = Wire.read();
  uint8_t mn   = Wire.read() & 0x7F;
  uint8_t hr   = Wire.read() & 0x3F;
  uint8_t day  = Wire.read() & 0x3F;
  Wire.read();  // weekday 무시
  uint8_t mo   = Wire.read() & 0x1F;
  uint8_t yr   = Wire.read();
  if(sec & 0x80){ /* VL 비트: 전압 저하로 시간 손실 가능 */ }
  t.tm_sec  = bcd2dec(sec & 0x7F);
  t.tm_min  = bcd2dec(mn);
  t.tm_hour = bcd2dec(hr);
  t.tm_mday = bcd2dec(day);
  t.tm_mon  = bcd2dec(mo) - 1;
  t.tm_year = bcd2dec(yr) + 100;  // 2000+yr, tm_year는 1900 기준
  t.tm_isdst = 0;
  return true;
}

bool rtcWrite(const struct tm& t){
  Wire.beginTransmission(RV8263_ADDR);
  Wire.write(0x04);
  Wire.write(dec2bcd(t.tm_sec));
  Wire.write(dec2bcd(t.tm_min));
  Wire.write(dec2bcd(t.tm_hour));
  Wire.write(dec2bcd(t.tm_mday));
  Wire.write(0);  // weekday
  Wire.write(dec2bcd(t.tm_mon+1));
  Wire.write(dec2bcd(t.tm_year-100));
  return Wire.endTransmission()==0;
}

// 컴파일 타임 파싱 (__DATE__="Jun 11 2026", __TIME__="14:30:00")
bool parseCompileTime(struct tm& t){
  static const char* months = "JanFebMarAprMayJunJulAugSepOctNovDec";
  char mon[4]={__DATE__[0],__DATE__[1],__DATE__[2],0};
  const char* p = strstr(months, mon);
  if(!p) return false;
  t.tm_mon  = (int)(p-months)/3;
  t.tm_mday = atoi(__DATE__+4);
  t.tm_year = atoi(__DATE__+7)-1900;
  t.tm_hour = atoi(__TIME__);
  t.tm_min  = atoi(__TIME__+3);
  t.tm_sec  = atoi(__TIME__+6);
  t.tm_isdst= 0;
  return true;
}

// RTC 초기화: 컴파일시간 < RTC시간이면 RTC 유지, 아니면 컴파일시간으로 설정
void initRTC(){
  struct tm compTime, rtcTime;
  if(!parseCompileTime(compTime)){
    Serial.println("[RTC] compile time parse fail");
    return;
  }
  // 컴파일 타임은 KST → UTC 변환 후 time_t 계산
  time_t compEpoch = mktime(&compTime) - TZ_OFFSET_SEC;

  bool rtcOk = rtcRead(rtcTime);
  time_t rtcEpoch = rtcOk ? mktime(&rtcTime) - TZ_OFFSET_SEC : 0;

  if(!rtcOk || rtcEpoch < compEpoch){
    // RTC 무효 또는 펌웨어보다 오래됨 → 컴파일 시간으로 설정
    rtcWrite(compTime);
    struct timeval tv = { compEpoch, 0 };
    settimeofday(&tv, nullptr);
    Serial.printf("[RTC] Set to compile time: %04d-%02d-%02d %02d:%02d:%02d KST\n",
      compTime.tm_year+1900, compTime.tm_mon+1, compTime.tm_mday,
      compTime.tm_hour, compTime.tm_min, compTime.tm_sec);
  } else {
    // RTC 시간 유효 → 시스템 시간 동기화
    struct timeval tv = { rtcEpoch, 0 };
    settimeofday(&tv, nullptr);
    Serial.printf("[RTC] Restored: %04d-%02d-%02d %02d:%02d:%02d KST\n",
      rtcTime.tm_year+1900, rtcTime.tm_mon+1, rtcTime.tm_mday,
      rtcTime.tm_hour, rtcTime.tm_min, rtcTime.tm_sec);
  }
  // TZ 설정 (NTP/localtime_r 모두 KST 적용)
  configTime(TZ_OFFSET_SEC, DST_OFFSET_SEC, NTP1, NTP2);
  timeSynced = true;
}

// NTP 동기화 후 RTC 업데이트
void syncRTCfromSystem(){
  time_t now;
  time(&now);
  if(now < 1577836800) return;  // 2020년 이전이면 무효
  // 시스템 시간(UTC)에 TZ_OFFSET 더해 KST로 변환 → RTC에 저장
  time_t kst = now + TZ_OFFSET_SEC;
  struct tm t;
  gmtime_r(&kst, &t);
  if(rtcWrite(t))
    Serial.println("[RTC] Synced from NTP");
}

// ════════════════════════════════════════════════════════════
//  MCP4651 — 듀얼 디지털 포텐셔미터 (I2C 0x28)
//  Wiper0 (reg=0): P0/R51 → Vpp 제어
//  Wiper1 (reg=1): P1/R48 → VDC 제어
// ════════════════════════════════════════════════════════════
void writeMCP4651(uint8_t reg, uint16_t val){
  val = min(val, (uint16_t)256);
  Wire.beginTransmission(MCP4651_ADDR);
  Wire.write(((reg & 0x0F)<<4) | 0x00 | (uint8_t)((val>>8)&0x01));
  Wire.write((uint8_t)(val & 0xFF));
  Wire.endTransmission();
}

void applyPotentiometer(){
  uint8_t stepVDC = vdcToStep(g_vdcSet);
  uint8_t stepVPP = vppToStep(g_vppSet);
  writeMCP4651(1, stepVDC);  // Wiper1 = VDC
  writeMCP4651(0, stepVPP);  // Wiper0 = Vpp
  Serial.printf("[POT] VDC=%.2fV(step%d) Vpp=%.2fV(step%d)\n",
    g_vdcSet, stepVDC, g_vppSet, stepVPP);
}

// ════════════════════════════════════════════════════════════
//  설정 저장/로드
// ════════════════════════════════════════════════════════════
void loadSettings(){
  prefs.begin("cfg",true);
  g_detectIntervalHr   = prefs.getUInt("d_intv", 1);
  g_detectDurationMin  = prefs.getUInt("d_dur",  1);
  g_preventIntervalHr  = prefs.getUInt("p_intv", 1);
  g_preventDurationMin = prefs.getUInt("p_dur",  10);
  g_vdcSet = prefs.getFloat("vdc_set", 0.7f);
  g_autoUpload     = prefs.getBool("au_on", true);
  g_autoUploadHour = prefs.getUInt("au_hr", 0);
  g_acOnAction     = (uint8_t)prefs.getUInt("ac_onA", 2);
  g_acOnMin        = prefs.getUInt("ac_onM", 10);
  g_acOnNext       = (uint8_t)prefs.getUInt("ac_onN", 1);
  g_acOffAction    = (uint8_t)prefs.getUInt("ac_ofA", 1);
  g_acOffMin       = prefs.getUInt("ac_ofM", 20);
  String sch = prefs.getString("sched", "");
  prefs.end();
  g_vppSet = g_vdcSet * 2.0f;
  if(sch.length() > 0) parseSchedule(sch.c_str());   // 저장된 시간표 복원
  Serial.printf("[CFG] DETECT=%luhr/%lumin PREVENT=%luhr/%lumin VDC=%.2fV\n",
    (unsigned long)g_detectIntervalHr,(unsigned long)g_detectDurationMin,
    (unsigned long)g_preventIntervalHr,(unsigned long)g_preventDurationMin,
    g_vdcSet);
}

void saveSettings(){
  prefs.begin("cfg",false);
  prefs.putUInt("d_intv", g_detectIntervalHr);
  prefs.putUInt("d_dur",  g_detectDurationMin);
  prefs.putUInt("p_intv", g_preventIntervalHr);
  prefs.putUInt("p_dur",  g_preventDurationMin);
  prefs.putFloat("vdc_set", g_vdcSet);
  prefs.putBool("au_on", g_autoUpload);
  prefs.putUInt("au_hr", g_autoUploadHour);
  prefs.putUInt("ac_onA", g_acOnAction);
  prefs.putUInt("ac_onM", g_acOnMin);
  prefs.putUInt("ac_onN", g_acOnNext);
  prefs.putUInt("ac_ofA", g_acOffAction);
  prefs.putUInt("ac_ofM", g_acOffMin);
  prefs.end();
}

void loadNasSettings(){
  prefs.begin("nas",true);
  String b=prefs.getString("base",""); if(b.length()>0) strlcpy(g_nasBase,b.c_str(),sizeof(g_nasBase));
  String d=prefs.getString("dir","");  if(d.length()>0) strlcpy(g_nasDir, d.c_str(),sizeof(g_nasDir));
  String u=prefs.getString("user",""); if(u.length()>0) strlcpy(g_nasUser,u.c_str(),sizeof(g_nasUser));
  String p=prefs.getString("pass",""); if(p.length()>0) strlcpy(g_nasPass,p.c_str(),sizeof(g_nasPass));
  prefs.end();
}

void saveNasSettings(){
  prefs.begin("nas",false);
  prefs.putString("base",g_nasBase);
  prefs.putString("dir", g_nasDir);
  prefs.putString("user",g_nasUser);
  prefs.putString("pass",g_nasPass);
  prefs.end();
}

// ════════════════════════════════════════════════════════════
//  기기 ID / 이름
// ════════════════════════════════════════════════════════════
void initDeviceId(){
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  snprintf(g_deviceId,sizeof(g_deviceId),"%02X%02X",mac[4],mac[5]);
  prefs.begin("device",true);
  String n=prefs.getString("name","");
  prefs.end();
  strlcpy(g_deviceName, n.length()>0 ? n.c_str() : g_deviceId, sizeof(g_deviceName));
  snprintf(g_topicData,  sizeof(g_topicData),  "proxi/%s/data",   g_deviceId);
  snprintf(g_topicStatus,sizeof(g_topicStatus),"proxi/%s/status", g_deviceId);
  snprintf(g_topicCmd,   sizeof(g_topicCmd),   "proxi/%s/cmd",    g_deviceId);
  snprintf(g_topicName,  sizeof(g_topicName),  "proxi/%s/name",   g_deviceId);
  Serial.printf("[DEVICE] ID=%s Name=%s\n",g_deviceId,g_deviceName);
}

void saveDeviceName(const char* name){
  prefs.begin("device",false);
  prefs.putString("name",name);
  prefs.end();
  strlcpy(g_deviceName,name,sizeof(g_deviceName));
}

// ════════════════════════════════════════════════════════════
//  Wi-Fi
// ════════════════════════════════════════════════════════════
void loadWifiCredentials(){
  prefs.begin("wificfg",true);
  String s=prefs.getString("ssid",""); String p=prefs.getString("pass","");
  prefs.end();
  strlcpy(g_wifiSsid,s.c_str(),sizeof(g_wifiSsid));
  strlcpy(g_wifiPass,p.c_str(),sizeof(g_wifiPass));
}
void saveWifiCredentials(const char* ssid,const char* pass){
  prefs.begin("wificfg",false);
  prefs.putString("ssid",ssid); prefs.putString("pass",pass);
  prefs.end();
  strlcpy(g_wifiSsid,ssid,sizeof(g_wifiSsid));
  strlcpy(g_wifiPass,pass,sizeof(g_wifiPass));
}

bool isTimeValid(){ time_t n;time(&n);return n>1577836800; }

void connectWiFiOnce(){
  WiFi.mode(WIFI_STA); WiFi.setSleep(false); WiFi.persistent(false); WiFi.setAutoReconnect(true);
  if(!strlen(g_wifiSsid)){ Serial.println("[WiFi] no SSID"); return; }
  WiFi.begin(g_wifiSsid,g_wifiPass);
  Serial.printf("[WiFi] connecting to %s",g_wifiSsid);
  uint32_t t=millis();
  while(WiFi.status()!=WL_CONNECTED&&millis()-t<15000){delay(250);Serial.print(".");}
  Serial.println();
  if(WiFi.status()==WL_CONNECTED){
    Serial.printf("[WiFi] IP=%s\n",WiFi.localIP().toString().c_str());
    // NTP 동기화 후 RTC 업데이트
    configTime(TZ_OFFSET_SEC,DST_OFFSET_SEC,NTP1,NTP2);
    for(int i=0;i<20;i++){if(isTimeValid()){timeSynced=true;syncRTCfromSystem();break;}delay(250);}
  } else Serial.println("[WiFi] failed");
}

void wifiKeepAlive(){
  if(!strlen(g_wifiSsid))return;
  if(WiFi.status()==WL_CONNECTED) return;
  if(millis()-lastWifiTryMs<WIFI_RETRY_MS)return;
  lastWifiTryMs=millis(); WiFi.begin(g_wifiSsid,g_wifiPass);
}

// ════════════════════════════════════════════════════════════
//  Storage (SD NAND / SPIFFS)
// ════════════════════════════════════════════════════════════
FS* activeFS(){ if(useSD)return &SD; if(useSPIFFS)return &SPIFFS; return nullptr; }
void setupStorage(){
  sdSpi.begin(SD_SCK_PIN,SD_MISO_PIN,SD_MOSI_PIN,SD_CS_PIN);
  if(SD.begin(SD_CS_PIN,sdSpi)){
    useSD=true;
    Serial.printf("[SD] OK %llu MB\n",SD.cardSize()/(1024ULL*1024ULL));
  } else Serial.println("[SD] FAIL");
  if(!useSD&&SPIFFS.begin(true)){ useSPIFFS=true; Serial.println("[SPIFFS] OK"); }
}

void formatNow(char* out,size_t sz,bool fn){
  time_t n;time(&n);struct tm t;localtime_r(&n,&t);
  if(fn) snprintf(out,sz,"%04d%02d%02d_%02d%02d%02d",t.tm_year+1900,t.tm_mon+1,t.tm_mday,t.tm_hour,t.tm_min,t.tm_sec);
  else   snprintf(out,sz,"%04d-%02d-%02d %02d:%02d:%02d",t.tm_year+1900,t.tm_mon+1,t.tm_mday,t.tm_hour,t.tm_min,t.tm_sec);
}

uint32_t nextOfflineIdx(){
  prefs.begin("logger",false); uint32_t i=prefs.getUInt("idx",0)+1; prefs.putUInt("idx",i); prefs.end(); return i;
}

void makeNewLogPath(){
  // RTC 항상 유효하므로 datetime 기반 파일명 사용
  // g_logTag: A/C ON 단계 감지="POL", A/C OFF 단계 감지="DRY" → 파일명에 포함
  char ts[20]; formatNow(ts,sizeof(ts),true);
  if(g_logTag[0])
    snprintf(currentLogPath,sizeof(currentLogPath),"/log_%s_%s.csv",g_logTag,ts);
  else
    snprintf(currentLogPath,sizeof(currentLogPath),"/log_%s.csv",ts);
}

void openNewLogFile(){
  FS* fs=activeFS();if(!fs)return;
  makeNewLogPath();
  logFile=fs->open(currentLogPath,FILE_WRITE);
  if(!logFile){Serial.printf("[LOG] open fail: %s\n",currentLogPath);return;}
  logFile.println(F("datetime,V_ADC0_Raw[V],V_ADC1_Raw[V],Vt[V],It[A]"));
  logFile.flush(); lastFlushMs=millis();
  Serial.printf("[LOG] %s\n",currentLogPath);
}

void appendLogLine(uint32_t ms,float tri,float vc,float vv,float vt,float it){
  if(!logFile)return;
  (void)ms; (void)tri;   // time_ms, tri_norm 컬럼 제거됨(v22.9)
  formatNow(g_dt,sizeof(g_dt),false);
  // 컬럼: datetime, V_ADC0_Raw, V_ADC1_Raw, Vt, It
  logFile.printf("%s,%.4f,%.4f,%.3f,%.6f\n", g_dt,vc,vv,vt,it);
  if(millis()-lastFlushMs>=LOG_FLUSH_MS){lastFlushMs=millis();logFile.flush();}
}

// ════════════════════════════════════════════════════════════
//  DAC / 삼각파
// ════════════════════════════════════════════════════════════
void writeMCP4725(uint8_t addr,uint16_t code){
  code=min(code,(uint16_t)4095);
  Wire.beginTransmission(addr);
  Wire.write(0x40);Wire.write(code>>4);Wire.write((code&0x0F)<<4);
  Wire.endTransmission();
}
void setDACtoZero(){ writeMCP4725(DAC_A_ADDR,0); writeMCP4725(DAC_B_ADDR,0); }
float triNorm(float p){ float v=4.0f*p; if(v>2.0f)v=4.0f-v; return v-1.0f; }
void updateDAC(){
  float ph=(float)((millis()-triStartMs)%TRI_PERIOD_MS)/(float)TRI_PERIOD_MS;
  float t=triNorm(ph);
  writeMCP4725(DAC_A_ADDR,(uint16_t)constrain((int32_t)DAC_MID_CODE+(int32_t)(DAC_AMP_CODE*t),0,4095));
  writeMCP4725(DAC_B_ADDR,(uint16_t)constrain((int32_t)DAC_MID_CODE-(int32_t)(DAC_AMP_CODE*t),0,4095));
}

// ════════════════════════════════════════════════════════════
//  ADS1115
// ════════════════════════════════════════════════════════════
void ads_write(uint8_t reg,uint16_t val){
  Wire.beginTransmission(ADS1115_ADDR);Wire.write(reg);Wire.write(val>>8);Wire.write(val&0xFF);Wire.endTransmission();
}
float ads_read(uint8_t ch){
  ads_write(0x01,ch==0?ADS1115_CFG_AIN0:ADS1115_CFG_AIN1); delay(10);
  Wire.beginTransmission(ADS1115_ADDR);Wire.write(0x00);Wire.endTransmission();
  Wire.requestFrom(ADS1115_ADDR,(uint8_t)2);
  if(Wire.available()<2)return NAN;
  return (float)(int16_t)((Wire.read()<<8)|Wire.read())*ADS1115_LSB;
}

void calibrateZeroOffset(){
  delay(50);
  float sum=0; int count=0;
  for(int i=0;i<16;i++){ float v=ads_read(0); if(!isnan(v)){sum+=v;count++;} delay(5); }
  if(count>0) g_vOffsetCal=sum/count;
  Serial.printf("[CAL] V_OFFSET=%.4fV (%d samples)\n",g_vOffsetCal,count);
}

// ════════════════════════════════════════════════════════════
//  모드 전환 — 단일 출력부 (DETECT/PREVENT 동시 불가)
// ════════════════════════════════════════════════════════════
void _doStopDetect(){
  setDACtoZero();
  if(logFile){logFile.flush();logFile.close();}
  // WebDAV 업로드 (forward declaration, defined later)
  extern void startWebDAV(const char*);
  startWebDAV(currentLogPath);
  g_detectActive=false;
  Serial.println("[MODE] DETECT 정지");
}

void _doStopPrevent(){
  g_preventActive=false;
  Serial.println("[MODE] PREVENT 정지");
}

void _doStartDetect(){
  g_detectActive=true;
  g_detectStartMs=millis();
  triStartMs=millis();
  graphClear();
  // 주기별 전류 피크 측정 초기화 (시퀀스는 단조 증가 유지)
  g_itPeakCur    = 0.0f;
  g_prevPeriodMs = millis();
  if(!g_detectNoLog) openNewLogFile();   // 출력만 모드면 로그 파일 생성 안 함
  Serial.printf("[MODE] DETECT 시작 (%s)\n", g_detectNoLog?"출력만/무기록":"기록");
}

void _doStartPrevent(){
  g_preventActive=true;
  g_preventStartMs=millis();
  applyPotentiometer();  // PREVENT 시작 시 포텐셔미터 값 적용
  Serial.println("[MODE] PREVENT 시작");
}

// 모드 전환 요청 (동일 모드면 무시, 다른 모드면 3초 전환)
void requestMode(Mode target){
  if(g_mode == target) return;

  if(g_mode == MODE_TRANSITION){
    g_nextMode = target;  // 전환 목적지만 변경
    return;
  }

  // 현재 모드 정지
  if(g_mode == MODE_DETECT)  _doStopDetect();
  if(g_mode == MODE_PREVENT) _doStopPrevent();

  if(target == MODE_IDLE){
    g_mode = MODE_IDLE;
    applyOutputs();
    updateLed();
    return;
  }

  // 3초 전환 시작
  g_mode = MODE_TRANSITION;
  g_nextMode = target;
  g_transitionMs = millis();
  applyOutputs();  // 모든 출력 LOW
  updateLed();
  Serial.printf("[MODE] 전환 시작 → %s (3초 대기)\n",
    target==MODE_DETECT?"DETECT":"PREVENT");
}

// loop()에서 호출 — 전환 완료 처리
void handleTransition(){
  if(g_mode != MODE_TRANSITION) return;
  if(millis()-g_transitionMs < TRANSITION_DELAY_MS) return;

  g_mode = g_nextMode;
  applyOutputs();
  if(g_mode == MODE_DETECT){
    _doStartDetect();
    g_detectStartMs = millis();
  } else if(g_mode == MODE_PREVENT){
    _doStartPrevent();
    g_preventStartMs = millis();
  }
  updateLed();
  Serial.printf("[MODE] 전환 완료 → %s\n",
    g_mode==MODE_DETECT?"DETECT":"PREVENT");
}

// 타이머 기반 모드 시작 헬퍼
void startDetect(){
  if(g_mode == MODE_DETECT) return;
  if(g_mode == MODE_IDLE){
    g_mode = MODE_DETECT;
    applyOutputs();
    _doStartDetect();
    updateLed();
  } else {
    requestMode(MODE_DETECT);
  }
}

void stopDetect(){
  if(g_mode != MODE_DETECT) return;
  _doStopDetect();
  g_mode = MODE_IDLE;
  applyOutputs();
  updateLed();
}

void startPrevent(){
  if(g_mode == MODE_PREVENT) return;
  if(g_mode == MODE_IDLE){
    g_mode = MODE_PREVENT;
    applyOutputs();
    _doStartPrevent();
    updateLed();
  } else {
    requestMode(MODE_PREVENT);
  }
}

void stopPrevent(){
  if(g_mode != MODE_PREVENT) return;
  _doStopPrevent();
  g_mode = MODE_IDLE;
  applyOutputs();
  updateLed();
}

// ════════════════════════════════════════════════════════════
//  동작 시간표 — 파싱 / 적용 / 정보
// ════════════════════════════════════════════════════════════
// 문자열("1300=D,1301=P,1311=I") → g_sched[] (저장은 안 함)
void parseSchedule(const char* s){
  g_schedCount = 0;
  strlcpy(g_schedStr, s, sizeof(g_schedStr));
  char buf[160]; strlcpy(buf, s, sizeof(buf));
  char* tok = strtok(buf, ",");
  while(tok && g_schedCount < 10){
    char* eq = strchr(tok, '=');
    if(eq){
      *eq = '\0';
      int hhmm = atoi(tok);            // 예: "1300"→1300, "0905"→905
      int hh = hhmm / 100, mm = hhmm % 100;
      if(hh>=0 && hh<=23 && mm>=0 && mm<=59){
        g_sched[g_schedCount].minOfDay = (uint16_t)(hh*60 + mm);
        g_sched[g_schedCount].action   = schedActionFromChar(eq[1]);
        g_schedCount++;
      }
    }
    tok = strtok(NULL, ",");
  }
  // 시각 오름차순 정렬 (삽입정렬)
  for(int i=1;i<g_schedCount;i++){
    SchedEvent k=g_sched[i]; int j=i-1;
    while(j>=0 && g_sched[j].minOfDay > k.minOfDay){ g_sched[j+1]=g_sched[j]; j--; }
    g_sched[j+1]=k;
  }
  g_schedLastApplied = -2;   // 시간표 바뀜 → 다음 점검에서 즉시 재적용
  Serial.printf("[SCHED] %d events parsed: %s\n", g_schedCount, g_schedStr);
}

// 외부(MQTT) 설정 → 파싱 + NVS 저장
void setScheduleFromStr(const char* s){
  parseSchedule(s);
  prefs.begin("cfg",false);
  prefs.putString("sched", g_schedStr);
  prefs.end();
}

// 현재 시각에 활성이어야 할 이벤트 인덱스 (없으면 -1)
int schedActiveIndex(const struct tm& t){
  if(g_schedCount==0) return -1;
  int nowMin = t.tm_hour*60 + t.tm_min;
  int active = -1;
  for(int i=0;i<g_schedCount;i++) if(g_sched[i].minOfDay <= nowMin) active = i;
  if(active < 0) active = g_schedCount-1;   // 자정 넘김: 전날 마지막 이벤트가 유효
  return active;
}

// loop()에서 호출 — 시각 도래 시 모드 전환
void checkSchedule(){
  if(g_powerOff)      return;             // 완전 꺼짐 중엔 시간표 중단
  if(g_schedCount==0) return;
  if(!isTimeValid())  return;             // RTC 시각 필요
  time_t n; time(&n); struct tm t; localtime_r(&n,&t);
  int active = schedActiveIndex(t);
  if(active < 0) return;
  if(active != g_schedLastApplied){
    g_schedLastApplied = active;
    uint8_t a = g_sched[active].action;
    Serial.printf("[SCHED] apply #%d %02d:%02d → %s\n", active,
      g_sched[active].minOfDay/60, g_sched[active].minOfDay%60,
      a==1?"DETECT(log)":(a==3?"DETECT(no-log)":(a==2?"PREVENT":"IDLE")));
    // DETECT 계열은 로그 여부 플래그를 먼저 세팅 (전환 완료 시 _doStartDetect가 참조)
    g_detectNoLog = (a==3);
    Mode target = (a==1||a==3) ? MODE_DETECT : (a==2 ? MODE_PREVENT : MODE_IDLE);
    requestMode(target);
  }
}

// JSON 표시용: 현재 구간 남은시간/길이(ms)와 다음 이벤트 문자열
void getSchedSeg(int32_t* segRemMs, int32_t* segLenMs, char* nxStr, size_t nxSz){
  *segRemMs = -1; *segLenMs = -1; if(nxSz) nxStr[0]='\0';
  if(g_schedCount==0 || !isTimeValid()) return;
  time_t n; time(&n); struct tm t; localtime_r(&n,&t);
  int nowMin = t.tm_hour*60 + t.tm_min;
  int active = schedActiveIndex(t);
  if(active < 0) return;
  int next = (active+1) % g_schedCount;
  int activeMin = g_sched[active].minOfDay;
  int nextMin   = g_sched[next].minOfDay;
  int segLenMin = (((nextMin-activeMin)%1440)+1440)%1440; if(segLenMin==0) segLenMin=1440;
  int toNextMin = (((nextMin-nowMin)%1440)+1440)%1440;
  int toNextSec = toNextMin*60 - t.tm_sec; if(toNextSec<0) toNextSec=0;
  *segLenMs = segLenMin*60000;
  *segRemMs = toNextSec*1000;
  if(nxSz) snprintf(nxStr, nxSz, "%02d:%02d %c", nextMin/60, nextMin%60,
                    schedCharFromAction(g_sched[next].action));
}

// ════════════════════════════════════════════════════════════
//  완전 꺼짐 (딥슬립) — CAR 펌웨어 전용  [car_v1]
//  ★ 차량용: 배터리 방전 방지를 위해 "진짜" 완전 꺼짐(딥슬립).
//  · 모든 출력/DAC/LED/TFT/Wi-Fi 차단 후 esp_deep_sleep_start().
//  · wake 소스 미설정 → RESET 버튼 또는 전원 재인가로만 복귀.
//  · 복귀 시 RTC 플래그 클리어 → 콜드 부팅처럼 정지(IDLE)로 시작.
//  · ★복귀 방법: A/C 신호(IO17 HIGH) ext0 wakeup, 또는 RESET/전원 재인가.
//  · 주의: 회로상 하드웨어 전원 차단이 없어 3.3V 라인의 op-amp/센서 등은
//    계속 소모할 수 있음(MCU/LED만 최소화). 로드스위치 권장.
// ════════════════════════════════════════════════════════════
void enterPowerOff(){
  Serial.println("[PWR] CAR 완전 꺼짐 → 딥슬립 (A/C신호 IO17 HIGH 또는 RESET/전원으로 복귀)");
  g_powerOff = true;
  requestMode(MODE_IDLE);          // 출력 모두 정지(IDLE)
  delay(100);
  digitalWrite(PIN_DETECT_EN, LOW);
  digitalWrite(PIN_PRT_EN,    LOW);
  digitalWrite(PIN_CNT_TMZ,   LOW);
  setDACtoZero();                  // DAC 0V
  ledOff();                        // RGB LED OFF
  if(logFile){ logFile.flush(); logFile.close(); }
  rtc_magic = 0; rtc_cnt = 0;      // 깨어날 때 콜드 부팅 취급(MSC 오진입 방지)
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  // ★ A/C 신호(IO17 HIGH)로 깨어나도록 ext0 wakeup 설정
  rtc_gpio_pulldown_en((gpio_num_t)AC_SIGNAL_PIN);   // A/C OFF 시 LOW 확실히
  rtc_gpio_pullup_dis((gpio_num_t)AC_SIGNAL_PIN);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)AC_SIGNAL_PIN, 1);  // HIGH=A/C ON → wake
  delay(200);
  Serial.flush();
  esp_deep_sleep_start();          // 절대 리턴하지 않음 (A/C ON 또는 RESET로 재부팅)
}
// CAR 라인은 딥슬립이라 원격 복귀(POWERON) 불가 — 호환용 빈 구현 유지
void exitPowerOff(){
  g_powerOff = false;
  g_schedLastApplied = -2;
}

// ════════════════════════════════════════════════════════════
//  WebDAV
// ════════════════════════════════════════════════════════════
String urlEncode(const String& in){
  String out; out.reserve(in.length()*3);
  for(size_t i=0;i<in.length();i++){
    uint8_t c=(uint8_t)in[i];
    if((c>='A'&&c<='Z')||(c>='a'&&c<='z')||(c>='0'&&c<='9')||c=='-'||c=='_'||c=='.'||c=='~'||c=='/')out+=(char)c;
    else{char b[4];snprintf(b,4,"%%%02X",c);out+=b;}
  }
  return out;
}

bool webdavMkdir(const char* dirPath){
  String url=String(WEBDAV_BASE)+urlEncode(String(dirPath));
  WiFiClientSecure cli; cli.setInsecure();
  HTTPClient http;
  if(!http.begin(cli,url)) return false;
  http.setAuthorization(WEBDAV_USER,WEBDAV_PASS);
  int code=http.sendRequest("MKCOL");
  http.end();
  return (code==201||code==405);
}

String getDeviceDir(){
  String dir=String(WEBDAV_DIR);
  if(!dir.startsWith("/")) dir="/"+dir;
  if(dir.endsWith("/")) dir.remove(dir.length()-1);
  return dir+"/"+String(g_deviceId);
}

bool uploadWebDAV(const char* path){
  if(WiFi.status()!=WL_CONNECTED) return false;
  FS* fs=activeFS(); if(!fs) return false;
  File f=fs->open(path,FILE_READ); if(!f) return false;
  String devDir=getDeviceDir();
  webdavMkdir(WEBDAV_DIR);
  webdavMkdir(devDir.c_str());
  String fn=String(path);
  int sl=fn.lastIndexOf('/'); if(sl>=0) fn=fn.substring(sl+1);
  String url=String(WEBDAV_BASE)+urlEncode(devDir+"/"+fn);
  size_t fsz=(size_t)f.size();
  WiFiClientSecure cli; cli.setInsecure();
  cli.setTimeout(20000);
  HTTPClient http;
  http.setReuse(false);
  if(!http.begin(cli,url)){ f.close(); return false; }
  http.setTimeout(20000);    // ★ TLS+파일 전송 대기 (기본 5s로는 PUT 타임아웃)
  http.setConnectTimeout(15000);
  http.setAuthorization(WEBDAV_USER,WEBDAV_PASS);
  http.addHeader("Content-Type","application/octet-stream");
  int code=http.sendRequest("PUT",&f,fsz);
  if(code<200||code>=300){
    String body=http.getString();
    Serial.printf("[WebDAV] PUT FAIL %s → %d (size=%u)\n  url:%s\n  resp:%s\n",
      path,code,(unsigned)fsz,url.c_str(),body.substring(0,160).c_str());
  } else {
    Serial.printf("[WebDAV] PUT OK %s → %d (%u bytes)\n",path,code,(unsigned)fsz);
  }
  http.end(); f.close();
  return (code>=200&&code<300);
}

static volatile bool webdavBusy=false;
static char webdavPath[64]="";

void webdavTask(void* p){
  bool ok=uploadWebDAV(webdavPath);
  Serial.println(ok?"[WebDAV] OK":"[WebDAV] FAIL");
  webdavBusy=false;
  vTaskDelete(NULL);
}

void webdavUploadAllTask(void* p){
  FS* fs=activeFS();
  if(!fs){ webdavBusy=false; vTaskDelete(NULL); return; }
  webdavMkdir(WEBDAV_DIR);
  webdavMkdir(getDeviceDir().c_str());
  File root=fs->open("/");
  if(!root||!root.isDirectory()){ webdavBusy=false; vTaskDelete(NULL); return; }
  int total=0, ok=0;
  root.rewindDirectory();
  while(true){
    File f=root.openNextFile();
    if(!f) break;
    String name=String(f.name());
    f.close();
    if(!name.endsWith(".csv")) continue;
    if(("/"+name)==String(currentLogPath)||name==String(currentLogPath)) continue;
    total++;
    String fullPath=name.startsWith("/")?name:"/"+name;
    if(uploadWebDAV(fullPath.c_str())) ok++;
    delay(200); yield();
  }
  root.close();
  Serial.printf("[WebDAV] Upload all done: %d/%d\n",ok,total);
  webdavBusy=false;
  vTaskDelete(NULL);
}

void startWebDAV(const char* path){
  if(webdavBusy) return;
  strlcpy(webdavPath,path,sizeof(webdavPath));
  webdavBusy=true;
  xTaskCreatePinnedToCore(webdavTask,"webdav",8192,NULL,1,NULL,1);
}

void startWebDAVAll(){
  if(webdavBusy){ Serial.println("[WebDAV] busy"); return; }
  webdavBusy=true;
  xTaskCreatePinnedToCore(webdavUploadAllTask,"webdav_all",8192,NULL,1,NULL,1);
}

// ════════════════════════════════════════════════════════════
//  A/C 신호 상태머신 (car_v4)
//  IO17: A/C ON=HIGH / OFF=LOW
//  [A/C ON 시퀀스]
//    - A/C ON(상승엣지)        → g_acOnAction 동작 시작 + ON타이머(g_acOnMin분)
//    - ON 동작 시간 종료(A/C ON 유지) → g_acOnNext 동작으로 전환(유지)
//  [A/C OFF 시퀀스]
//    - A/C ON→OFF 전환        → g_acOffAction 동작 시작 + OFF타이머(g_acOffMin분)
//        └ 그 사이 A/C ON 재개 → ON 시퀀스 다시 시작(타이머 취소)
//        └ OFF 동작 시간 종료  → 완전 꺼짐(딥슬립)  ※데이터는 다음 부팅 시 자동 업로드
//  - 완전 꺼짐(딥슬립) 중 A/C ON(IO17 HIGH) → ext0 wakeup → 부팅 → IDLE → ON 시퀀스
//  - 전원 인가(부팅)는 항상 IDLE로 시작 (A/C 신호는 loop의 checkAcSignal이 감지)
//  - 콜드 부팅(A/C OFF)에서는 IDLE 유지(awake, 설정 가능) — 자동 sleep 안 함
// ════════════════════════════════════════════════════════════
void applyAcAction(uint8_t a){
  g_detectNoLog = false;   // A/C 감지는 기록
  requestMode(a==1 ? MODE_DETECT : (a==2 ? MODE_PREVENT : MODE_IDLE));
}

// OFF 동작 타이머 종료 → 완전 꺼짐(딥슬립). 데이터는 다음 부팅 시 자동 업로드됨.
void acTimerEndShutdown(){
  Serial.println("[AC] OFF 동작 타이머 종료 → 완전 꺼짐(딥슬립)");
  requestMode(MODE_IDLE);
  enterPowerOff();   // 딥슬립 (A/C ON으로 깨어남) — 리턴 안 함
}

// loop()에서 주기 호출 (약 300ms)
void checkAcSignal(){
  bool acHigh = (digitalRead(AC_SIGNAL_PIN) == HIGH);
  if(acHigh){
    if(!g_acPrevHigh || g_acOffTimerOn){     // 새로 ON, 또는 OFF동작 중 A/C 재개
      g_acOffTimerOn = false;
      g_acOnTimerOn  = true;
      g_acOnTimerMs  = millis();
      strlcpy(g_logTag,"POL",sizeof(g_logTag));   // A/C ON 단계 감지 → POL(pollution)
      applyAcAction(g_acOnAction);
      Serial.printf("[AC] ON → ON동작 %lu분\n",(unsigned long)g_acOnMin);
    } else if(g_acOnTimerOn){                // ON 동작 진행 중
      if(millis()-g_acOnTimerMs >= g_acOnMin*60000UL){
        g_acOnTimerOn = false;
        strlcpy(g_logTag,"POL",sizeof(g_logTag)); // 다음 동작도 A/C ON 단계 → POL
        applyAcAction(g_acOnNext);           // ON 동작 종료 → 다음 동작으로 전환
        Serial.println("[AC] ON동작 종료 → 다음 동작으로 전환");
      }
    }
  } else {
    if(g_acPrevHigh){                         // ON→OFF 전환
      g_acOnTimerOn  = false;
      g_acOffTimerOn = true;
      g_acOffTimerMs = millis();
      strlcpy(g_logTag,"DRY",sizeof(g_logTag));   // A/C OFF 단계 감지 → DRY
      applyAcAction(g_acOffAction);
      Serial.printf("[AC] OFF 전환 → OFF동작 %lu분\n",(unsigned long)g_acOffMin);
    } else if(g_acOffTimerOn){                // OFF 동작 타이머 진행
      if(millis()-g_acOffTimerMs >= g_acOffMin*60000UL){
        g_acOffTimerOn = false;
        acTimerEndShutdown();                 // 완전 꺼짐(딥슬립) — 리턴 안 함
      }
    }
    // else: A/C OFF + 타이머 없음 → IDLE 유지(awake, 설정 가능). 자동 sleep 안 함.
  }
  g_acPrevHigh = acHigh;
}

// ── 매일 지정 시각 자동 업로드 ────────────────────────────────
//  조건: 자동 업로드 ON + Wi-Fi 연결 + 시간 유효 + NAS 설정 존재.
//  설정 시각(g_autoUploadHour) 진입 시 그날 1회만 UPLOAD_ALL.
//  하루 1회만 실행되도록 tm_yday 로 가드 (재부팅 시 RAM 초기화는 무해).
void checkDailyUpload(){
  if(!g_autoUpload)               return;   // 자동 업로드 OFF면 skip
  if(WiFi.status()!=WL_CONNECTED) return;   // Wi-Fi 끊기면 skip
  if(g_nasBase[0]=='\0')          return;   // NAS 미설정이면 skip
  if(!isTimeValid())              return;   // 시간 동기 안 됐으면 skip

  time_t n; time(&n);
  struct tm t; localtime_r(&n,&t);

  static int lastUploadYday = -1;
  // 설정 시각(시)에 진입하면 그날 1회만 업로드
  if((uint32_t)t.tm_hour==g_autoUploadHour && lastUploadYday != t.tm_yday){
    lastUploadYday = t.tm_yday;
    Serial.printf("[AUTO] %02d:%02d daily NAS upload triggered\n", t.tm_hour, t.tm_min);
    startWebDAVAll();
  }
}

// ── 매시 정각 RTC 시간 재보정 ─────────────────────────────────
//  Wi-Fi 연결 상태에서 매 정각(분==0)에 1회, NTP 시각을 RTC에 다시 기록.
//  → RTC 드리프트 보정 + 백업배터리 단절 후 전원 복구 시 자동 교정.
//  (SNTP는 백그라운드로 시스템 시각을 주기 동기화하므로, 그 값을 RTC에 반영)
void checkHourlyTimeSync(){
  if(WiFi.status()!=WL_CONNECTED) return;
  if(!isTimeValid())              return;

  time_t n; time(&n);
  struct tm t; localtime_r(&n,&t);

  static int lastSyncHourKey = -1;
  int hourKey = t.tm_yday*24 + t.tm_hour;   // 정각마다 고유 키
  if(t.tm_min==0 && lastSyncHourKey != hourKey){
    lastSyncHourKey = hourKey;
    syncRTCfromSystem();   // 시스템(NTP) 시각 → RTC 기록
    Serial.printf("[RTC] hourly resync @ %02d:00\n", t.tm_hour);
  }
}

// ════════════════════════════════════════════════════════════
//  TFT 제거됨 (car_v2) — 디스플레이 관련 함수 전부 삭제
//  setupTFT / drawTransitionScreen / drawPreventScreen / drawDetectGraph 없음
// ════════════════════════════════════════════════════════════

// ════════════════════════════════════════════════════════════
//  MQTT
// ════════════════════════════════════════════════════════════
void mqttOnMessage(char* topic,byte* payload,unsigned int length){
  char msg[600]="";   // OTA URL(시놀로지 공유링크 등) 대응 위해 확대 (구 256)
  unsigned int cl=min((unsigned int)(sizeof(msg)-1),length);
  memcpy(msg,payload,cl); msg[cl]='\0';
  Serial.printf("[MQTT] cmd: %s\n",msg);

  if(strcmp(topic,g_topicCmd)!=0) return;

  if(strcmp(msg,"DETECT_START")==0){
    g_detectManualOff=false; g_detectManualOn=true;
    g_detectNoLog=false;   // 수동 감지 시작은 기본 기록
    g_logTag[0]='\0';      // 수동 감지는 태그 없음 (POL/DRY 잔재 제거)
    if(g_powerOff) exitPowerOff();   // 수동 시작 시 완전꺼짐 해제
    startDetect();
  }
  else if(strcmp(msg,"DETECT_STOP")==0){
    g_detectManualOff=true; g_detectManualOn=false;
    stopDetect();
  }
  else if(strcmp(msg,"PREVENT_START")==0){
    g_preventManualOff=false; g_preventManualOn=true;
    if(g_powerOff) exitPowerOff();   // 수동 시작 시 완전꺼짐 해제
    startPrevent();
  }
  else if(strcmp(msg,"PREVENT_STOP")==0){
    g_preventManualOff=true; g_preventManualOn=false;
    stopPrevent();
  }
  else if(strncmp(msg,"NAME:",5)==0){
    saveDeviceName(msg+5);
  }
  else if(strncmp(msg,"SCHED:",6)==0){
    // 동작 시간표 설정: SCHED:1300=D,1301=P,1311=I  (HHMM=동작)
    setScheduleFromStr(msg+6);
    // 설정 직후 현재 시각 기준으로 즉시 반영되도록 강제 재평가
    // (checkSchedule의 lastApplied는 static이라 다음 호출에서 반영됨)
  }
  else if(strncmp(msg,"VPSET:",6)==0){
    // VPSET:0.7  → VDC=0.7V, Vpp=1.4V(자동)
    float vdc=atof(msg+6);
    if(vdc>=0.1f && vdc<=2.5f){
      g_vdcSet=vdc;
      g_vppSet=vdc*2.0f;
      saveSettings();
      applyPotentiometer();
      Serial.printf("[CMD] VPSET VDC=%.2fV Vpp=%.2fV\n",g_vdcSet,g_vppSet);
    }
  }
  else if(strncmp(msg,"OTA:",4)==0){
    // 인터넷 OTA: 기기가 URL에서 .bin을 받아 플래시. (대시보드가 MQTT로 전송)
    //  · http/https 자동 분기, 리다이렉트(302) 추적
    //  · URL이 저장된 NAS 주소로 시작하면 NAS 계정으로 Basic 인증
    //  · 성공 시 esp_restart() 로 새 펌웨어 부팅
    String url = String(msg+4); url.trim();
    Serial.printf("[OTA] start: %s\n", url.c_str());
    if(WiFi.status()!=WL_CONNECTED){ Serial.println("[OTA] no Wi-Fi"); return; }
    if(logFile){ logFile.flush(); logFile.close(); }
    requestMode(MODE_IDLE);          // 출력 정지 후 업데이트
    ledPurple();                     // OTA 진행 표시(보라)

    bool https = url.startsWith("https");
    WiFiClientSecure sc; sc.setInsecure(); sc.setTimeout(20000);
    WiFiClient pc;
    HTTPClient oh;
    bool begun = https ? oh.begin(sc, url) : oh.begin(pc, url);
    if(!begun){ Serial.println("[OTA] begin fail"); updateLed(); return; }
    oh.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);  // 공유링크 리다이렉트 추적
    oh.setTimeout(20000);
    oh.setConnectTimeout(15000);
    // NAS WebDAV에 올린 .bin이면 저장된 NAS 계정으로 인증
    if(g_nasBase[0] && g_nasUser[0] && url.startsWith(g_nasBase))
      oh.setAuthorization(g_nasUser, g_nasPass);

    int hc = oh.GET();
    if(hc==HTTP_CODE_OK){
      int len = oh.getSize();
      WiFiClient* s = oh.getStreamPtr();
      if(Update.begin(len>0?len:UPDATE_SIZE_UNKNOWN, U_FLASH)){
        size_t w = Update.writeStream(*s);
        if(Update.end(true)){
          Serial.printf("[OTA] OK %u bytes → 재부팅\n",(unsigned)w);
          oh.end(); delay(500); esp_restart();   // 새 펌웨어로 부팅 (리턴 안 함)
        } else Serial.printf("[OTA] end err: %s\n", Update.errorString());
      } else Serial.printf("[OTA] begin err: %s\n", Update.errorString());
    } else Serial.printf("[OTA] HTTP %d\n", hc);
    oh.end();
    updateLed();   // 실패 시 LED 원복
  }
  else if(strncmp(msg,"WIFI:",5)==0){
    char tmp[256]; strlcpy(tmp,msg+5,sizeof(tmp));
    char* comma=strchr(tmp,',');
    if(comma){ *comma='\0'; saveWifiCredentials(tmp,comma+1); }
  }
  else if(strcmp(msg,"UPLOAD_ALL")==0){ startWebDAVAll(); }
  else if(strcmp(msg,"POWEROFF")==0){
    // 완전 꺼짐 = 딥슬립 (car) — A/C 신호 IO17 또는 RESET/전원으로 복귀
    Serial.println("[CMD] POWEROFF (deep sleep) requested");
    enterPowerOff();
  }
  else if(strcmp(msg,"POWERON")==0){
    Serial.println("[CMD] POWERON requested");
    exitPowerOff();
  }
  else if(strncmp(msg,"ACSET:",6)==0){
    // A/C 설정: ACSET:<onAct>,<onMin>,<onNext>,<offAct>,<offMin>
    //   동작=P방지/D감지/I정지, 분=1~240
    //   예) ACSET:P,10,D,D,20  = ON→방지 10분→감지 / OFF→감지 20분→딥슬립
    char tmp[48]; strlcpy(tmp,msg+6,sizeof(tmp));
    char* f[5]; int n=0; char* tok=strtok(tmp,",");
    while(tok && n<5){ f[n++]=tok; tok=strtok(NULL,","); }
    if(n>=5){
      g_acOnAction  = schedActionFromChar(f[0][0]);   // P/D/I → 2/1/0
      g_acOnMin     = (uint32_t)constrain(atoi(f[1]),1,240);
      g_acOnNext    = schedActionFromChar(f[2][0]);
      g_acOffAction = schedActionFromChar(f[3][0]);
      g_acOffMin    = (uint32_t)constrain(atoi(f[4]),1,240);
      saveSettings();
      Serial.printf("[CMD] ACSET on=%d onMin=%lu next=%d off=%d offMin=%lu\n",
        (int)g_acOnAction,(unsigned long)g_acOnMin,(int)g_acOnNext,
        (int)g_acOffAction,(unsigned long)g_acOffMin);
    }
  }
  else if(strncmp(msg,"AUTOUP:",7)==0){
    // 자동 업로드 설정: AUTOUP:<on 0/1>,<hour 0~23>
    char tmp[32]; strlcpy(tmp,msg+7,sizeof(tmp));
    char* comma=strchr(tmp,',');
    if(comma){
      *comma='\0';
      g_autoUpload     = (atoi(tmp)!=0);
      int hr           = atoi(comma+1);
      g_autoUploadHour = (uint32_t)constrain(hr,0,23);
      saveSettings();
      Serial.printf("[CMD] AUTOUP set: %s @ %02d시\n",
        g_autoUpload?"ON":"OFF", (int)g_autoUploadHour);
    }
  }
  else if(strcmp(msg,"TIMESYNC")==0){
    // 수동 시간 동기화: NTP 재요청 후 RTC 재보정
    Serial.println("[CMD] manual TIMESYNC requested");
    if(WiFi.status()==WL_CONNECTED){
      configTime(TZ_OFFSET_SEC,DST_OFFSET_SEC,NTP1,NTP2);   // NTP 재요청
      bool ok=false;
      for(int i=0;i<20;i++){ if(isTimeValid()){ syncRTCfromSystem(); ok=true; break; } delay(100); }
      Serial.println(ok?"[CMD] TIMESYNC OK (RTC updated)":"[CMD] TIMESYNC: time not valid yet");
    } else {
      Serial.println("[CMD] TIMESYNC skip: Wi-Fi not connected");
    }
  }
  else if(strcmp(msg,"REBOOT")==0){
    Serial.println("[CMD] Reboot requested.");
    delay(200);
    esp_restart();
  }
  else if(strcmp(msg,"MSC")==0){
    // MQTT 명령으로 USB 저장소 모드 진입
    // 더블리셋과 동일한 매직넘버를 세팅한 뒤 재부팅
    Serial.println("[CMD] MSC mode requested via MQTT → rebooting...");
    requestMode(MODE_IDLE);   // 현재 모드 안전하게 종료
    delay(300);
    rtc_magic = RST_MAGIC;
    rtc_cnt   = 1;            // checkDoubleReset()이 2번째 리셋으로 인식
    delay(100);
    esp_restart();
  }
  else if(strcmp(msg,"DLMODE")==0){
    // MQTT 명령으로 유선(USB) 펌웨어 다운로드 모드 진입
    Serial.println("[CMD] Wired download mode requested via MQTT");
    requestMode(MODE_IDLE);   // 현재 모드 안전하게 종료
    delay(300);
    enterDownloadMode();      // ROM 다운로드 모드로 재부팅 (절대 리턴 안 함)
  }
  else if(strncmp(msg,"NASSET:",7)==0){
    char tmp[512]; strlcpy(tmp,msg+7,sizeof(tmp));
    char* p1=strchr(tmp,'|');
    if(p1){
      *p1='\0'; strlcpy(g_nasBase,tmp,sizeof(g_nasBase));
      char* p2=strchr(p1+1,'|');
      if(p2){
        *p2='\0'; strlcpy(g_nasDir,p1+1,sizeof(g_nasDir));
        char* p3=strchr(p2+1,'|');
        if(p3){
          *p3='\0'; strlcpy(g_nasUser,p2+1,sizeof(g_nasUser));
          strlcpy(g_nasPass,p3+1,sizeof(g_nasPass));
        }
        saveNasSettings();
      }
    }
  }
}

bool mqttConnect(){
  if(WiFi.status()!=WL_CONNECTED)return false;
  mqttWifiClient.setInsecure();
  mqttClient.setServer(MQTT_HOST,MQTT_PORT);
  mqttClient.setCallback(mqttOnMessage);
  mqttClient.setKeepAlive(15);
  mqttClient.setBufferSize(1024);   // OTA URL 등 긴 명령 수신 대응 (구 512)
  char willMsg[64];
  snprintf(willMsg,sizeof(willMsg),"{\"id\":\"%s\",\"name\":\"%s\",\"online\":false}",g_deviceId,g_deviceName);
  char cid[32]; snprintf(cid,sizeof(cid),"proxi-%s",g_deviceId);
  bool ok=mqttClient.connect(cid,MQTT_USER,MQTT_PASS,g_topicStatus,0,true,willMsg);
  if(ok){
    mqttClient.subscribe(g_topicCmd);
    char om[128];
    snprintf(om,sizeof(om),"{\"id\":\"%s\",\"name\":\"%s\",\"online\":true,\"ip\":\"%s\"}",
      g_deviceId,g_deviceName,WiFi.localIP().toString().c_str());
    mqttClient.publish(g_topicStatus,om,true);
    Serial.printf("[MQTT] connected as %s\n",cid);
  } else Serial.printf("[MQTT] fail rc=%d\n",mqttClient.state());
  return ok;
}

void mqttPublish(){
  if(!mqttClient.connected())return;
  uint32_t now=millis();

  // 현재 모드 문자열
  const char* modeStr = "IDLE";
  if(g_mode==MODE_DETECT)     modeStr="DETECT";
  else if(g_mode==MODE_PREVENT) modeStr="PREVENT";
  else if(g_mode==MODE_TRANSITION) modeStr="TRANSITION";

  uint32_t transRem = 0;
  if(g_mode==MODE_TRANSITION)
    transRem = (uint32_t)max((int32_t)0,(int32_t)TRANSITION_DELAY_MS-(int32_t)(now-g_transitionMs));

  // 시간표 구간 정보 (남은시간/구간길이/다음이벤트)
  int32_t segRem=-1, segLen=-1; char nxStr[16];
  getSchedSeg(&segRem, &segLen, nxStr, sizeof(nxStr));

  // RTC 현재 시각 문자열 생성 ("YYYY-MM-DD HH:MM:SS" KST)
  char g_nowStr[24];
  formatNow(g_nowStr, sizeof(g_nowStr), false);

  // ★ NaN 방어: 측정 전(IDLE 등) latest 값이 NAN이면 0으로 (nan은 invalid JSON)
  float jvt =isnan(latest.vDiff)      ?0.0f:latest.vDiff;
  float jit =isnan(latest.currentA)   ?0.0f:latest.currentA;
  float jv0 =isnan(latest.vCurrentRaw)?0.0f:latest.vCurrentRaw;
  float jv1 =isnan(latest.vVoltageRaw)?0.0f:latest.vVoltageRaw;
  float jtri=isnan(latest.triNorm)    ?0.0f:latest.triNorm;

  snprintf(g_latestJson,sizeof(g_latestJson),
    "{\"id\":\"%s\",\"name\":\"%s\","
    "\"mode\":\"%s\",\"detect\":%s,\"prevent\":%s,"
    "\"vt\":%.3f,\"it\":%.6f,"
    "\"it_pk\":%.6f,\"it_pk_n\":%lu,"
    "\"v_adc0\":%.4f,\"v_adc1\":%.4f,\"tri\":%.3f,"
    "\"seg_rem\":%ld,\"seg_len\":%ld,\"nx\":\"%s\","
    "\"sched\":\"%s\","
    "\"vdc\":%.2f,\"vpp\":%.2f,"
    "\"trans_rem\":%lu,\"nolog\":%d,\"poweroff\":%d,"
    "\"ac_on\":\"%c\",\"ac_onmin\":%lu,\"ac_onnext\":\"%c\","
    "\"ac_off\":\"%c\",\"ac_offmin\":%lu,\"ac_sig\":%d,"
    "\"fw\":\"%s\",\"ssid\":\"%s\","
    "\"ip\":\"%s\",\"time\":\"%s\",\"time_ms\":%lu}",
    g_deviceId,g_deviceName,
    modeStr,
    g_mode==MODE_DETECT?"true":"false",
    g_mode==MODE_PREVENT?"true":"false",
    jvt,jit,
    isnan(g_itPeakLast)?0.0f:g_itPeakLast,(unsigned long)g_itPeakSeq,
    jv0,jv1,jtri,
    (long)segRem,(long)segLen,nxStr,
    g_schedStr,
    g_vdcSet,g_vppSet,
    (unsigned long)transRem,(g_mode==MODE_DETECT && g_detectNoLog)?1:0,g_powerOff?1:0,
    schedCharFromAction(g_acOnAction),(unsigned long)g_acOnMin,schedCharFromAction(g_acOnNext),
    schedCharFromAction(g_acOffAction),(unsigned long)g_acOffMin,
    (digitalRead(AC_SIGNAL_PIN)==HIGH)?1:0,
    FW_VERSION,g_wifiSsid,
    WiFi.localIP().toString().c_str(),
    g_nowStr,(unsigned long)now);
  mqttClient.publish(g_topicData,g_latestJson);
}

void mqttLoop(){
  if(WiFi.status()!=WL_CONNECTED)return;
  if(!mqttClient.connected()){
    if(millis()-lastMqttReconnMs>=MQTT_RECONN_MS){lastMqttReconnMs=millis();mqttConnect();}
  } else {
    mqttClient.loop();
    if(millis()-lastMqttPublishMs>=MQTT_PUBLISH_MS){lastMqttPublishMs=millis();mqttPublish();}
  }
}

// ════════════════════════════════════════════════════════════
//  Web Server — HTML 헬퍼
// ════════════════════════════════════════════════════════════
static const char HTML_HEAD1[] PROGMEM =
  "<!doctype html><html><head>"
  "<meta charset='UTF-8'/>"
  "<meta name='viewport' content='width=device-width,initial-scale=1'/>"
  "<title>";
static const char HTML_HEAD2[] PROGMEM =
  "</title><style>"
  "body{font-family:monospace;margin:16px;background:#1a1a2e;color:#eee}"
  "h3{color:#0af}a{color:#0af}"
  "input[type=text],input[type=password],input[type=number]{"
  "background:#16213e;color:#eee;border:1px solid #0af;"
  "padding:6px;width:100%;box-sizing:border-box;margin:4px 0}"
  "input[type=submit],button{"
  "background:#0af;color:#000;border:none;padding:8px 20px;"
  "cursor:pointer;margin-top:8px;font-size:1em;border-radius:4px}"
  "table{width:100%;border-collapse:collapse}"
  "th{background:#16213e;padding:8px;text-align:left;color:#0af}"
  "td{padding:6px 8px;border-bottom:1px solid #333}"
  ".ib{background:#0d1b2a;border:1px solid #0af;border-radius:8px;padding:16px;margin:12px 0}"
  ".il{color:#aaa;font-size:.85em;margin-bottom:2px}"
  ".iv{color:#0ff;font-size:1.1em;font-weight:bold}"
  "</style></head><body>";
static const char HTML_FOOT[] PROGMEM = "</body></html>";

inline void htmlStart(const char* title){
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200,"text/html","");
  server.sendContent_P(HTML_HEAD1);
  server.sendContent(title);
  server.sendContent_P(HTML_HEAD2);
}
inline void htmlEnd(){ server.sendContent_P(HTML_FOOT); server.sendContent(""); }
inline void hP(const __FlashStringHelper* s){ server.sendContent(s); }
inline void hS(const String& s){ server.sendContent(s); }
inline void hC(const char* s){ server.sendContent(s); }

static bool checkAuth(){ if(!server.authenticate(UPDATE_USER,UPDATE_PASS)){server.requestAuthentication();return false;}return true; }

void handleRoot(){
  htmlStart("PROXI v22");
  hP(F("<h3>PROXI Logger v22</h3><div class='ib'>"));
  hP(F("<div class='il'>ID</div><div class='iv'>")); hC(g_deviceId); hP(F("</div>"));
  hP(F("<div class='il'>이름</div><div class='iv'>")); hC(g_deviceName); hP(F("</div>"));
  hP(F("<div class='il'>모드</div><div class='iv'>"));
  if(g_mode==MODE_DETECT)       hP(F("<span style='color:#00f0f0'>DETECT</span>"));
  else if(g_mode==MODE_PREVENT) hP(F("<span style='color:#00f000'>PREVENT</span>"));
  else if(g_mode==MODE_TRANSITION) hP(F("<span style='color:#ff0'>SWITCHING...</span>"));
  else                          hP(F("<span style='color:#888'>IDLE</span>"));
  hP(F("</div></div>"));
  hP(F("<ul>"
    "<li><a href='/wifi'>Wi-Fi 설정</a></li>"
    "<li><a href='/files'>파일 브라우저</a></li>"
    "<li><a href='/plot'>실시간 그래프</a></li>"
    "<li><a href='/latest'>최신값 JSON</a></li>"
    "<li><a href='/update'>OTA 업데이트 (Wi-Fi)</a></li>"
    "<li><a href='/dl' style='color:#fa0'>유선 펌웨어 업데이트 (USB)</a></li>"
    "<li><a href='/msc' style='color:#f55'>USB 저장소 모드 (MSC)</a></li>"
    "</ul>"));
  htmlEnd();
}

// ── 유선(USB) 펌웨어 업데이트 모드 페이지 ─────────────────────
void handleDlGet(){
  htmlStart("유선 펌웨어 업데이트");
  hP(F("<h3 style='color:#fa0'>유선 펌웨어 업데이트 (USB)</h3><p><a href='/'>← 홈</a></p>"
    "<div class='ib'>"
    "<p>기기를 PC에 <b>USB 케이블</b>로 연결한 뒤 진입하면, 칩이 "
    "<b>ROM 다운로드 모드</b>로 재부팅됩니다. 그 상태에서 PC의 "
    "<b>Arduino IDE</b> 또는 <b>esptool</b>로 펌웨어(.bin)를 구울 수 있습니다.</p>"
    "<p>① 진입 → ② PC에서 해당 COM 포트 선택 → ③ 업로드 → ④ 완료 후 자동 정상 부팅</p>"
    "<p style='color:#fa0'>⚠ 진입 시 기기가 재부팅되고 펌웨어가 멈춥니다(다운로드 모드 대기).<br>"
    "그냥 빠져나오려면 전원을 껐다 켜세요.</p>"
    "<p style='color:#888;font-size:.85em'>※ 케이스에 BOOT/RESET 버튼이 없어도 "
    "버튼 없이 다운로드 모드로 들어갑니다.</p>"
    "</div>"
    "<form method='POST' action='/dl' "
    "onsubmit=\"return confirm('유선 업데이트(다운로드) 모드로 진입합니다.\\n기기가 재부팅됩니다. 계속할까요?');\">"
    "<input type='submit' value='유선 업데이트 모드 진입' "
    "style='background:#fa0;color:#000'/></form>"));
  htmlEnd();
}
void handleDlPost(){
  htmlStart("유선 펌웨어 업데이트");
  hP(F("<h3>다운로드 모드로 진입 중...</h3>"
    "<p>기기가 곧 재부팅되어 ROM 다운로드 모드로 대기합니다.<br>"
    "PC의 Arduino IDE / esptool 에서 COM 포트를 선택해 펌웨어를 업로드하세요.<br>"
    "업로드가 끝나면 자동으로 정상 부팅됩니다.</p>"));
  htmlEnd();
  Serial.println("[CMD] Wired download mode requested via WebUI");
  requestMode(MODE_IDLE);   // 현재 모드 안전하게 종료
  delay(500);
  enterDownloadMode();      // ROM 다운로드 모드로 재부팅 (절대 리턴 안 함)
}

// ── USB MSC 모드 진입 페이지 ──────────────────────────────────
void handleMscGet(){
  htmlStart("USB 저장소 모드");
  hP(F("<h3 style='color:#f55'>USB 저장소 모드 (MSC)</h3><p><a href='/'>← 홈</a></p>"
    "<div class='ib'>"
    "<p>기기를 PC에 USB로 연결한 상태에서 진입하면, 내부 저장소가 "
    "<b>USB 이동식 디스크</b>로 인식되어 로그 파일을 직접 복사할 수 있습니다.</p>"
    "<p style='color:#fa0'>⚠ 진입 시 기기가 재부팅되며 감지/방지 동작이 중단됩니다.<br>"
    "복귀하려면 전원을 껐다 켜거나 기기 버튼으로 리셋하세요.</p>"
    "</div>"
    "<form method='POST' action='/msc' "
    "onsubmit=\"return confirm('USB 저장소 모드로 진입합니다.\\n기기가 재부팅됩니다. 계속할까요?');\">"
    "<input type='submit' value='USB 저장소 모드 진입' "
    "style='background:#f55;color:#fff'/></form>"));
  htmlEnd();
}
void handleMscPost(){
  htmlStart("USB 저장소 모드");
  hP(F("<h3>진입 중...</h3>"
    "<p>기기가 곧 재부팅되어 USB 저장소 모드로 들어갑니다.<br>"
    "PC에서 이동식 디스크가 나타날 때까지 잠시 기다려 주세요.</p>"));
  htmlEnd();
  // MQTT "MSC" 명령과 동일한 진입 절차
  Serial.println("[CMD] MSC mode requested via WebUI → rebooting...");
  requestMode(MODE_IDLE);   // 현재 모드 안전하게 종료
  delay(500);
  rtc_magic = RST_MAGIC;
  rtc_cnt   = 1;            // checkDoubleReset()이 2번째 리셋으로 인식
  delay(100);
  esp_restart();
}

void handleWifiGet(){
  htmlStart("Wi-Fi 설정");
  hP(F("<h3>Wi-Fi 설정</h3><p><a href='/'>← 홈</a></p>"
    "<form method='POST' action='/wifi'>"
    "<label>SSID</label><input type='text' name='ssid' required/>"
    "<label>Password</label><input type='password' name='pass'/>"
    "<input type='submit' value='저장'/></form><hr/><p>현재 SSID: <b style='color:#0af'>"));
  hC(g_wifiSsid);
  hP(F("</b></p>"));
  htmlEnd();
}
void handleWifiPost(){
  if(!server.hasArg("ssid")||server.arg("ssid").isEmpty()){server.send(400,"text/plain","SSID필요");return;}
  saveWifiCredentials(server.arg("ssid").c_str(),server.arg("pass").c_str());
  htmlStart("Wi-Fi 설정");
  hP(F("<h3>저장 완료</h3><p>재부팅해주세요.</p><p><a href='/'>← 홈</a></p>"));
  htmlEnd();
}

void handleFiles(){
  FS* fs=activeFS(); if(!fs){server.send(500,"text/plain","No storage");return;}
  if(logFile)logFile.flush();
  struct FileEntry{ String name; size_t size; };
  std::vector<FileEntry> files;
  File root=fs->open("/");
  if(root&&root.isDirectory()){
    root.rewindDirectory();
    while(true){ File f=root.openNextFile(); if(!f) break; if(!f.isDirectory()) files.push_back({String(f.name()),f.size()}); f.close(); yield(); }
    root.close();
  }
  std::sort(files.begin(),files.end(),[](const FileEntry&a,const FileEntry&b){return a.name>b.name;});
  htmlStart("파일 브라우저");
  hP(F("<h3>파일 브라우저</h3><p><a href='/'>← 홈</a></p>"));
  hP(F("<style>.sa{background:#0af;color:#000;border:none;padding:4px 10px;border-radius:4px;font-size:11px;margin:2px}"
    ".td{background:#f44;color:#fff;border:none;padding:6px 14px;border-radius:4px;font-size:12px;margin:2px;cursor:pointer}"
    ".ck{width:16px;height:16px;accent-color:#0af;cursor:pointer}</style>"));
  hP(F("<div style='margin-bottom:8px'>"
    "<button class='sa' onclick='batchDL()'>선택 다운로드</button>"
    "<button class='td' onclick='batchDel()'>선택 삭제</button></div>"));
  hP(F("<table><tr><th><input type='checkbox' class='ck' id='allCk' onchange='toggleAll(this)'/></th>"
    "<th>파일명</th><th>크기</th><th>작업</th></tr>"));
  for(auto& fe:files){
    char szS[16],buf[512];
    if(fe.size>=1048576) snprintf(szS,16,"%.1fMB",fe.size/1048576.0f);
    else if(fe.size>=1024) snprintf(szS,16,"%.1fKB",fe.size/1024.0f);
    else snprintf(szS,16,"%uB",(unsigned)fe.size);
    String nm=fe.name.startsWith("/")?fe.name:"/"+fe.name;
    snprintf(buf,sizeof(buf),
      "<tr><td><input type='checkbox' class='ck fc' value='%s'/></td>"
      "<td>%s</td><td>%s</td><td>"
      "<a style='color:#0f0;margin-right:8px' href='/download?f=%s'>다운로드</a>"
      "<a style='color:#f55' href='/delete?f=%s' onclick=\"return confirm('삭제?')\">삭제</a>"
      "</td></tr>",nm.c_str(),fe.name.c_str(),szS,nm.c_str(),nm.c_str());
    hC(buf);
  }
  hP(F("</table>"));
  hP(F("<script>"
    "function toggleAll(e){document.querySelectorAll('.fc').forEach(c=>c.checked=e.checked);}"
    "function getChecked(){return[...document.querySelectorAll('.fc:checked')].map(c=>c.value);}"
    "function batchDL(){var fs=getChecked();if(!fs.length){alert('파일선택');return;}"
      "fs.forEach(function(f){var a=document.createElement('a');a.href='/download?f='+encodeURIComponent(f);"
      "a.download='';document.body.appendChild(a);a.click();document.body.removeChild(a);});}"
    "function batchDel(){var fs=getChecked();if(!fs.length){alert('파일선택');return;}"
      "if(!confirm(fs.length+'개 삭제?'))return;"
      "var i=0;function nx(){if(i>=fs.length){location.reload();return;}"
      "fetch('/delete?f='+encodeURIComponent(fs[i++])).then(nx).catch(nx);}nx();}"
    "</script>"));
  htmlEnd();
}

void handleDelete(){
  if(!server.hasArg("f")){server.send(400,"text/plain","missing ?f=");return;}
  FS* fs=activeFS();if(!fs){server.send(500,"text/plain","no storage");return;}
  String path=server.arg("f"); if(!path.startsWith("/"))path="/"+path;
  if(path==String(currentLogPath)){server.send(403,"text/plain","기록중 파일");return;}
  if(fs->remove(path)){server.sendHeader("Location","/files");server.send(303);}
  else server.send(500,"text/plain","삭제실패");
}

void handleDownload(){
  FS* fs=activeFS();if(!fs){server.send(500,"text/plain","no storage");return;}
  if(!server.hasArg("f")){server.send(400,"text/plain","missing ?f=");return;}
  String p=server.arg("f"); if(!p.startsWith("/"))p="/"+p;
  if(logFile)logFile.flush();
  File f=fs->open(p,FILE_READ);if(!f){server.send(404,"text/plain","not found");return;}
  String fn=p.substring(p.lastIndexOf('/')+1);
  server.sendHeader("Connection","close");
  server.sendHeader("Content-Disposition","attachment; filename=\""+fn+"\"");
  server.streamFile(f,p.endsWith(".csv")?"text/csv":"application/octet-stream");
  f.close();
}

void handleLatest(){
  const char* modeStr="IDLE";
  if(g_mode==MODE_DETECT)     modeStr="DETECT";
  else if(g_mode==MODE_PREVENT) modeStr="PREVENT";
  else if(g_mode==MODE_TRANSITION) modeStr="TRANSITION";
  uint32_t transRem=0;
  if(g_mode==MODE_TRANSITION)
    transRem=(uint32_t)max((int32_t)0,(int32_t)TRANSITION_DELAY_MS-(int32_t)(millis()-g_transitionMs));
  snprintf(g_latestJson,sizeof(g_latestJson),
    "{\"id\":\"%s\",\"name\":\"%s\","
    "\"mode\":\"%s\",\"detect\":%s,\"prevent\":%s,"
    "\"vt\":%.3f,\"it\":%.6f,"
    "\"v_adc0\":%.4f,\"v_adc1\":%.4f,\"tri\":%.3f,"
    "\"d_intv\":%lu,\"d_dur\":%lu,"
    "\"p_intv\":%lu,\"p_dur\":%lu,"
    "\"vdc\":%.2f,\"vpp\":%.2f,"
    "\"trans_rem\":%lu,"
    "\"fw\":\"%s\",\"ssid\":\"%s\","
    "\"log_file\":\"%s\",\"ip\":\"%s\","
    "\"time_ms\":%lu}",
    g_deviceId,g_deviceName,
    modeStr,
    g_mode==MODE_DETECT?"true":"false",
    g_mode==MODE_PREVENT?"true":"false",
    isnan(latest.vDiff)?0.0f:latest.vDiff, isnan(latest.currentA)?0.0f:latest.currentA,
    isnan(latest.vCurrentRaw)?0.0f:latest.vCurrentRaw, isnan(latest.vVoltageRaw)?0.0f:latest.vVoltageRaw, isnan(latest.triNorm)?0.0f:latest.triNorm,
    (unsigned long)g_detectIntervalHr,(unsigned long)g_detectDurationMin,
    (unsigned long)g_preventIntervalHr,(unsigned long)g_preventDurationMin,
    g_vdcSet,g_vppSet,
    (unsigned long)transRem,
    FW_VERSION,g_wifiSsid,
    currentLogPath,
    WiFi.status()==WL_CONNECTED?WiFi.localIP().toString().c_str():"",
    (unsigned long)millis());
  server.send(200,"application/json",g_latestJson);
}

static const char PLOT_PAGE[] PROGMEM =
"<!doctype html><html><head><meta charset='UTF-8'/>"
"<meta name='viewport' content='width=device-width,initial-scale=1'/>"
"<title>Live Graph</title></head>"
"<body style='font-family:monospace;margin:12px;background:#1a1a2e;color:#eee'>"
"<h3 style='color:#0af'>Live Graph</h3><div id='info'>loading...</div>"
"<canvas id='cv' width='980' height='360' style='border:1px solid #0af;max-width:100%;height:auto'></canvas>"
"<script>\n"
"const cv=document.getElementById('cv'),ctx=cv.getContext('2d'),info=document.getElementById('info');\n"
"let buf=[];\n"
"function mX(x,a,b){return 50+(cv.width-70)*((x-a)/(b-a));}\n"
"function mY(y,a,b){return 20+(cv.height-50)*(1-((y-a)/(b-a)));}\n"
"function render(){if(buf.length<2)return;ctx.clearRect(0,0,cv.width,cv.height);\n"
"let xn=buf[0].v,xx=buf[0].v,yn=buf[0].i,yx=buf[0].i;\n"
"for(const p of buf){xn=Math.min(xn,p.v);xx=Math.max(xx,p.v);yn=Math.min(yn,p.i);yx=Math.max(yx,p.i);}\n"
"if(xn===xx){xn-=1;xx+=1;}if(yn===yx){yn-=1;yx+=1;}\n"
"ctx.strokeStyle='#0af';ctx.strokeRect(50,20,cv.width-70,cv.height-50);\n"
"ctx.strokeStyle='#ff0';ctx.beginPath();\n"
"buf.forEach((p,n)=>{const x=mX(p.v,xn,xx),y=mY(p.i,yn,yx);n===0?ctx.moveTo(x,y):ctx.lineTo(x,y);});\n"
"ctx.stroke();}\n"
"async function tick(){try{const j=await(await fetch('/latest',{cache:'no-store'})).json();\n"
"if(j.mode==='DETECT'){buf.push({v:+j.vt,i:+j.it});if(buf.length>200)buf.shift();}\n"
"info.textContent='['+j.name+'] '+j.mode+' Vt='+Number(j.vt).toFixed(3)+' It='+Number(j.it).toFixed(6);\n"
"render();}catch(e){info.textContent='err:'+e;}}\n"
"setInterval(tick,200);tick();\n"
"</script></body></html>";

void handlePlot(){ server.send_P(200,"text/html",PLOT_PAGE); }

static const char UPDATE_PAGE[] PROGMEM =
  "<!doctype html><html><head><meta charset='UTF-8'/>"
  "<title>펌웨어 업데이트</title></head>"
  "<body style='font-family:monospace;margin:16px;background:#1a1a2e;color:#eee'>"
  "<h3 style='color:#0af'>펌웨어 업데이트</h3><p><a href='/' style='color:#0af'>← 홈</a></p>"
  "<form method='POST' action='/update' enctype='multipart/form-data' id='uf'>"
  "<input type='file' name='firmware' accept='.bin' id='ff' "
  "style='background:#16213e;color:#eee;border:1px solid #0af;padding:8px;width:100%;border-radius:4px;margin-bottom:8px'/><br/>"
  "<button type='submit' id='ub' style='background:#0af;color:#000;border:none;padding:10px 24px;"
  "border-radius:4px;font-size:14px;cursor:pointer;width:100%'>업로드</button></form>"
  "<div id='pg' style='display:none;margin-top:12px'>"
  "<div style='background:#16213e;height:20px;border-radius:10px;overflow:hidden'>"
  "<div id='pb' style='height:100%;background:#0af;width:0%;border-radius:10px'></div></div>"
  "<div id='pt' style='text-align:center;color:#aaa;font-size:12px;margin-top:4px'>업로드 중...</div></div>"
  "<script>"
  "document.getElementById('uf').onsubmit=function(e){"
  "e.preventDefault();var f=document.getElementById('ff').files[0];"
  "if(!f){alert('.bin 파일을 선택하세요');return;}"
  "document.getElementById('ub').disabled=true;"
  "document.getElementById('pg').style.display='block';"
  "var x=new XMLHttpRequest();x.open('POST','/update');"
  "x.upload.onprogress=function(e){if(e.lengthComputable){"
  "var p=Math.round(e.loaded/e.total*100);"
  "document.getElementById('pb').style.width=p+'%';"
  "document.getElementById('pt').textContent=p+'%';}};"
  "x.onload=function(){"
  "document.getElementById('pt').textContent=x.responseText==='OK'?'완료! 재부팅해주세요.':'실패';"
  "document.getElementById('pb').style.background=x.responseText==='OK'?'#0f0':'#f44';"
  "document.getElementById('pb').style.width='100%';};"
  "var fd=new FormData();fd.append('firmware',f);x.send(fd);};"
  "</script></body></html>";

void handleUpdateGet(){ if(!checkAuth()) return; server.send_P(200,"text/html",UPDATE_PAGE); }
void handleUpdateDone(){
  if(!checkAuth()) return;
  server.sendHeader("Connection","close");
  server.send(200,"text/plain",!Update.hasError()?"OK":"FAIL");
}
void handleUpdateUpload(){
  if(!checkAuth()) return;
  HTTPUpload& up=server.upload();
  if     (up.status==UPLOAD_FILE_START)   { if(logFile) logFile.flush(); Update.begin(UPDATE_SIZE_UNKNOWN,U_FLASH); }
  else if(up.status==UPLOAD_FILE_WRITE)   { Update.write(up.buf,up.currentSize); }
  else if(up.status==UPLOAD_FILE_END)     { Update.end(true); }
  else if(up.status==UPLOAD_FILE_ABORTED) { Update.end(); }
}

void setupWebServer(){
  server.on("/",        handleRoot);
  server.on("/latest",  handleLatest);
  server.on("/plot",    handlePlot);
  server.on("/files",   handleFiles);
  server.on("/download",handleDownload);
  server.on("/delete",  handleDelete);
  server.on("/wifi",    HTTP_GET,  handleWifiGet);
  server.on("/wifi",    HTTP_POST, handleWifiPost);
  server.on("/update",  HTTP_GET,  handleUpdateGet);
  server.on("/update",  HTTP_POST, handleUpdateDone, handleUpdateUpload);
  server.on("/dl",      HTTP_GET,  handleDlGet);
  server.on("/dl",      HTTP_POST, handleDlPost);
  server.on("/msc",     HTTP_GET,  handleMscGet);
  server.on("/msc",     HTTP_POST, handleMscPost);
  server.begin();
  Serial.println("[WEB] started");
}

// ════════════════════════════════════════════════════════════
//  더블리셋 감지 — RTC 슬로우 메모리 (5초 타임아웃, LED 피드백)
// ════════════════════════════════════════════════════════════
#include <esp_system.h>

RTC_NOINIT_ATTR uint32_t rtc_magic;
RTC_NOINIT_ATTR uint32_t rtc_cnt;

// RST_MAGIC, rtc_magic, rtc_cnt → 파일 상단에서 선언됨
constexpr uint32_t RST_CLEAR_MS = 5000;         // 5초 (v20.4: 10초)

bool checkDoubleReset(){
  Serial.printf("[RST] magic=0x%08X cnt=%lu reason=%d\n",
    (unsigned)rtc_magic,(unsigned long)rtc_cnt,(int)esp_reset_reason());

  if(rtc_magic == RST_MAGIC){
    if(rtc_cnt >= 1){
      rtc_cnt = 0;
      Serial.println("[RST] DOUBLE RESET → MSC mode");
      return true;
    }
    rtc_cnt = 1;
    Serial.println("[RST] 1st reset → 빨강 LED 피드백 시작");
    return false;
  }
  rtc_magic = RST_MAGIC;
  rtc_cnt   = 0;
  Serial.println("[RST] power-on boot");
  return false;
}

// loop()에서 호출 — 1번 리셋 후 5초간 빨강 LED 빠른 점멸
void handleFirstResetFeedback(){
  static bool done = false;
  if(done || rtc_cnt != 1) return;
  uint32_t now = millis();
  if(now >= RST_CLEAR_MS){
    rtc_cnt = 0;
    done = true;
    updateLed();
    Serial.println("[RST] cnt auto-cleared after 5s");
    return;
  }
  // 250ms 주기 빨강 점멸 (논블로킹)
  static uint32_t lastBlink = 0;
  static bool blinkOn = false;
  if(now - lastBlink >= 250){
    lastBlink = now;
    blinkOn = !blinkOn;
    if(blinkOn) ledRed(); else ledOff();
  }
}

// ════════════════════════════════════════════════════════════
//  USB MSC — SD.readRAW / SD.writeRAW 방식 (v21.9)
//
//  CSNP1GCR01-BOW는 SD.h(SPI 모드)와 완벽히 동작함이 이미 검증됨.
//  raw SPI 재구현 없이 SD 라이브러리 내장 드라이버를 그대로 사용.
//  SD.end() 호출 없이 SD.h 유지 → readRAW/writeRAW로 섹터 직접 접근.
// ════════════════════════════════════════════════════════════

// ────────────────────────────────────────────────────────────
//  TinyUSB MSC 콜백 — SD.readRAW / SD.writeRAW 사용
//  SD.h SPI 드라이버가 이미 CSNP1GCR01-BOW와 검증됨.
//  raw SPI 재구현 없이 라이브러리 내부 드라이버 그대로 활용.
// ────────────────────────────────────────────────────────────
static int32_t onMscRead(uint32_t lba, uint32_t offset, void* buf, uint32_t bufsize){
  if(!s_mscReady) return -1;
  (void)offset;                           // TinyUSB: 항상 offset=0
  uint8_t* dst     = (uint8_t*)buf;
  uint32_t sectors = bufsize / 512;
  for(uint32_t i = 0; i < sectors; i++){
    if(!SD.readRAW(dst + i * 512, lba + i)){
      Serial.printf("[MSC] readRAW fail lba=%lu\n", (unsigned long)(lba+i));
      return -1;
    }
  }
  return (int32_t)bufsize;
}

static int32_t onMscWrite(uint32_t lba, uint32_t offset, uint8_t* buf, uint32_t bufsize){
  if(!s_mscReady) return -1;
  (void)offset;
  uint32_t sectors = bufsize / 512;
  for(uint32_t i = 0; i < sectors; i++){
    if(!SD.writeRAW(buf + i * 512, lba + i)){
      Serial.printf("[MSC] writeRAW fail lba=%lu\n", (unsigned long)(lba+i));
      return -1;
    }
  }
  return (int32_t)bufsize;
}

// ────────────────────────────────────────────────────────────
//  MSC 모드 진입 (더블리셋 또는 MQTT "MSC" 명령)
// ────────────────────────────────────────────────────────────
void runUsbMscMode(){
  // ★ RTC 카운터 즉시 클리어 — WDT 리셋 시 MSC 무한루프 방지
  rtc_cnt   = 0;
  rtc_magic = 0;

  Serial.println("[MSC] entering USB mass-storage mode...");

  // 열린 파일(로그) 먼저 닫기
  if(logFile){ logFile.close(); }

  // SD가 초기화 안 된 경우 재시도
  if(!useSD){
    Serial.println("[MSC] SD not ready, initializing...");
    sdSpi.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    delay(200);
    if(SD.begin(SD_CS_PIN, sdSpi)){
      useSD = true;
      Serial.println("[MSC] SD init OK");
    } else {
      Serial.println("[MSC] SD init FAILED");
      setRgbLed(1,0,0);
      while(true){ delay(300); ledOff(); delay(300); setRgbLed(1,0,0); esp_task_wdt_reset(); }
    }
  }

  // 섹터 수: SD.cardSize() 바이트 → 512 나눔
  s_mscSectors = (uint32_t)(SD.cardSize() / 512ULL);
  if(s_mscSectors == 0){
    s_mscSectors = 2097152UL;   // 폴백: 1GB
    Serial.println("[MSC] cardSize 0 → fallback 1GB");
  }
  Serial.printf("[MSC] sectors=%lu (%.0f MB)\n",
    (unsigned long)s_mscSectors, s_mscSectors * 512.0f / 1048576.0f);

  // 섹터 0 검증 — SD.readRAW로 실제 읽기 확인
  {
    static uint8_t sec0[512];
    if(SD.readRAW(sec0, 0)){
      Serial.printf("[MSC] Sector0[0..3]: %02X %02X %02X %02X\n",
        sec0[0], sec0[1], sec0[2], sec0[3]);
    } else {
      Serial.println("[MSC] Sector0 readRAW FAILED");
      setRgbLed(1,0,0);
      while(true){ delay(300); ledOff(); delay(300); setRgbLed(1,0,0); esp_task_wdt_reset(); }
    }
  }

  s_mscReady = true;

  msc.vendorID("PROXI");
  msc.productID("SDNAND");
  msc.productRevision("1.0");
  msc.onRead(onMscRead);
  msc.onWrite(onMscWrite);
  msc.isWritable(true);      // 쓰기 허용 (없으면 Windows가 읽기전용 처리)
  msc.mediaPresent(true);
  msc.begin(s_mscSectors, 512);
  USB.begin();

  Serial.printf("[MSC] USB started  sectors=%lu\n", (unsigned long)s_mscSectors);
  setRgbLed(1, 0, 0);   // 빨간색 유지
  while(true){ esp_task_wdt_reset(); vTaskDelay(1); }
}

// ════════════════════════════════════════════════════════════
//  setup
// ════════════════════════════════════════════════════════════
void setup(){
  Serial.begin(115200); delay(300);

  // I2C 초기화 (SDA=8, SCL=9)
  Wire.begin(8, 9);

  // 출력 핀 초기화
  pinMode(LED_R_PIN,    OUTPUT); digitalWrite(LED_R_PIN,    HIGH);
  pinMode(LED_G_PIN,    OUTPUT); digitalWrite(LED_G_PIN,    HIGH);
  pinMode(LED_B_PIN,    OUTPUT); digitalWrite(LED_B_PIN,    HIGH);
  pinMode(PIN_DETECT_EN,OUTPUT); digitalWrite(PIN_DETECT_EN,LOW);
  pinMode(PIN_PRT_EN,   OUTPUT); digitalWrite(PIN_PRT_EN,   LOW);
  pinMode(PIN_CNT_TMZ,  OUTPUT); digitalWrite(PIN_CNT_TMZ,  LOW);
  pinMode(SD_CS_PIN,    OUTPUT); digitalWrite(SD_CS_PIN,    HIGH);
  pinMode(PIN_SW,       INPUT_PULLUP);
  pinMode(AC_SIGNAL_PIN, INPUT_PULLDOWN);   // ★ A/C 신호 입력 (HIGH=ON), 풀다운

  ledRed();

  // 더블리셋 확인 → MSC 모드
  bool dbl = checkDoubleReset();
  if(dbl){
    Serial.println("[BOOT] MSC mode");
    runUsbMscMode();
  }

  // 일반 부팅
  setupStorage();
  loadWifiCredentials();
  loadSettings();
  loadNasSettings();

  // RTC 초기화 (컴파일타임 기준)
  initRTC();

  // DAC 초기화 + 영점 캘리브레이션
  triStartMs = lastLogMs = millis();
  setDACtoZero();
  calibrateZeroOffset();

  // 포텐셔미터 초기화
  applyPotentiometer();

  connectWiFiOnce();
  initDeviceId();
  setupWebServer();

  if(WiFi.status()==WL_CONNECTED){
    char mdn[32]; snprintf(mdn,sizeof(mdn),"proxi-%s",g_deviceId);
    if(MDNS.begin(mdn)) Serial.printf("[mDNS] http://%s.local\n",mdn);
    mqttConnect();
    ledBlink(5,1,0,1,300);  // 보라색 5번 점멸 — Wi-Fi 연결 성공
    Serial.println("=============================");
    Serial.printf("[READY] FW: %s\n",FW_VERSION);
    Serial.printf("[READY] IP: %s\n",WiFi.localIP().toString().c_str());
    Serial.println("=============================");
  } else {
    char ap[24]; snprintf(ap,sizeof(ap),"PROXI-%s",g_deviceId);
    WiFi.mode(WIFI_AP); WiFi.softAP(ap);
    if(MDNS.begin("proxi")) Serial.println("[mDNS] http://proxi.setup");
    g_isApMode=true;
    ledBlink(5,0,1,0,300);  // 녹색 5번 점멸 — AP 모드 진입
    Serial.printf("[READY] AP: %s  URL: http://192.168.4.1\n",ap);
  }

  // ── 전원 인가 시 NAS 자동 업로드 (car_v4) ───────────────────
  //  부팅(콜드부팅 또는 딥슬립 ext0 wakeup)마다 이전 세션 로그를 NAS로 업로드.
  //  background task로 동작 — 진행 중인 로그(currentLogPath)는 제외.
  if(WiFi.status()==WL_CONNECTED && g_nasBase[0]){
    Serial.println("[BOOT] 전원 인가 → NAS 자동 업로드 시작");
    startWebDAVAll();
  }

  // ── A/C 신호 초기화 (car_v4) ─────────────────────────────────
  //  ★ 전원 인가는 항상 IDLE(정지)로 시작.
  //  g_acPrevHigh=false 로 둬서, A/C가 이미 ON 상태면 loop의 checkAcSignal이
  //  '새 ON(상승엣지)'으로 인식해 ON 시퀀스를 시작한다(딥슬립 wake 포함).
  //  A/C OFF면 IDLE 유지(설정용으로 awake, 자동 sleep 안 함).
  {
    g_acPrevHigh   = false;
    g_acOnTimerOn  = false;
    g_acOffTimerOn = false;
    requestMode(MODE_IDLE);
    Serial.println("[AC] boot: IDLE 시작 (A/C 신호 감지 대기)");
  }

  updateLed();
}

// ════════════════════════════════════════════════════════════
//  loop
// ════════════════════════════════════════════════════════════
void loop(){
  uint32_t now=millis();

  handleFirstResetFeedback();
  handleTransition();
  server.handleClient();
  wifiKeepAlive();
  mqttLoop();

  // ── 정각 RTC 재보정 (20초마다) ─ car는 매일자동업로드 미사용 ──
  static uint32_t lastDailyChk=0;
  if(now-lastDailyChk >= 20000){
    lastDailyChk=now;
    checkHourlyTimeSync();   // 매 정각 NTP→RTC 재보정
  }

  // ── A/C 신호 상태머신 점검 (300ms) ─ car는 시간표 대신 A/C로 제어 ──
  static uint32_t lastAcChk=0;
  if(now-lastAcChk >= 300){ lastAcChk=now; checkAcSignal(); }

  // ── DETECT 중 측정 및 로깅 ─────────────────────────────────
  if(g_mode == MODE_DETECT){
    updateDAC();
    if(now-lastLogMs >= LOG_INTERVAL_MS){
      lastLogMs=now;
      float ph=(float)((now-triStartMs)%TRI_PERIOD_MS)/(float)TRI_PERIOD_MS;
      float tri=triNorm(ph);
      float vc=ads_read(0), vv=ads_read(1);
      if(isnan(vc)||isnan(vv)) return;
      float it=(vc-g_vOffsetCal)/(R_SHUNT*INA_GAIN);
      float vt=VDIFF_GAIN*(vv-VDIFF_CENTER_V);
      latest={now,tri,vc,vv,vt,it,true};
      if(it>g_itPeakCur) g_itPeakCur=it;        // 주기 내 전류 최대값 갱신
      if(!g_detectNoLog) appendLogLine(now,tri,vc,vv,vt,it);  // 출력만 모드: 파일 기록 생략
    }
    // 파형 1주기(TRI_PERIOD_MS=20초) 완료 → 그 구간 전류 피크 확정 + 시퀀스 증가
    //  (대시보드가 it_pk_n 변화를 감지해 꺾은선 그래프에 점 추가)
    if(now-g_prevPeriodMs >= TRI_PERIOD_MS){
      g_prevPeriodMs=now;
      g_itPeakLast=g_itPeakCur;
      g_itPeakCur=0.0f;
      g_itPeakSeq++;
      Serial.printf("[DET] 주기 #%lu 전류피크 = %.6f A\n",
        (unsigned long)g_itPeakSeq, g_itPeakLast);
    }
  }
  // (TFT 제거됨 — 화면 갱신 없음)

  latest.detecting = (g_mode==MODE_DETECT);
  yield();
}
