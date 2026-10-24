/* ============================================================================
 *  ESP32 - DUT (Device Under Test) cho Logic Analyzer 16 kênh  —  BẢN v2
 *
 *  Sinh tín hiệu 2 CHIỀU THẬT (MISO / RX / ACK) để verify decoder chuyên nghiệp.
 *  Khi thầy cắm IC I2C/SPI/UART thật -> GỠ jumper self-slave, decoder chạy y hệt.
 *
 *  SƠ ĐỒ 16 KÊNH (GPIO -> i_probe FPGA, giữ nguyên dây vật lý):
 *    CH-1  GPIO17  UART-A TX        (Serial1)
 *    CH-2  GPIO21  UART-A RX        (Serial2, đối thoại chéo)
 *    CH-3  GPIO22  I2C-A SDA        (Wire master + Wire1 slave@0x3C)
 *    CH-4  GPIO5   I2C-A SCL
 *    CH-5  GPIO18  SPI-A SCK        (VSPI master)
 *    CH-6  GPIO23  SPI-A MOSI
 *    CH-7  GPIO16  SPI-A MISO       (HSPI slave xuất data)
 *    CH-8  GPIO19  SPI-A CS
 *    CH-9  GPIO2   UART-B TX        (bit-bang)
 *    CH-10 GPIO4   I2C-B SDA        (bit-bang master)
 *    CH-11 GPIO12  I2C-B SCL        (bit-bang master)
 *    CH-12 GPIO13  PWM 1kHz         (LEDC)
 *    CH-13 GPIO14  PWM 10kHz        (LEDC)
 *    CH-14 GPIO15  CLK 1MHz         (LEDC)
 *    CH-15 GPIO25  CLK 500kHz       (LEDC)
 *    CH-16 GPIO26  CLK 100kHz       (LEDC)
 *
 *  JUMPER cần cắm (self-slave, chỉ khi dev không có IC thật):
 *    I2C-A : GPIO22 <-> GPIO32 (SDA) ; GPIO5  <-> GPIO33 (SCL)
 *    SPI-A : GPIO18  -> GPIO34 (SCK) ; GPIO23 -> GPIO35 (MOSI) ; GPIO19 -> GPIO36 (CS)
 *    UART-A: GPIO17  -> GPIO27 (U2 RX); GPIO21 -> GPIO14?  -> để riêng, xem UART_A_RX_IN
 *
 *  Lưu ý: SPI-slave dùng ESP-IDF driver (driver/spi_slave.h). Nếu gặp trục trặc
 *  timing, có thể #define ENABLE_SPI_SLAVE 0 -> MISO nằm im (decoder vẫn được
 *  bảo chứng bằng test offline gui/test_decode_offline.py).
 * ============================================================================ */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "driver/spi_slave.h"

#define ENABLE_SPI_SLAVE 1     // 1: HSPI slave xuất MISO thật; 0: bỏ (MISO im)

// ----------------------------- Chân (theo sơ đồ trên) -----------------------
#define PIN_UART_A_TX 17
#define PIN_UART_A_RX_DEV 21   // "thiết bị" trả lời -> chính là đường CH-2
#define PIN_I2C_A_SDA 22
#define PIN_I2C_A_SCL 5
#define PIN_SPI_A_SCK 18
#define PIN_SPI_A_MOSI 23
#define PIN_SPI_A_MISO 16
#define PIN_SPI_A_CS 19
#define PIN_UART_B_TX 2
#define PIN_I2C_B_SDA 4
#define PIN_I2C_B_SCL 12
#define PIN_PWM_1KHZ 13
#define PIN_PWM_10KHZ 14
#define PIN_CLK_1MHZ 15
#define PIN_CLK_500KHZ 25
#define PIN_CLK_100KHZ 26

// Chân slave (KHÔNG probe, nối jumper tới master)
#define PIN_I2C1_SDA 32        // Wire1 slave SDA  (<-> 22)
#define PIN_I2C1_SCL 33        // Wire1 slave SCL  (<-> 5)
#define PIN_SPI_SLV_SCK 34     // HSPI slave SCK in (<- 18)  (input-only OK)
#define PIN_SPI_SLV_MOSI 35    // HSPI slave MOSI in (<- 23)
#define PIN_SPI_SLV_CS 36      // HSPI slave CS in (<- 19)

const uint8_t I2C_A_ADDR = 0x3C;

// ----------------------------- Dữ liệu mẫu (đã biết) ------------------------
const uint8_t UART_A_PATTERN[] = {0x55, 0xAA, 0x00, 0xFF, 0x3C};
const char    UART_B_TEXT[]    = "Hi!";
uint8_t SPI_MOSI_PATTERN[] = {0xAA, 0x55, 0x3C, 0x00};
volatile uint8_t spiSlaveResp[4] = {0x1F, 0x2E, 0x3D, 0x4C};   // MISO trả về

HardwareSerial UARTA(1);       // UART-A TX  -> GPIO17
HardwareSerial UARTA_DEV(2);   // "thiết bị" -> GPIO21 (= CH-2 RX)
SPIClass vspi(VSPI);

// ============================================================================
//  I2C-A slave (Wire1) — trả ACK + data thật cho master
// ============================================================================
volatile uint8_t i2cSlaveMem = 0xA5;
void onI2cReceive(int n) { while (Wire1.available()) i2cSlaveMem = Wire1.read(); }
void onI2cRequest()      { Wire1.write(i2cSlaveMem); }   // master đọc -> có data

// ============================================================================
//  SPI-A slave (HSPI, ESP-IDF) — xuất MISO thật khi master clock
// ============================================================================
#if ENABLE_SPI_SLAVE
static spi_slave_transaction_t spiTrans;
static uint8_t spiRxBuf[8];
void spiSlaveSetup() {
  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = PIN_SPI_SLV_MOSI;
  buscfg.miso_io_num = PIN_SPI_A_MISO;      // slave xuất MISO ra CH-7
  buscfg.sclk_io_num = PIN_SPI_SLV_SCK;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;

  spi_slave_interface_config_t slvcfg = {};
  slvcfg.spics_io_num = PIN_SPI_SLV_CS;
  slvcfg.queue_size = 3;
  slvcfg.mode = 0;                          // SPI mode 0
  slvcfg.flags = 0;

  spi_slave_initialize(HSPI_HOST, &buscfg, &slvcfg, SPI_DMA_DISABLED);
}
// Nạp sẵn 1 giao dịch để slave tự đáp khi master clock (non-blocking).
void spiSlaveQueue() {
  memset(&spiTrans, 0, sizeof(spiTrans));
  spiTrans.length = sizeof(spiSlaveResp) * 8;
  spiTrans.tx_buffer = (const void *)spiSlaveResp;
  spiTrans.rx_buffer = spiRxBuf;
  spi_slave_queue_trans(HSPI_HOST, &spiTrans, 0);
}
void spiSlaveCollect() {
  spi_slave_transaction_t *rt;
  spi_slave_get_trans_result(HSPI_HOST, &rt, 0);   // thu kết quả (không chặn)
}
#endif

// ============================================================================
//  Bit-bang UART TX (cho UART-B, GPIO2)
// ============================================================================
void bitbangUartByte(int pin, uint8_t b, uint32_t bitUs) {
  digitalWrite(pin, LOW); delayMicroseconds(bitUs);           // start
  for (int i = 0; i < 8; i++) {                                // LSB first
    digitalWrite(pin, (b >> i) & 1); delayMicroseconds(bitUs);
  }
  digitalWrite(pin, HIGH); delayMicroseconds(bitUs);           // stop
}

// ============================================================================
//  Bit-bang I2C master (cho I2C-B, GPIO4/12) — không slave -> addr + NACK
// ============================================================================
static inline void i2cbSda(int v) { digitalWrite(PIN_I2C_B_SDA, v); }
static inline void i2cbScl(int v) { digitalWrite(PIN_I2C_B_SCL, v); }
void i2cbStart() { i2cbSda(1); i2cbScl(1); delayMicroseconds(3); i2cbSda(0); delayMicroseconds(3); i2cbScl(0); }
void i2cbStop()  { i2cbSda(0); i2cbScl(1); delayMicroseconds(3); i2cbSda(1); delayMicroseconds(3); }
void i2cbByte(uint8_t b) {
  for (int i = 0; i < 8; i++) {
    i2cbSda((b >> (7 - i)) & 1); delayMicroseconds(2);
    i2cbScl(1); delayMicroseconds(3); i2cbScl(0); delayMicroseconds(2);
  }
  // nhịp thứ 9 (ACK): thả SDA cao, đọc (không slave -> NACK)
  i2cbSda(1); delayMicroseconds(2); i2cbScl(1); delayMicroseconds(3); i2cbScl(0);
}
void i2cbWrite(uint8_t addr, uint8_t data) {
  i2cbStart(); i2cbByte((addr << 1) | 0); i2cbByte(data); i2cbStop();
}

// ============================================================================
//  setup
// ============================================================================
void setup() {
  Serial.begin(115200);

  // 1. UART-A (2 chiều): host TX (17) + "thiết bị" TX (21 = CH-2)
  UARTA.begin(115200, SERIAL_8N1, /*rx*/27, /*tx*/PIN_UART_A_TX);
  UARTA_DEV.begin(115200, SERIAL_8N1, /*rx*/-1, /*tx*/PIN_UART_A_RX_DEV);

  // 2. I2C-A: master (Wire) + slave@0x3C (Wire1)  [jumper 22<->32, 5<->33]
  Wire.begin(PIN_I2C_A_SDA, PIN_I2C_A_SCL, 100000);
  Wire1.begin(I2C_A_ADDR, PIN_I2C1_SDA, PIN_I2C1_SCL, 100000);
  Wire1.onReceive(onI2cReceive);
  Wire1.onRequest(onI2cRequest);

  // 3. SPI-A: master (VSPI) + slave (HSPI) xuất MISO  [jumper 18->34,23->35,19->36]
  pinMode(PIN_SPI_A_CS, OUTPUT); digitalWrite(PIN_SPI_A_CS, HIGH);
  vspi.begin(PIN_SPI_A_SCK, /*miso*/-1, PIN_SPI_A_MOSI, /*ss*/-1);
#if ENABLE_SPI_SLAVE
  spiSlaveSetup();
#endif

  // 4. UART-B (bit-bang) + I2C-B (bit-bang)
  pinMode(PIN_UART_B_TX, OUTPUT); digitalWrite(PIN_UART_B_TX, HIGH);
  pinMode(PIN_I2C_B_SDA, OUTPUT); pinMode(PIN_I2C_B_SCL, OUTPUT);
  i2cbSda(1); i2cbScl(1);

  // 5. PWM + CLK (LEDC, ESP32 core 3.x)
  ledcAttach(PIN_PWM_1KHZ, 1000, 8);    ledcWrite(PIN_PWM_1KHZ, 192);   // 75%
  ledcAttach(PIN_PWM_10KHZ, 10000, 8);  ledcWrite(PIN_PWM_10KHZ, 64);   // 25%
  ledcAttach(PIN_CLK_1MHZ, 1000000, 1); ledcWrite(PIN_CLK_1MHZ, 1);
  ledcAttach(PIN_CLK_500KHZ, 500000, 1);ledcWrite(PIN_CLK_500KHZ, 1);
  ledcAttach(PIN_CLK_100KHZ, 100000, 1);ledcWrite(PIN_CLK_100KHZ, 1);
}

// ============================================================================
//  loop — sinh giao dịch tất cả các bus, rải khoảng nghỉ để tách khung
// ============================================================================
uint32_t loopCount = 0;
void loop() {
  // --- UART-A: host phát pattern, "thiết bị" phát chuỗi khác (2 chiều) ---
  for (uint8_t b : UART_A_PATTERN) UARTA.write(b);
  UARTA_DEV.print(UART_B_TEXT);
  UARTA.flush(); UARTA_DEV.flush();

  // --- UART-B (bit-bang) ---
  for (uint8_t b : UART_A_PATTERN) bitbangUartByte(PIN_UART_B_TX, b, 9);  // ~115200

  // --- I2C-A: master ghi + đọc (slave Wire1 trả ACK + data) ---
  Wire.beginTransmission(I2C_A_ADDR);
  Wire.write(0xAA); Wire.write(0x55);
  Wire.endTransmission();
  Wire.requestFrom((int)I2C_A_ADDR, 1);   // đọc -> repeated-START + data từ slave
  while (Wire.available()) Wire.read();

  // --- I2C-B (bit-bang master, addr 0x50) ---
  i2cbWrite(0x50, 0x12);

  // --- SPI-A: master transfer (slave HSPI đáp MISO) ---
#if ENABLE_SPI_SLAVE
  spiSlaveQueue();
#endif
  vspi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_SPI_A_CS, LOW);
  for (uint8_t &b : SPI_MOSI_PATTERN) vspi.transfer(b);
  digitalWrite(PIN_SPI_A_CS, HIGH);
  vspi.endTransaction();
#if ENABLE_SPI_SLAVE
  spiSlaveCollect();
#endif

  if ((loopCount++ % 100) == 0)
    Serial.printf("Gen 16ch... loop=%u\n", loopCount);

  delay(2);   // khoảng nghỉ tách khung (đủ nhanh cho decode ở divisor cao)
}
