#include <SPI.h>
#include <stdio.h>
#include <string.h>
#include <STM32LowPower.h>

// ============================================================
//                    用户配置
// ============================================================

#define NODE_ID                 2
#define LORA_FREQ_HZ            433000000UL

// RTC 唤醒周期：1 分钟
#define HEARTBEAT_INTERVAL_MS   60000UL

// 随机退避
#define TX_RANDOM_MAX_MS        2000UL

// ============================================================
//                    引脚定义
// ============================================================

#define WATER_DETECT_N          PB2
#define ledPin                  PB9

#define LORA_CS                 PA4
#define LORA_SCK                PA5
#define LORA_MISO               PA6
#define LORA_MOSI               PA7
#define LORA_RSTN               PB0
#define LORA_DIO0               PB1

// ============================================================
//                    SX1278 寄存器
// ============================================================

#define REG_FIFO                    0x00
#define REG_OP_MODE                 0x01
#define REG_FRF_MSB                 0x06
#define REG_FRF_MID                 0x07
#define REG_FRF_LSB                 0x08
#define REG_PA_CONFIG               0x09
#define REG_LNA                     0x0C
#define REG_FIFO_ADDR_PTR           0x0D
#define REG_FIFO_TX_BASE_ADDR       0x0E
#define REG_FIFO_RX_BASE_ADDR       0x0F
#define REG_FIFO_RX_CURRENT_ADDR    0x10
#define REG_IRQ_FLAGS               0x12
#define REG_RX_NB_BYTES             0x13
#define REG_PKT_SNR_VALUE           0x19
#define REG_PKT_RSSI_VALUE          0x1A
#define REG_MODEM_CONFIG1           0x1D
#define REG_MODEM_CONFIG2           0x1E
#define REG_PREAMBLE_MSB            0x20
#define REG_PREAMBLE_LSB            0x21
#define REG_PAYLOAD_LENGTH          0x22
#define REG_MODEM_CONFIG3           0x26
#define REG_SYNC_WORD               0x39
#define REG_DIO_MAPPING1            0x40
#define REG_VERSION                 0x42

// ============================================================
//                    SX1278 模式
// ============================================================

#define MODE_SLEEP                  0x00
#define MODE_STDBY                  0x01
#define MODE_TX                     0x03
#define MODE_RXCONTINUOUS           0x05
#define MODE_LORA                   0x80

// ============================================================
//                    SPI 设置
// ============================================================

SPISettings sx1278SPI(8000000, MSBFIRST, SPI_MODE0);

// ============================================================
//                    全局变量
// ============================================================

volatile bool waterWakeFlag = false;

enum WakeReason
{
  WAKE_UNKNOWN = 0,
  WAKE_WATER,
  WAKE_RTC
};

WakeReason wakeReason = WAKE_UNKNOWN;

uint16_t seqNum = 0;

// ============================================================
//                    函数声明
// ============================================================

void SX1278_Init();
void SX1278_SetFrequency(uint32_t freq);
void SX1278_SendPacket(uint8_t* data, uint8_t len);

uint8_t readRegister(uint8_t addr);
void writeRegister(uint8_t addr, uint8_t value);
void setMode(uint8_t mode);

void sendLoRaReport(uint8_t status);

void blinkLED(uint8_t count);

uint16_t readBatteryVoltageMV();

void waterInterruptHandler();

void enterLowPowerForHeartbeat();

// ============================================================
//                    SETUP
// ============================================================

void setup()
{
  Serial.setTx(PA9);
  Serial.setRx(PA10);
  Serial.begin(115200);

  delay(50);

  Serial.println();
  Serial.println("====================================");
  Serial.println(" Water Sensor LoRa Low Power Node");
  Serial.println("   Send Once Version");
  Serial.println("====================================");

  Serial.print("Node ID: ");
  Serial.println(NODE_ID);

  // ----------------------------------------------------------
  // LED 初始化
  // ----------------------------------------------------------

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // ----------------------------------------------------------
  // 水浸输入
  // ----------------------------------------------------------

  pinMode(WATER_DETECT_N, INPUT_PULLUP);

  // ----------------------------------------------------------
  // 随机种子
  // ----------------------------------------------------------

  randomSeed(NODE_ID + analogRead(PA0));

  // ----------------------------------------------------------
  // 低功耗初始化
  // ----------------------------------------------------------

  LowPower.begin();

  LowPower.attachInterruptWakeup(
    WATER_DETECT_N,
    waterInterruptHandler,
    FALLING
  );

  // ----------------------------------------------------------
  // SX1278 初始化
  // ----------------------------------------------------------

  SX1278_Init();

  Serial.println("SX1278 Ready");
  Serial.println("Low power mode ready");
  Serial.println();

  // ----------------------------------------------------------
  // 上电 LED 自检：闪 3 下
  // ----------------------------------------------------------

  Serial.println("Power-on LED test...");
  blinkLED(3);

  // ----------------------------------------------------------
  // 上电后发送一次心跳
  // ----------------------------------------------------------

  Serial.println("Power-on heartbeat...");
  sendLoRaReport(0);

  // 发送完成后确保 LED 熄灭
  digitalWrite(ledPin, HIGH);
}

// ============================================================
//                    LOOP
// ============================================================

void loop()
{
  // ==========================================================
  // 1. 浸水中断唤醒
  // ==========================================================

  if (waterWakeFlag)
  {
    waterWakeFlag = false;

    wakeReason = WAKE_WATER;

    Serial.println();
    Serial.println("!!! WATER INTERRUPT !!!");

    // 浸水报警只发送一次
    sendLoRaReport(1);

    // 报警发送完成后 LED 闪 2 下
    blinkLED(2);

    // 进入低功耗
    enterLowPowerForHeartbeat();

    return;
  }

  // ==========================================================
  // 2. RTC 唤醒 → 心跳
  // ==========================================================

  wakeReason = WAKE_RTC;

  Serial.println();
  Serial.println("RTC wakeup -> heartbeat");

  // 心跳只发送一次
  sendLoRaReport(0);

  // 心跳发送完成后 LED 闪 2 下
  blinkLED(2);

  // 进入低功耗
  enterLowPowerForHeartbeat();
}

// ============================================================
//                    进入低功耗
// ============================================================

void enterLowPowerForHeartbeat()
{
  Serial.flush();

  // SX1278 进入 Sleep
  setMode(MODE_SLEEP);

  // LED 确保关闭
  digitalWrite(ledPin, HIGH);

  // MCU 深度睡眠
  LowPower.deepSleep(HEARTBEAT_INTERVAL_MS);

  // 唤醒后 SX1278 回到 Standby
  setMode(MODE_STDBY);
}

// ============================================================
//                    浸水中断 ISR
// ============================================================

void waterInterruptHandler()
{
  waterWakeFlag = true;
}

// ============================================================
//                    LoRa 数据发送（只发一次）
// ============================================================

void sendLoRaReport(uint8_t status)
{
  // ----------------------------------------------------------
  // 随机退避
  // ----------------------------------------------------------

  uint32_t delayMs = random(0, TX_RANDOM_MAX_MS);

  delayMs += (NODE_ID % 500);

  delay(delayMs);

  // ----------------------------------------------------------
  // 读取电池电压
  // ----------------------------------------------------------

  uint16_t BAT = readBatteryVoltageMV();

  int rssi = 0;

  char buf[48];

  // ----------------------------------------------------------
  // 生成数据
  //
  // HB = Heartbeat
  // AL = Alarm
  // ----------------------------------------------------------

  int len = sprintf(
    buf,
    "%s,%03d,%u,%d,%d.%03d,%d\n",
    status ? "AL" : "HB",
    NODE_ID,
    seqNum,
    rssi,
    BAT / 1000,
    BAT % 1000,
    status
  );

  // ----------------------------------------------------------
  // 数据长度检查
  // ----------------------------------------------------------

  if (len <= 0 || len >= (int)sizeof(buf))
  {
    Serial.println("ERROR: protocol buffer overflow");
    return;
  }

  // ----------------------------------------------------------
  // 串口打印
  // ----------------------------------------------------------

  Serial.println();
  Serial.println("========== LoRa TX (Once) ==========");

  Serial.print("TX TEXT: ");
  Serial.print(buf);

  Serial.print("TX LEN : ");
  Serial.println(len);

  // ----------------------------------------------------------
  // LoRa 发送
  // ----------------------------------------------------------

  setMode(MODE_STDBY);

  SX1278_SendPacket(
    (uint8_t*)buf,
    (uint8_t)len
  );

  Serial.println("===================================");

  // ----------------------------------------------------------
  // 序号递增
  // ----------------------------------------------------------

  seqNum++;
}

// ============================================================
//                    SX1278 初始化
// ============================================================

void SX1278_Init()
{
  // ----------------------------------------------------------
  // GPIO
  // ----------------------------------------------------------

  pinMode(LORA_CS, OUTPUT);
  pinMode(LORA_RSTN, OUTPUT);
  pinMode(LORA_DIO0, INPUT);

  digitalWrite(LORA_CS, HIGH);
  digitalWrite(LORA_RSTN, HIGH);

  // ----------------------------------------------------------
  // 硬件复位
  // ----------------------------------------------------------

  digitalWrite(LORA_RSTN, LOW);

  delay(10);

  digitalWrite(LORA_RSTN, HIGH);

  delay(50);

  // ----------------------------------------------------------
  // SPI 初始化
  // ----------------------------------------------------------

  SPI.setMOSI(LORA_MOSI);
  SPI.setMISO(LORA_MISO);
  SPI.setSCLK(LORA_SCK);

  SPI.begin();

  delay(10);

  // ----------------------------------------------------------
  // 读取芯片版本
  // ----------------------------------------------------------

  uint8_t version = readRegister(REG_VERSION);

  Serial.print("SX1278 VERSION = 0x");

  if (version < 0x10)
  {
    Serial.print("0");
  }

  Serial.println(version, HEX);

  if (version != 0x12)
  {
    Serial.println("ERROR: SX1278 not detected!");
  }
  else
  {
    Serial.println("SX1278 detected");
  }

  // ----------------------------------------------------------
  // LoRa 模式
  // ----------------------------------------------------------

  setMode(MODE_SLEEP);

  delay(10);

  writeRegister(
    REG_OP_MODE,
    MODE_LORA
  );

  delay(10);

  // ----------------------------------------------------------
  // 设置频率 433 MHz
  // ----------------------------------------------------------

  SX1278_SetFrequency(LORA_FREQ_HZ);

  // ----------------------------------------------------------
  // PA 设置
  //
  // 0x8F：
  // PA_SELECT = 1
  // 最大输出功率设置
  //
  // 注意：
  // 如果使用 CR2032，建议后续重新评估发射功率。
  // ----------------------------------------------------------

  writeRegister(
    REG_PA_CONFIG,
    0x8F
  );

  // ----------------------------------------------------------
  // BW = 250 kHz
  // CR = 4/5
  // Explicit Header
  // ----------------------------------------------------------

  writeRegister(
    REG_MODEM_CONFIG1,
    0x82
  );

  // ----------------------------------------------------------
  // SF = 12
  // CRC ON
  // ----------------------------------------------------------

  writeRegister(
    REG_MODEM_CONFIG2,
    0xC4
  );

  // ----------------------------------------------------------
  // LowDataRateOptimize = ON
  // AGC Auto = ON
  // ----------------------------------------------------------

  writeRegister(
    REG_MODEM_CONFIG3,
    0x0C
  );

  // ----------------------------------------------------------
  // Preamble = 8
  // ----------------------------------------------------------

  writeRegister(
    REG_PREAMBLE_MSB,
    0x00
  );

  writeRegister(
    REG_PREAMBLE_LSB,
    0x08
  );

  // ----------------------------------------------------------
  // Sync Word
  // ----------------------------------------------------------

  writeRegister(
    REG_SYNC_WORD,
    0x12
  );

  // ----------------------------------------------------------
  // FIFO
  // ----------------------------------------------------------

  writeRegister(
    REG_FIFO_TX_BASE_ADDR,
    0x00
  );

  writeRegister(
    REG_FIFO_RX_BASE_ADDR,
    0x00
  );

  // ----------------------------------------------------------
  // DIO0 = TxDone
  // ----------------------------------------------------------

  writeRegister(
    REG_DIO_MAPPING1,
    0x40
  );

  // ----------------------------------------------------------
  // Standby
  // ----------------------------------------------------------

  setMode(MODE_STDBY);

  // ----------------------------------------------------------
  // 寄存器确认
  // ----------------------------------------------------------

  Serial.println();
  Serial.println("======== 寄存器确认 ========");

  Serial.print("MODEM1 = 0x");
  Serial.println(
    readRegister(REG_MODEM_CONFIG1),
    HEX
  );

  Serial.print("MODEM2 = 0x");
  Serial.println(
    readRegister(REG_MODEM_CONFIG2),
    HEX
  );

  Serial.print("MODEM3 = 0x");
  Serial.println(
    readRegister(REG_MODEM_CONFIG3),
    HEX
  );

  Serial.print("SYNC   = 0x");
  Serial.println(
    readRegister(REG_SYNC_WORD),
    HEX
  );

  Serial.println("==========================");
}

// ============================================================
//                    设置频率
// ============================================================

void SX1278_SetFrequency(uint32_t freq)
{
  uint64_t frf =
    ((uint64_t)freq << 19) /
    32000000ULL;

  writeRegister(
    REG_FRF_MSB,
    (uint8_t)(frf >> 16)
  );

  writeRegister(
    REG_FRF_MID,
    (uint8_t)(frf >> 8)
  );

  writeRegister(
    REG_FRF_LSB,
    (uint8_t)frf
  );
}

// ============================================================
//                    LoRa 发送
// ============================================================

void SX1278_SendPacket(
  uint8_t* data,
  uint8_t len
)
{
  // ----------------------------------------------------------
  // 长度检查
  // ----------------------------------------------------------

  if (len == 0 || len > 255)
  {
    Serial.println("ERROR: invalid TX length");
    return;
  }

  // ----------------------------------------------------------
  // Standby
  // ----------------------------------------------------------

  setMode(MODE_STDBY);

  // ----------------------------------------------------------
  // FIFO 指针
  // ----------------------------------------------------------

  writeRegister(
    REG_FIFO_ADDR_PTR,
    0x00
  );

  // ----------------------------------------------------------
  // Payload 长度
  // ----------------------------------------------------------

  writeRegister(
    REG_PAYLOAD_LENGTH,
    len
  );

  // ----------------------------------------------------------
  // 写入 FIFO
  // ----------------------------------------------------------

  SPI.beginTransaction(sx1278SPI);

  digitalWrite(
    LORA_CS,
    LOW
  );

  SPI.transfer(
    REG_FIFO | 0x80
  );

  for (uint8_t i = 0; i < len; i++)
  {
    SPI.transfer(data[i]);
  }

  digitalWrite(
    LORA_CS,
    HIGH
  );

  SPI.endTransaction();

  // ----------------------------------------------------------
  // 清除 IRQ
  // ----------------------------------------------------------

  writeRegister(
    REG_IRQ_FLAGS,
    0xFF
  );

  // ----------------------------------------------------------
  // 开始发送
  // ----------------------------------------------------------

  setMode(MODE_TX);

  unsigned long start = millis();

  bool txDone = false;

  // ----------------------------------------------------------
  // 等待 TxDone
  // ----------------------------------------------------------

  while (millis() - start < 3000)
  {
    if (readRegister(REG_IRQ_FLAGS) & 0x08)
    {
      txDone = true;
      break;
    }

    delay(1);
  }

  // ----------------------------------------------------------
  // 发送结果
  // ----------------------------------------------------------

  if (txDone)
  {
    Serial.println("TX DONE");
  }
  else
  {
    Serial.println("TX TIMEOUT!");
  }

  // ----------------------------------------------------------
  // 清除 IRQ
  // ----------------------------------------------------------

  writeRegister(
    REG_IRQ_FLAGS,
    0xFF
  );

  // ----------------------------------------------------------
  // 回 Standby
  // ----------------------------------------------------------

  setMode(MODE_STDBY);
}

// ============================================================
//                    SPI 读寄存器
// ============================================================

uint8_t readRegister(uint8_t addr)
{
  uint8_t value;

  SPI.beginTransaction(
    sx1278SPI
  );

  digitalWrite(
    LORA_CS,
    LOW
  );

  SPI.transfer(
    addr & 0x7F
  );

  value = SPI.transfer(0x00);

  digitalWrite(
    LORA_CS,
    HIGH
  );

  SPI.endTransaction();

  return value;
}

// ============================================================
//                    SPI 写寄存器
// ============================================================

void writeRegister(
  uint8_t addr,
  uint8_t value
)
{
  SPI.beginTransaction(
    sx1278SPI
  );

  digitalWrite(
    LORA_CS,
    LOW
  );

  SPI.transfer(
    addr | 0x80
  );

  SPI.transfer(value);

  digitalWrite(
    LORA_CS,
    HIGH
  );

  SPI.endTransaction();
}

// ============================================================
//                    设置 SX1278 模式
// ============================================================

void setMode(uint8_t mode)
{
  writeRegister(
    REG_OP_MODE,
    MODE_LORA | mode
  );

  delay(2);
}

// ============================================================
//                    电池电压
// ============================================================

uint16_t readBatteryVoltageMV()
{
  const uint32_t VREFINT_CAL_MV = 1212;
  const uint32_t ADC_MAX = 4095;

  analogReadResolution(12);

  uint32_t vref = 0;

  for (int i = 0; i < 8; i++)
  {
    vref += analogRead(AVREF);
  }

  vref /= 8;

  if (vref == 0)
  {
    return 3300;
  }

  uint32_t vdd_mv =
    (VREFINT_CAL_MV * ADC_MAX) /
    vref;

  // 限制异常值
  if (vdd_mv < 1800)
  {
    vdd_mv = 1800;
  }

  if (vdd_mv > 3600)
  {
    vdd_mv = 3600;
  }

  return (uint16_t)vdd_mv;
}

// ============================================================
//                    LED 闪烁
// ============================================================
//
// count = 闪烁次数
//
// 例如：
// blinkLED(3) -> 闪3下
// blinkLED(2) -> 闪2下
//
// 最终 LED 始终为 LOW
//
// ============================================================

void blinkLED(uint8_t count)
{
  for (uint8_t i = 0; i < count; i++)
  {
    // 点亮
    digitalWrite(
      ledPin,
      HIGH
    );

    delay(100);

    // 熄灭
    digitalWrite(
      ledPin,
      LOW
    );

    // 两次闪烁之间间隔
    if (i < count - 1)
    {
      delay(100);
    }
  }

  // 确保最终熄灭
  digitalWrite(
    ledPin,
    LOW
  );
}
