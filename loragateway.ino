#include <SPI.h>

// ============================================================
//                    引脚定义
// ============================================================

#define LORA_CS             PA4
#define LORA_SCK            PA5
#define LORA_MISO           PA6
#define LORA_MOSI           PA7
#define LORA_RSTN           PB0
#define LORA_DIO0           PB1

#define ledPin              PB9

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
//                    模式
// ============================================================

#define MODE_SLEEP                  0x00
#define MODE_STDBY                  0x01
#define MODE_TX                     0x03
#define MODE_RXCONTINUOUS           0x05

#define MODE_LORA                   0x80

// ============================================================
//                    IRQ
// ============================================================

#define IRQ_RX_DONE                 0x40
#define IRQ_PAYLOAD_CRC_ERROR       0x20

// ============================================================
//                    SPI
// ============================================================

SPISettings sx1278SPI(
  8000000,
  MSBFIRST,
  SPI_MODE0
);

// ============================================================
//                    函数声明
// ============================================================

void SX1278_Init();

void SX1278_SetFrequency(
  uint32_t freq
);

uint8_t readRegister(
  uint8_t addr
);

void writeRegister(
  uint8_t addr,
  uint8_t value
);

void setMode(
  uint8_t mode
);

void startReceive();

void printHex(
  uint8_t* buf,
  uint8_t len
);

// ============================================================
//                    SETUP
// ============================================================

void setup()
{
  Serial.setTx(PA9);
  Serial.setRx(PA10);

  Serial.begin(115200);

  while (!Serial)
  {
    ;
  }

  Serial.println();
  Serial.println("====================================");
  Serial.println("  Water Sensor LoRa 主站 (接收端)");
  Serial.println("====================================");

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  SX1278_Init();

  startReceive();

  Serial.println();
  Serial.println("SX1278 Ready - Listening...");
  Serial.println("等待水浸传感器数据...");
  Serial.println();
}

// ============================================================
//                    LOOP
// ============================================================

void loop()
{
  uint8_t flags =
    readRegister(REG_IRQ_FLAGS);

  // --------------------------------------------------------
  // RxDone
  // --------------------------------------------------------

  if (flags & IRQ_RX_DONE)
  {
    Serial.println();
    Serial.println("******** RxDone ********");

    // ----------------------------------------------------
    // 先记录 CRC 状态
    // ----------------------------------------------------

    bool crcError =
      (flags & IRQ_PAYLOAD_CRC_ERROR);

    Serial.print("IRQ FLAGS = 0x");

    if (flags < 0x10)
      Serial.print("0");

    Serial.println(
      flags,
      HEX
    );

    Serial.print("CRC = ");

    if (crcError)
      Serial.println("ERROR");
    else
      Serial.println("OK");

    // ----------------------------------------------------
    // 读取包长度
    // ----------------------------------------------------

    uint8_t len =
      readRegister(REG_RX_NB_BYTES);

    Serial.print("RX_NB_BYTES = ");
    Serial.println(len);

    if (len == 0 || len > 64)
    {
      Serial.println(
        "ERROR: invalid packet length"
      );

      writeRegister(
        REG_IRQ_FLAGS,
        0xFF
      );

      startReceive();

      return;
    }

    // ----------------------------------------------------
    // 读取当前 RX FIFO 地址
    // ----------------------------------------------------

    uint8_t currentAddr =
      readRegister(
        REG_FIFO_RX_CURRENT_ADDR
      );

    Serial.print(
      "FIFO_RX_CURRENT_ADDR = 0x"
    );

    if (currentAddr < 0x10)
      Serial.print("0");

    Serial.println(
      currentAddr,
      HEX
    );

    // ----------------------------------------------------
    // 设置 FIFO 读取地址
    // ----------------------------------------------------

    writeRegister(
      REG_FIFO_ADDR_PTR,
      currentAddr
    );

    // ----------------------------------------------------
    // 读取 FIFO
    // ----------------------------------------------------

    uint8_t buf[65];

    SPI.beginTransaction(
      sx1278SPI
    );

    digitalWrite(
      LORA_CS,
      LOW
    );

    SPI.transfer(
      REG_FIFO & 0x7F
    );

    for (uint8_t i = 0; i < len; i++)
    {
      buf[i] =
        SPI.transfer(0x00);
    }

    digitalWrite(
      LORA_CS,
      HIGH
    );

    SPI.endTransaction();

    // ----------------------------------------------------
    // RSSI
    // ----------------------------------------------------

    uint8_t rssiRaw =
      readRegister(
        REG_PKT_RSSI_VALUE
      );

    int16_t rssi =
      (int16_t)rssiRaw - 164;

    // ----------------------------------------------------
    // SNR
    // ----------------------------------------------------

    int8_t snrRaw =
      (int8_t)readRegister(
        REG_PKT_SNR_VALUE
      );

    float snr =
      snrRaw / 4.0f;

    // ============ ★★★ 新增：解析从站数据，用主站真实 RSSI 替换 ============
    {
      char statusStr[3];
      int nodeId, seq, rssi_slave, bat_int, bat_frac, status_val;

      // 解析从站发来的原始 buf（rssi_slave 是从站填的 0，丢弃不用）
      sscanf((char*)buf, "%2[^,],%d,%d,%d,%d.%d,%d",
             statusStr,      // "HB" 或 "AL"
             &nodeId,        // NODE_ID
             &seq,           // seqNum
             &rssi_slave,    // 从站的 0，不用
             &bat_int,       // 电压整数部分
             &bat_frac,      // 电压小数部分
             &status_val);   // 0=正常, 1=浸水

      // 用主站真实 RSSI 重新组包，写回 buf
      int newLen = sprintf((char*)buf, "%s,%03d,%u,%d,%d.%03d,%d\n",
                           statusStr,
                           nodeId,
                           (unsigned int)seq,
                           rssi,        // ← 主站真实 RSSI（如 -45）
                           bat_int,
                           bat_frac,
                           status_val);

      len = (uint8_t)newLen;  // 更新包长度，后面打印用
    }
    // ============ ★★★ 新增结束 ============


    // ----------------------------------------------------
    // 打印
    // ----------------------------------------------------




    

    Serial.println();

    Serial.print("[收到] 长度=");
    Serial.print(len);

    Serial.print("  RSSI=");
    Serial.print(rssi);
    Serial.print(" dBm");

    Serial.print("  SNR=");
    Serial.print(snr, 2);
    Serial.println(" dB");

    //Serial.print("TEXT: ");

    for (uint8_t i = 0; i < len; i++)
    {
      uint8_t c = buf[i];

      if (c == '\r' || c == '\n')
      {
        continue;
      }

      if (c >= 32 && c <= 126)
      {
        Serial.write(c);
      }
    }

    Serial.println();

    Serial.print("HEX : ");

    printHex(
      buf,
      len
    );

    Serial.println(
      "--------------------------------"
    );

    // ----------------------------------------------------
    // 如果 CRC 错误，明确告诉我们
    // ----------------------------------------------------

    if (crcError)
    {
      Serial.println(
        "!!! CRC ERROR: 此包数据不要用于业务处理 !!!"
      );
    }
    else
    {
      Serial.println(
        ">>> CRC OK: 数据包有效"
      );
    }

    // ----------------------------------------------------
    // LED
    // ----------------------------------------------------

    digitalWrite(
      ledPin,
      HIGH
    );

    delay(40);

    digitalWrite(
      ledPin,
      LOW
    );

    // ----------------------------------------------------
    // 清除所有 IRQ
    // ----------------------------------------------------

    writeRegister(
      REG_IRQ_FLAGS,
      0xFF
    );

    // ----------------------------------------------------
    // 重新进入 RX
    // ----------------------------------------------------

    startReceive();

    Serial.println();
    Serial.println(
      "等待下一包..."
    );
  }
}

// ============================================================
//                    SX1278 初始化
// ============================================================

void SX1278_Init()
{
  pinMode(
    LORA_CS,
    OUTPUT
  );

  pinMode(
    LORA_RSTN,
    OUTPUT
  );

  pinMode(
    LORA_DIO0,
    INPUT
  );

  digitalWrite(
    LORA_CS,
    HIGH
  );

  // --------------------------------------------------------
  // 硬件复位
  // --------------------------------------------------------

  digitalWrite(
    LORA_RSTN,
    LOW
  );

  delay(10);

  digitalWrite(
    LORA_RSTN,
    HIGH
  );

  delay(50);

  // --------------------------------------------------------
  // SPI
  // --------------------------------------------------------

  SPI.setMOSI(
    LORA_MOSI
  );

  SPI.setMISO(
    LORA_MISO
  );

  SPI.setSCLK(
    LORA_SCK
  );

  SPI.begin();

  delay(10);

  // --------------------------------------------------------
  // 芯片检测
  // --------------------------------------------------------

  uint8_t version =
    readRegister(
      REG_VERSION
    );

  Serial.print(
    "SX1278 VERSION = 0x"
  );

  if (version < 0x10)
    Serial.print("0");

  Serial.println(
    version,
    HEX
  );

  if (version != 0x12)
  {
    Serial.println(
      "ERROR: SX1278 not found!"
    );

    while (1)
    {
      digitalWrite(
        ledPin,
        !digitalRead(ledPin)
      );

      delay(300);
    }
  }

  Serial.println(
    "SX1278 detected"
  );

  // --------------------------------------------------------
  // LoRa 模式
  // --------------------------------------------------------

  setMode(
    MODE_SLEEP
  );

  delay(10);

  writeRegister(
    REG_OP_MODE,
    MODE_LORA
  );

  delay(10);

  // --------------------------------------------------------
  // 433 MHz
  // --------------------------------------------------------

  SX1278_SetFrequency(
    433000000UL
  );

  // --------------------------------------------------------
  // PA
  // --------------------------------------------------------

  writeRegister(
    REG_PA_CONFIG,
    0x8F
  );

  // --------------------------------------------------------
  // LoRa 参数
  // --------------------------------------------------------

  // BW = 250 kHz
  // CR = 4/5
  writeRegister(
    REG_MODEM_CONFIG1,
    0x82
  );

  // SF = 12
  // CRC ON
  writeRegister(
    REG_MODEM_CONFIG2,
    0xC4
  );

  // LowDataRateOptimize ON
  // AGC Auto ON
  writeRegister(
    REG_MODEM_CONFIG3,
    0x0C
  );

  // --------------------------------------------------------
  // Preamble = 8
  // --------------------------------------------------------

  writeRegister(
    REG_PREAMBLE_MSB,
    0x00
  );

  writeRegister(
    REG_PREAMBLE_LSB,
    0x08
  );

  // --------------------------------------------------------
  // Sync Word
  // --------------------------------------------------------

  writeRegister(
    REG_SYNC_WORD,
    0x12
  );

  // --------------------------------------------------------
  // FIFO
  // --------------------------------------------------------

  writeRegister(
    REG_FIFO_TX_BASE_ADDR,
    0x00
  );

  writeRegister(
    REG_FIFO_RX_BASE_ADDR,
    0x00
  );

  writeRegister(
    REG_FIFO_ADDR_PTR,
    0x00
  );

  // --------------------------------------------------------
  // DIO0 = RxDone
  // --------------------------------------------------------

  writeRegister(
    REG_DIO_MAPPING1,
    0x00
  );

  // --------------------------------------------------------
  // Standby
  // --------------------------------------------------------

  setMode(
    MODE_STDBY
  );

  // --------------------------------------------------------
  // 打印寄存器
  // --------------------------------------------------------

  Serial.println();
  Serial.println(
    "======== 寄存器确认 ========"
  );

  Serial.print(
    "MODEM1 = 0x"
  );

  Serial.println(
    readRegister(REG_MODEM_CONFIG1),
    HEX
  );

  Serial.print(
    "MODEM2 = 0x"
  );

  Serial.println(
    readRegister(REG_MODEM_CONFIG2),
    HEX
  );

  Serial.print(
    "MODEM3 = 0x"
  );

  Serial.println(
    readRegister(REG_MODEM_CONFIG3),
    HEX
  );

  Serial.print(
    "SYNC   = 0x"
  );

  Serial.println(
    readRegister(REG_SYNC_WORD),
    HEX
  );

  uint16_t preamble =
    ((uint16_t)
     readRegister(REG_PREAMBLE_MSB) << 8)
    |
    readRegister(REG_PREAMBLE_LSB);

  Serial.print(
    "PREAM  = "
  );

  Serial.println(
    preamble
  );

  Serial.print(
    "FREQ   = "
  );

  uint8_t f1 =
    readRegister(REG_FRF_MSB);

  uint8_t f2 =
    readRegister(REG_FRF_MID);

  uint8_t f3 =
    readRegister(REG_FRF_LSB);

  if (f1 < 0x10) Serial.print("0");
  Serial.print(f1, HEX);

  Serial.print(" ");

  if (f2 < 0x10) Serial.print("0");
  Serial.print(f2, HEX);

  Serial.print(" ");

  if (f3 < 0x10) Serial.print("0");
  Serial.println(f3, HEX);

  Serial.println(
    "=========================="
  );
}

// ============================================================
//                    设置频率
// ============================================================

void SX1278_SetFrequency(
  uint32_t freq
)
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
//                    开始连续接收
// ============================================================

void startReceive()
{
  // --------------------------------------------------------
  // Standby
  // --------------------------------------------------------

  setMode(
    MODE_STDBY
  );

  // --------------------------------------------------------
  // FIFO RX 起始地址
  // --------------------------------------------------------

  writeRegister(
    REG_FIFO_RX_BASE_ADDR,
    0x00
  );

  writeRegister(
    REG_FIFO_ADDR_PTR,
    0x00
  );

  // --------------------------------------------------------
  // 清 IRQ
  // --------------------------------------------------------

  writeRegister(
    REG_IRQ_FLAGS,
    0xFF
  );

  // --------------------------------------------------------
  // 连续接收
  // --------------------------------------------------------

  setMode(
    MODE_RXCONTINUOUS
  );
}

// ============================================================
//                    打印 HEX
// ============================================================

void printHex(
  uint8_t* buf,
  uint8_t len
)
{
  for (uint8_t i = 0; i < len; i++)
  {
    if (buf[i] < 0x10)
      Serial.print("0");

    Serial.print(
      buf[i],
      HEX
    );

    Serial.print(" ");
  }

  Serial.println();
}

// ============================================================
//                    SPI 读寄存器
// ============================================================

uint8_t readRegister(
  uint8_t addr
)
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

  value =
    SPI.transfer(0x00);

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

  SPI.transfer(
    value
  );

  digitalWrite(
    LORA_CS,
    HIGH
  );

  SPI.endTransaction();
}

// ============================================================
//                    模式
// ============================================================

void setMode(
  uint8_t mode
)
{
  writeRegister(
    REG_OP_MODE,
    MODE_LORA | mode
  );

  delay(2);
}
