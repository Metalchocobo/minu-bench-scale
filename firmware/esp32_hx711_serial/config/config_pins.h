#pragma once

// =============================================================================
// CONFIG_PINS.H - Definizioni pin hardware per Minù Bench Scale
// =============================================================================

// ========================= HX711 =========================
static const int HX711_SCK_PIN  = 16;   // GPIO16 (RX2)
static const int HX711_DOUT_PIN = 35;   // GPIO35 (input-only)

// ========================= I2C (INA219) =========================
static const int I2C_SDA_PIN = 32;
static const int I2C_SCL_PIN = 33;

// ========================= OLED SSD1322 (SPI) =========================
static const int OLED_SCK_PIN  = 18;    // VSPI SCK
static const int OLED_MOSI_PIN = 23;    // VSPI MOSI
static const int OLED_CS_PIN   = 25;
static const int OLED_DC_PIN   = 26;
static const int OLED_RST_PIN  = 27;

// ========================= TASTIERA 4x2 =========================
static const int KEYPAD_ROW_PINS[4] = { 17, 5, 13, 14 };  // R1-R4
static const int KEYPAD_COL_PINS[2] = { 19, 21 };         // C1-C2

// ========================= BUZZER =========================
static const int BUZZER_PIN = 22;       // LEDC PWM

// ========================= DFPLAYER =========================
static const int DFPLAYER_TX_PIN   = 4;     // ESP32 TX -> DFPlayer RX
static const int DFPLAYER_RX_PIN   = 34;    // ESP32 RX <- DFPlayer TX (opzionale)
static const int DFPLAYER_EN_PIN   = 2;     // Power-gate (HIGH = ON)
static const int DFPLAYER_BUSY_PIN = 39;    // BUSY (no pull interno su GPIO39)

// ========================= LED SLEEP =========================
static const int SLEEP_LED_PIN = 15;
static const bool SLEEP_LED_ACTIVE_HIGH = true;  // HIGH = LED acceso
