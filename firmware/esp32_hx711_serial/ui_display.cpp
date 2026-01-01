#include "ui_display.h"

#include <SPI.h>
#include <U8g2lib.h>
#include <WiFi.h>

#include "battery_monitor.h"

// -----------------------------------------------------------------------------
// PIN OLED (come firmware originale)
//   CLK  = GPIO18  (pin D18, SPI HW)
//   MOSI = GPIO23  (pin D23, SPI HW)
//   CS   = GPIO25
//   DC   = GPIO26
//   RST  = GPIO27
// -----------------------------------------------------------------------------

static const int OLED_CS  = 25;
static const int OLED_DC  = 26;
static const int OLED_RST = 27;

// U8g2: SSD1322 256x64, 4-wire SPI hardware, full buffer
// Assicurarsi che in u8g2 sia attivo U8G2_16BIT per 256x64.
static U8G2_SSD1322_NHD_256X64_F_4W_HW_SPI oled(
  U8G2_R2,
  /* cs=*/ OLED_CS,
  /* dc=*/ OLED_DC,
  /* reset=*/ OLED_RST
);

// Variabile globale esterna per capire il preset di lavoro (WORK vs LIVE)
// definita nel .ino principale
extern bool stEnable;

// -----------------------------------------------------------------------------
// LOGO MINÙ (bitmap 1-bit, 56x56)
// -----------------------------------------------------------------------------

#define MINU_LOGO_WIDTH  56
#define MINU_LOGO_HEIGHT 56

static const unsigned char MINU_LOGO_bits[] U8X8_PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0xE0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xF3,
  0x03, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x03, 0x0E, 0x00, 0x00, 0x00, 0x00,
  0xFE, 0x01, 0x30, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x01, 0xC0, 0x00, 0x00,
  0x00, 0x00, 0xF8, 0x01, 0x80, 0x01, 0x00, 0x00, 0xC0, 0x80, 0x01, 0x00,
  0x02, 0x00, 0x00, 0x20, 0x00, 0x01, 0x00, 0x04, 0x00, 0x00, 0x30, 0x00,
  0x02, 0x00, 0x08, 0x00, 0x00, 0x18, 0x00, 0x04, 0x00, 0x18, 0x00, 0x00,
  0x08, 0x00, 0x08, 0x02, 0x10, 0x00, 0x00, 0x04, 0x00, 0x88, 0x07, 0x20,
  0x00, 0x00, 0x04, 0x00, 0xD0, 0x04, 0x20, 0x00, 0x00, 0x02, 0xC6, 0x61,
  0x04, 0x40, 0x00, 0x00, 0x02, 0x67, 0x21, 0x06, 0x40, 0x00, 0x00, 0xC0,
  0x24, 0x11, 0x02, 0x00, 0x00, 0x00, 0x38, 0x16, 0x09, 0x03, 0x00, 0x00,
  0xFC, 0x07, 0x92, 0x05, 0x01, 0x00, 0x00, 0x00, 0x00, 0x8A, 0x84, 0x05,
  0x18, 0x00, 0x00, 0x00, 0xC7, 0x82, 0x04, 0x20, 0x00, 0x00, 0x00, 0xC5,
  0xC1, 0x72, 0x02, 0x00, 0x00, 0x01, 0xC3, 0x41, 0x72, 0x0A, 0x1E, 0x00,
  0x81, 0xE3, 0x40, 0x59, 0x8A, 0x21, 0x00, 0x81, 0x61, 0x20, 0x5D, 0x6F,
  0x20, 0x00, 0x81, 0x61, 0x20, 0xCB, 0x1E, 0x00, 0x00, 0x83, 0x00, 0x30,
  0x60, 0x08, 0x00, 0x00, 0x02, 0x00, 0x10, 0x08, 0x00, 0x00, 0x00, 0x02,
  0x00, 0x10, 0x08, 0x00, 0x00, 0x00, 0x06, 0x00, 0x10, 0x10, 0x00, 0x00,
  0x00, 0x04, 0x00, 0x10, 0x10, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x30, 0x30,
  0x01, 0x00, 0x00, 0x08, 0x00, 0x20, 0x18, 0x02, 0x00, 0x00, 0x18, 0x00,
  0x00, 0x38, 0x02, 0x00, 0x00, 0x30, 0x00, 0x00, 0xF8, 0x04, 0x00, 0x00,
  0x60, 0x00, 0x00, 0xF8, 0x05, 0x00, 0x00, 0xC0, 0x00, 0x00, 0xF0, 0x07,
  0x00, 0x00, 0x80, 0x01, 0x00, 0xE0, 0x07, 0x00, 0x00, 0x00, 0x07, 0x00,
  0xE0, 0x07, 0x00, 0x00, 0x00, 0x1C, 0x00, 0xC0, 0x07, 0x00, 0x00, 0x00,
  0xF0, 0x00, 0x02, 0x07, 0x00, 0x00, 0x00, 0x80, 0xFF, 0x01, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


// -----------------------------------------------------------------------------
// HELPER TESTO
// -----------------------------------------------------------------------------
static void drawCenteredText(const char *str, int16_t y) {
  int16_t w = oled.getStrWidth(str);
  int16_t x = (256 - w) / 2;
  oled.drawStr(x, y, str);
}

static void drawCenteredTextUTF8(const char *str, int16_t y) {
  int16_t w = oled.getUTF8Width(str);
  int16_t x = (256 - w) / 2;
  oled.drawUTF8(x, y, str);
}

// -----------------------------------------------------------------------------
// ICONA BATTERIA
// -----------------------------------------------------------------------------
static void drawBatteryIcon(int x, int y, uint8_t level, bool charging) {
  const int w = 19;   // larghezza corpo batteria
  const int h = 8;

  // Stato "vuota" aggiuntivo: 0 tacche e icona lampeggiante
  if (!charging && level == 0) {
    bool visible = ((millis() / 400) % 2) == 0;
    if (!visible) {
      return; // niente icona in questa fase
    }
  }

  // cornice batteria
  oled.drawFrame(x, y, w, h);
  // terminale a destra
  oled.drawBox(x + w, y + 2, 2, h - 4);

  // livello base 0..4
  uint8_t bars = (level > 4) ? 4 : level;

  // animazione semplice in carica: step che cicla 0..4
  if (charging) {
    uint32_t t = millis();
    uint8_t phase = (t / 300) % 5;   // ogni 300 ms cambia
    if (phase == 0) {
      bars = 0;                      // lampeggio completo vuoto
    } else {
      bars = phase;                  // 1..4 tacche
    }
  }

  // barre interne (4 step)
  int barWidth   = 3;
  int barHeight  = h - 4; // dentro il frame
  int spacing    = 1;

  for (int i = 0; i < 4; i++) {
    if (i < bars) {
      int bx = x + 2 + i * (barWidth + spacing);
      int by = y + 2;
      oled.drawBox(bx, by, barWidth, barHeight);
    }
  }
}

// -----------------------------------------------------------------------------
// ICONA RETE (placeholder, per ora non collegata a logica reale)
// -----------------------------------------------------------------------------
static void drawNetIcon(int x, int y, bool connected) {
  int baseY = y + 6;  // "suolo" delle tacche

  // 3 barrette crescenti
  oled.drawBox(x,     baseY - 2, 2, 2); // alta 2
  oled.drawBox(x + 3, baseY - 4, 2, 4); // alta 4
  oled.drawBox(x + 6, baseY - 6, 2, 6); // alta 6

  if (!connected) {
    // slash di disconnessione
    oled.drawLine(x,     y,
                  x + 7, y + 6);
  }
}

// -----------------------------------------------------------------------------
// ICONA TARGET (mirino)
// -----------------------------------------------------------------------------
static void drawTargetIcon(int x, int y) {
  int cx = x + 5;
  int cy = y + 5;

  // cerchio principale
  oled.drawCircle(cx, cy, 4);
  // puntino centrale
  oled.drawDisc(cx, cy, 1);

  // croce che sporge oltre il cerchio
  oled.drawVLine(cx, y,     11);  // verticale
  oled.drawHLine(x,  cy,    11);  // orizzontale
}

// -----------------------------------------------------------------------------
// FORMATTAZIONE GRAMMI (es. 12250 -> "12.250")
// -----------------------------------------------------------------------------
void formatGramsWithDot(long gDisp, char* buf, size_t len) {
  long value = gDisp;
  bool neg = false;
  if (value < 0) {
    neg = true;
    value = -value;
  }

  char tmp[16];
  int pos = 0;
  int digits = 0;

  if (value == 0) {
    tmp[pos++] = '0';
  } else {
    while (value > 0 and pos < (int)sizeof(tmp) - 1) {
      if (digits > 0 and (digits % 3) == 0) {
        tmp[pos++] = '.';
      }
      int digit = value % 10;
      tmp[pos++] = '0' + digit;
      value /= 10;
      digits++;
    }
  }

  if (neg and pos < (int)sizeof(tmp) - 1) {
    tmp[pos++] = '-';
  }
  tmp[pos] = '\0';

  int outLen = (pos < (int)len - 1) ? pos : (int)len - 1;
  for (int i = 0; i < outLen; ++i) {
    buf[i] = tmp[pos - 1 - i];
  }
  buf[outLen] = '\0';
}

// -----------------------------------------------------------------------------
// HELPER: MODE/STATO/BATTERIA
// -----------------------------------------------------------------------------

// Mode: preset di lavoro (work / live) basato su stEnable
static const char* uiGetModeLabel() {
  if (stEnable) {
    // Preset "work/normal": MA_DEFAULT, ST on
    return "WORK";
  } else {
    // Preset "fine/live": MA_FINE, ST off
    return "LIVE";
  }
}

// StateLabel viene passato già come "STABLE", "UNSTABLE" o "LIVE"
static const char* uiGetStateLabel(const char* stateLabel) {
  return stateLabel;
}

// Batteria: mappa i 4 livelli logici su 0..4 tacche
static uint8_t uiGetBatteryLevel4Step() {
  BatteryStatus st = battery_get_status();
  switch (st.level) {
    case BATT_LEVEL_FULL:     return 4;
    case BATT_LEVEL_GOOD:     return 3;
    case BATT_LEVEL_LOW:      return 2;
    case BATT_LEVEL_CRITICAL: return 1;
    case BATT_LEVEL_EMPTY:    return 0;
    default:                  return 1;
  }
}

static bool uiIsBatteryCharging() {
  BatteryStatus st = battery_get_status();
  return st.charging;
}

// -----------------------------------------------------------------------------
// INIZIALIZZAZIONE UI
// -----------------------------------------------------------------------------
void ui_init() {
  // SPI per OLED
  SPI.begin(18, -1, 23, OLED_CS); // SCK=18, MISO unused, MOSI=23, SS=OLED_CS
  oled.begin();  
}

// -----------------------------------------------------------------------------
// LAYOUT BOOT (logo Minù + "Ronin 00")
// -----------------------------------------------------------------------------
void ui_showBoot(const char* line1, const char* line2) {
  oled.clearBuffer();

  // ----- Gruppo logo + scritta "Ronin 00" -----
  const char* roninText = "Ronin 00";

  // Font più piccolo e centrato rispetto al logo
  oled.setFont(u8g2_font_logisoso24_tf);
  int16_t textW = oled.getStrWidth(roninText);

  const int16_t GAP      = 8;   // spazio tra logo e testo
  const int16_t centerY  = 26;  // centro verticale del gruppo

  int16_t groupW = MINU_LOGO_WIDTH + GAP + textW;
  int16_t groupX = (256 - groupW) / 2;  // centro il gruppo orizzontalmente

  // Logo Minù 56x56, centrato su centerY
  int16_t logoX = groupX;
  int16_t logoY = centerY - (MINU_LOGO_HEIGHT / 2);
  oled.drawXBMP(logoX, logoY, MINU_LOGO_WIDTH, MINU_LOGO_HEIGHT, MINU_LOGO_bits);

  // Scritta "Ronin 00" centrata verticalmente rispetto al logo
  const int16_t TEXT_H = 24;  // altezza nominale del font logisoso24
  int16_t textX = logoX + MINU_LOGO_WIDTH + GAP;
  int16_t textBaselineY = centerY + (TEXT_H / 2) - 1;
  oled.drawStr(textX, textBaselineY, roninText);

  // ----- Stato boot in basso (2 righe) -----
  if (!line1) line1 = "";
  if (!line2) line2 = "";

  oled.setFont(u8g2_font_6x12_tr);
  drawCenteredTextUTF8(line1, 52);
  drawCenteredTextUTF8(line2, 62);

  oled.sendBuffer();
 
}

// -----------------------------------------------------------------------------
// SCHERMATA ERRORE (bloccante durante boot)
// -----------------------------------------------------------------------------
void ui_showError(const char* title, const char* detail, const char* hint) {
  if (!title) title = "";
  if (!detail) detail = "";
  if (!hint) hint = "";

  oled.clearBuffer();
  oled.setFont(u8g2_font_6x12_tr);

  drawCenteredTextUTF8("ERRORE", 12);
  drawCenteredTextUTF8(title, 28);
  drawCenteredTextUTF8(detail, 44);
  drawCenteredTextUTF8(hint, 62);

  oled.sendBuffer();
}

// -----------------------------------------------------------------------------
// AVVISO BATTERIA SCARICA + SPEGNIMENTO
// -----------------------------------------------------------------------------
void ui_showBatteryShutdown(float voltage_V) {
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x12_tr);

  // Titolo
  drawCenteredTextUTF8("BATTERIA SCARICA", 14);

  // Riga tensione
  char vline[24];
  snprintf(vline, sizeof(vline), "V=%.2f", voltage_V);
  drawCenteredTextUTF8(vline, 32);

  // Messaggio
  drawCenteredTextUTF8("COLLEGA ALIMENTATORE", 52);

  oled.sendBuffer();
}

void ui_powerSave(bool enable) {
  // U8G2: 1 = power save ON (display off), 0 = ON
  oled.setPowerSave(enable ? 1 : 0);
}

// -----------------------------------------------------------------------------
// LAYOUT PRINCIPALE PESO
// -----------------------------------------------------------------------------
void ui_renderWeight(long gDisp, const char* stateLabel) {
  oled.clearBuffer();
  const int16_t BASELINE_Y = 56;   // baseline comune per le "g"

  // ------- Valori dinamici per la UI -------
  const char* modeValue  = uiGetModeLabel();   // es: WORK / LIVE
  const char* stateValue = stateLabel;         // STABLE / UNSTABLE / LIVE

  // ------- Riga alta: batteria + wifi -------
  uint8_t batteryLevel = uiGetBatteryLevel4Step();  // 0..4
  bool    isCharging   = uiIsBatteryCharging();
  bool    netConnected = WiFi.isConnected();

  drawBatteryIcon(2, 2, batteryLevel, isCharging);
  drawNetIcon(30, 3, netConnected);

  // ------- Info Mode / State (allineate) -------
  oled.setFont(u8g2_font_6x12_tr);

  const int16_t LABEL_X = 2;

  int16_t wModeLabel  = oled.getStrWidth("Mode:");
  int16_t wStateLabel = oled.getStrWidth("State:");
  int16_t labelMaxW   = (wModeLabel > wStateLabel) ? wModeLabel : wStateLabel;
  const int16_t VALUE_X = LABEL_X + labelMaxW + 8; // 8 px dopo i due punti

  // Riga 1: Mode
  oled.drawStr(LABEL_X, 24, "Mode:");
  oled.drawStr(VALUE_X, 24, modeValue);

  // Riga 2: State
  oled.drawStr(LABEL_X, 36, "State:");
  oled.drawStr(VALUE_X, 36, stateValue);

  // ------- Icona target (mirino) -------
  drawTargetIcon(2, BASELINE_Y - 11);

  // ------- Target: placeholder fisso 10.500g -------
  const char *targetStr = "10.500";

  oled.setFont(u8g2_font_logisoso16_tn);
  int16_t targetX     = 2 + 11 + 3;              // mirino (11) + 3 px
  int16_t targetWidth = oled.getStrWidth(targetStr);
  oled.drawStr(targetX, BASELINE_Y, targetStr);

  oled.setFont(u8g2_font_6x12_tr);
  const int16_t TARGET_G_MARGIN = 0;
  int16_t gTargetX             = targetX + targetWidth + TARGET_G_MARGIN;
  oled.drawStr(gTargetX, BASELINE_Y, "g");

  // ------- Peso principale: gDisp formattato come 12.250 -------
  char weightStr[16];
  formatGramsWithDot(gDisp, weightStr, sizeof(weightStr));

  oled.setFont(u8g2_font_logisoso46_tn);
  int16_t weightWidth = oled.getStrWidth(weightStr);

  oled.setFont(u8g2_font_6x12_tr);
  int16_t gWidth = oled.getStrWidth("g");

  const int16_t RIGHT_MARGIN = 2;   // distanza dal bordo destro
  const int16_t MAIN_OVERLAP = 0;   // versione “senza infilare” nella g

  // Posizione della g principale fissata sul bordo destro
  int16_t gMainX = 256 - gWidth - RIGHT_MARGIN;
  oled.drawStr(gMainX, BASELINE_Y, "g");

  // Numero grande agganciato alla g
  oled.setFont(u8g2_font_logisoso46_tn);
  int16_t weightX = gMainX - weightWidth + MAIN_OVERLAP;
  oled.drawStr(weightX, BASELINE_Y, weightStr);

  oled.sendBuffer();
}


void ui_renderTareProgress(uint8_t progressPct) {
  if (progressPct > 100) progressPct = 100;

  oled.clearBuffer();

  // ------- Riga alta: batteria + wifi -------
  uint8_t batteryLevel = uiGetBatteryLevel4Step();  // 0..4
  bool    isCharging   = uiIsBatteryCharging();
  bool    netConnected = WiFi.isConnected();

  drawBatteryIcon(2, 2, batteryLevel, isCharging);
  drawNetIcon(30, 3, netConnected);

  

  // ------- Colonna sinistra: Mode / State (manteniamo la UI come prima) -------
  const char* modeValue  = uiGetModeLabel();   // es: WORK / LIVE
  const char* stateValue = "TARE";            // durante barra tara

  const int16_t LABEL_X = 2;
  const int16_t VALUE_X = 44;

  oled.setFont(u8g2_font_6x12_tr);

  // Riga 1: Mode
  oled.drawStr(LABEL_X, 24, "Mode:");
  oled.drawStr(VALUE_X, 24, modeValue);

  // Riga 2: State
  oled.drawStr(LABEL_X, 36, "State:");
  oled.drawStr(VALUE_X, 36, stateValue);

// ------- Barra di stabilizzazione (10px più corta, allineata a destra) -------
  const int16_t BAR_RIGHT = 250;
  const int16_t BAR_W = 170;     // prima 180
  const int16_t BAR_H = 8;
  const int16_t BAR_X = BAR_RIGHT - BAR_W;
  const int16_t BAR_Y = 54;

  // ------- Testo "- TARA -" centrato orizzontalmente sopra la barra -------
  oled.setFont(u8g2_font_logisoso24_tf);
  const char* txt = "- TARA -";
  int16_t wTxt = oled.getStrWidth(txt);
  int16_t xTxt = BAR_X + (BAR_W - wTxt) / 2;
  oled.drawStr(xTxt, 40, txt);

  oled.drawFrame(BAR_X, BAR_Y, BAR_W, BAR_H);
  int16_t fillW = (int16_t)((BAR_W - 2) * (int32_t)progressPct / 100);
  if (fillW < 0) fillW = 0;
  if (fillW > (BAR_W - 2)) fillW = BAR_W - 2;
  if (fillW > 0) oled.drawBox(BAR_X + 1, BAR_Y + 1, fillW, BAR_H - 2);

  oled.sendBuffer();
}
