#include "ui_display.h"

#include <SPI.h>
#include <U8g2lib.h>
#include <WiFi.h>

#include "battery_monitor.h"
#include "net_ota_cloud.h"

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

static void drawCenteredTextInArea(const char* str, int16_t areaX, int16_t areaW, int16_t y) {
  int16_t w = oled.getStrWidth(str);
  int16_t x = areaX + (areaW - w) / 2;
  if (x < areaX) x = areaX;
  oled.drawStr(x, y, str);
}

static void drawCenteredTextUTF8(const char *str, int16_t y) {
  int16_t w = oled.getUTF8Width(str);
  int16_t x = (256 - w) / 2;
  oled.drawUTF8(x, y, str);
}

// -----------------------------------------------------------------------------
// HX WARN ICON (triangolino) - in alto a destra
// -----------------------------------------------------------------------------
static bool g_hxWarnIcon = false;

void ui_setHxWarn(bool enable) {
  g_hxWarnIcon = enable;
}

static void drawWarnTriangleIconTopRight() {
  if (!g_hxWarnIcon) return;

  // Badge HX WARN in alto a destra: testo piccolo con simbolo ASCII.
  // Richiesta: togliere il triangolino grafico; usare un triangolo ASCII nel testo.
  // Nota: ASCII è sempre disponibile; usiamo "/!\\" (fallback implicito: si può sostituire con "!" se preferito).

  oled.setFont(u8g2_font_4x6_tr);

  const char* label = "Errore Cella /!\\";
  const int labelW = oled.getStrWidth(label);

  const int margin = 2;
  int labelX = 256 - margin - labelW;
  if (labelX < 0) labelX = 0;

  const int labelY = 8; // baseline: dentro la barra alta
  oled.drawStr(labelX, labelY, label);
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
// ICONA MQTT (frecce ↑↓ bidirezionali)
// Area: 9px largo x 7px alto (due frecce affiancate)
// -----------------------------------------------------------------------------
static void drawMqttIcon(int x, int y, bool connected, bool visible) {
  if (!visible) return;  // WiFi non connesso: icona nascosta

  // Se MQTT disconnesso (ma WiFi up): lampeggia
  if (!connected) {
    bool blink = ((millis() / 500) % 2) == 0;
    if (!blink) return;
  }

  // Freccia su (↑) — colonna sinistra
  // Punta triangolare in alto, asta verso il basso
  oled.drawPixel(x + 1, y);              // apice
  oled.drawPixel(x,     y + 1);           // ala sinistra
  oled.drawPixel(x + 2, y + 1);           // ala destra
  oled.drawVLine(x + 1, y + 2, 5);       // asta (5px verso il basso)

  // Freccia giù (↓) — colonna destra, distanziata di 5px
  // Asta dall'alto, punta triangolare in basso
  oled.drawVLine(x + 6, y, 5);           // asta (5px dall'alto)
  oled.drawPixel(x + 5, y + 5);           // ala sinistra
  oled.drawPixel(x + 7, y + 5);           // ala destra
  oled.drawPixel(x + 6, y + 6);          // apice
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
// SCHERMATA STANDBY: Zzz... (transizione prima del light-sleep)
// -----------------------------------------------------------------------------
void ui_renderSleepZzz() {
  oled.clearBuffer();

  // Disegno: 3 "Z" su diagonale, tutte centrate nel complesso.
  // Richiesta: prima Z più grande (in alto), ultima Z più piccola (in basso), senza puntini.

  // Font diversi per dare la gerarchia visiva.
  // (Questi font sono parte della libreria U8G2 standard.)
  oled.setFont(u8g2_font_10x20_tr);
  int16_t wZ1 = oled.getStrWidth("Z");

  oled.setFont(u8g2_font_6x12_tr);
  int16_t wZ2 = oled.getStrWidth("Z");

  oled.setFont(u8g2_font_5x8_tr);
  int16_t wZ3 = oled.getStrWidth("Z");

  const int16_t spacing = 10;
  const int16_t totalW = wZ1 + wZ2 + wZ3 + spacing * 2;
  const int16_t x0 = (256 - totalW) / 2;

  // Baseline: in alto -> in basso
  const int16_t y1 = 24;
  const int16_t y2 = 40;
  const int16_t y3 = 54;

  // Z1 (grande)
  oled.setFont(u8g2_font_10x20_tr);
  oled.drawStr(x0, y1, "Z");

  // Z2 (media)
  oled.setFont(u8g2_font_6x12_tr);
  oled.drawStr(x0 + wZ1 + spacing, y2, "Z");

  // Z3 (piccola)
  oled.setFont(u8g2_font_5x8_tr);
  oled.drawStr(x0 + wZ1 + spacing + wZ2 + spacing, y3, "Z");

  oled.sendBuffer();
}

void ui_renderAudioStatus(bool enabled, bool ready, bool logSaved) {
  oled.clearBuffer();

  oled.setFont(u8g2_font_logisoso24_tf);
  drawCenteredText(enabled ? "AUDIO ON" : "AUDIO OFF", 30);

  oled.setFont(u8g2_font_6x12_tr);
  if (!enabled) {
    drawCenteredText("DFPLAYER SPENTO", 48);
  } else if (ready) {
    drawCenteredText("DFPLAYER PRONTO", 48);
  } else {
    drawCenteredText("RIAVVIO MODULO...", 48);
  }

  oled.setFont(u8g2_font_5x8_tr);
  drawCenteredText(logSaved ? "LOG DIAGNOSTICO SALVATO" : "NESSUN LOG SALVATO", 62);
  oled.sendBuffer();
}

// -----------------------------------------------------------------------------
// LAYOUT PRINCIPALE PESO
// -----------------------------------------------------------------------------
static void formatLiveHalfGrams(long gDispX2, char* buf, size_t len) {
  long value = gDispX2;
  bool neg = false;
  if (value < 0) {
    neg = true;
    value = -value;
  }

  long whole = value / 2;
  int decimal = (value % 2) ? 5 : 0;
  if (neg) {
    snprintf(buf, len, "-%ld.%d", whole, decimal);
  } else {
    snprintf(buf, len, "%ld.%d", whole, decimal);
  }
}

static void renderWeightString(const char* weightStr, const char* stateLabel,
                               bool showTarget, float targetWeight,
                               const char* ingredientName,
                               bool mqttConnected, bool mqttVisible) {
  oled.clearBuffer();
  const int16_t BASELINE_Y = 56;   // baseline comune per le "g"

  // ------- Valori dinamici per la UI -------
  const char* modeValue  = uiGetModeLabel();   // es: WORK / LIVE
  const char* stateValue = stateLabel;         // STABLE / UNSTABLE / LIVE

  // ------- Riga alta: batteria + wifi + mqtt -------
  uint8_t batteryLevel = uiGetBatteryLevel4Step();  // 0..4
  bool    isCharging   = uiIsBatteryCharging();
  bool    netConnected = WiFi.isConnected();

  drawBatteryIcon(2, 2, batteryLevel, isCharging);
  drawNetIcon(30, 3, netConnected);
  drawMqttIcon(44, 3, mqttConnected, mqttVisible);
  drawWarnTriangleIconTopRight();

  // ------- Info Mode + State (stessa riga) + Ingrediente -------
  oled.setFont(u8g2_font_6x12_tr);

  const int16_t LABEL_X = 2;

  char infoLine[24];
  if (strcmp(modeValue, "LIVE") == 0) {
    // In LIVE non mostrare stato STABLE/UNSTABLE
    strlcpy(infoLine, "LIVE", sizeof(infoLine));
  } else {
    snprintf(infoLine, sizeof(infoLine), "%s | %s", modeValue, stateValue);
  }
  oled.drawStr(LABEL_X, 24, infoLine);

  // Ingrediente (se presente): max 15 caratteri, oltre -> 15 + puntini ravvicinati
  if (ingredientName && ingredientName[0] != '\0') {
    bool truncated = false;
    char ingLine[24] = {0};
    size_t n = strlen(ingredientName);
    if (n > 15) {
      memcpy(ingLine, ingredientName, 15);
      ingLine[15] = '\0';
      truncated = true;
    } else {
      strlcpy(ingLine, ingredientName, sizeof(ingLine));
    }
    oled.drawStr(LABEL_X, 36, ingLine);

    if (truncated) {
      // Puntini disegnati a pixel (piu' ravvicinati del font standard)
      int16_t textW = oled.getStrWidth(ingLine);
      int16_t dotX = LABEL_X + textW + 1;
      int16_t dotY = 34;
      oled.drawPixel(dotX,     dotY);
      oled.drawPixel(dotX + 2, dotY);
      oled.drawPixel(dotX + 4, dotY);
    }
  }

  // ------- Icona target (mirino) + peso target da MQTT -------
  if (showTarget && targetWeight > 0.0f) {
    drawTargetIcon(2, BASELINE_Y - 11);

    // Formatta il target weight come intero in grammi (es. 450 → "450")
    char targetStr[16];
    formatGramsWithDot((long)targetWeight, targetStr, sizeof(targetStr));

    oled.setFont(u8g2_font_logisoso16_tn);
    int16_t targetX     = 2 + 11 + 3;              // mirino (11) + 3 px
    int16_t targetWidth = oled.getStrWidth(targetStr);
    oled.drawStr(targetX, BASELINE_Y, targetStr);

    oled.setFont(u8g2_font_6x12_tr);
    const int16_t TARGET_G_MARGIN = 0;
    int16_t gTargetX             = targetX + targetWidth + TARGET_G_MARGIN;
    oled.drawStr(gTargetX, BASELINE_Y, "g");
  }

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

void ui_renderWeight(long gDisp, const char* stateLabel,
                     bool showTarget, float targetWeight,
                     const char* ingredientName,
                     bool mqttConnected, bool mqttVisible) {
  char weightStr[16];
  formatGramsWithDot(gDisp, weightStr, sizeof(weightStr));
  renderWeightString(weightStr, stateLabel, showTarget, targetWeight,
                     ingredientName, mqttConnected, mqttVisible);
}

void ui_renderWeightLiveHalf(long gDispX2, const char* stateLabel,
                             bool showTarget, float targetWeight,
                             const char* ingredientName,
                             bool mqttConnected, bool mqttVisible) {
  char weightStr[16];
  formatLiveHalfGrams(gDispX2, weightStr, sizeof(weightStr));
  renderWeightString(weightStr, stateLabel, showTarget, targetWeight,
                     ingredientName, mqttConnected, mqttVisible);
}


void ui_renderTareStatus(ScaleState::TareUiState state, bool autoMode) {
  oled.clearBuffer();

  // ------- Riga alta: batteria + wifi -------
  uint8_t batteryLevel = uiGetBatteryLevel4Step();  // 0..4
  bool    isCharging   = uiIsBatteryCharging();
  bool    netConnected = WiFi.isConnected();

  drawBatteryIcon(2, 2, batteryLevel, isCharging);
  drawNetIcon(30, 3, netConnected);
#if ENABLE_MQTT
  drawMqttIcon(44, 3, Net::isMqttConnected(), netConnected);
#endif
  drawWarnTriangleIconTopRight();

  // ------- Colonna sinistra: Mode / State (manteniamo la UI come prima) -------
  const char* modeValue  = uiGetModeLabel();   // es: WORK / LIVE
  const char* stateValue = "TARE";
  if (state == ScaleState::TARE_UI_OK) {
    stateValue = "OK";
  } else if (state == ScaleState::TARE_UI_FAILED) {
    stateValue = "NO";
  }

  const int16_t LABEL_X = 2;
  const int16_t VALUE_X = 44;

  oled.setFont(u8g2_font_6x12_tr);

  // Riga 1: Mode
  oled.drawStr(LABEL_X, 24, "Mode:");
  oled.drawStr(VALUE_X, 24, modeValue);

  // Riga 2: State
  oled.drawStr(LABEL_X, 36, "State:");
  oled.drawStr(VALUE_X, 36, stateValue);

  // ------- Stato TARA: stabilizzazione, non countdown -------
  const int16_t TARE_AREA_X = 86;
  const int16_t TARE_AREA_W = 166;

  oled.setFont(u8g2_font_logisoso24_tf);
  const char* txt = "- TARA -";
  if (state == ScaleState::TARE_UI_OK) {
    txt = "TARA OK";
  } else if (state == ScaleState::TARE_UI_FAILED) {
    txt = "INSTABILE";
  }
  drawCenteredTextInArea(txt, TARE_AREA_X, TARE_AREA_W, 40);

  oled.setFont(u8g2_font_6x12_tr);
  if (state == ScaleState::TARE_UI_FAILED) {
    drawCenteredTextInArea("Ripeti a peso fermo", TARE_AREA_X, TARE_AREA_W, 58);
  } else if (state == ScaleState::TARE_UI_OK) {
    drawCenteredTextInArea("Zero aggiornato", TARE_AREA_X, TARE_AREA_W, 58);
  } else {
    drawCenteredTextInArea(autoMode ? "Auto" : "Manuale", TARE_AREA_X, TARE_AREA_W, 52);

    const uint8_t phase = (millis() / 180) % 3;
    const int16_t dotY = 57;
    const int16_t dotW = 5;
    const int16_t gap = 8;
    const int16_t dotsW = dotW * 3 + gap * 2;
    const int16_t x0 = TARE_AREA_X + (TARE_AREA_W - dotsW) / 2;
    for (uint8_t i = 0; i < 3; i++) {
      int16_t x = x0 + i * (dotW + gap);
      if (i == phase) {
        oled.drawBox(x, dotY, dotW, dotW);
      } else {
        oled.drawFrame(x, dotY, dotW, dotW);
      }
    }
  }

  oled.sendBuffer();
}


void ui_renderHxError(bool hard, bool showLast, long lastG) {
  oled.clearBuffer();

  oled.setFont(u8g2_font_6x12_tr);

  // Titolo
  drawCenteredTextUTF8("ERRORE SENSORE PESO", 14);
  drawCenteredTextUTF8(hard ? "HX711: ERROR HARD" : "HX711: ERROR", 28);

  // Richiesta: in schermata ERRORE non mostrare l'ultimo peso rilevato.
  (void)showLast;
  (void)lastG;
  drawCenteredTextUTF8("Nessun dato valido", 56);

  oled.sendBuffer();
}

// =============================================================================
// CALIBRATION WIZARD UI
// =============================================================================

// Helper: disegna una barra di progresso
static void drawProgressBar(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t pct) {
  if (pct > 100) pct = 100;
  oled.drawFrame(x, y, w, h);
  int16_t fillW = (int16_t)((w - 2) * (int32_t)pct / 100);
  if (fillW < 0) fillW = 0;
  if (fillW > (w - 2)) fillW = w - 2;
  if (fillW > 0) {
    oled.drawBox(x + 1, y + 1, fillW, h - 2);
  }
}

void ui_renderCalStepZero(uint8_t progress) {
  oled.clearBuffer();

  // Titolo
  oled.setFont(u8g2_font_logisoso16_tf);
  drawCenteredText("CALIBRAZIONE", 18);

  // Istruzione
  oled.setFont(u8g2_font_6x12_tr);
  drawCenteredText("1/4 - Piatto vuoto", 34);
  drawCenteredText("Rimuovi tutto e premi ENTER", 46);

  // Barra progresso campionamento
  drawProgressBar(40, 52, 176, 8, progress);

  // Hint in basso
  oled.setFont(u8g2_font_5x8_tr);
  oled.drawStr(4, 62, "CLEAR=annulla");

  oled.sendBuffer();
}

void ui_renderCalStepPlace(uint8_t progress) {
  oled.clearBuffer();

  oled.setFont(u8g2_font_logisoso16_tf);
  drawCenteredText("CALIBRAZIONE", 18);

  oled.setFont(u8g2_font_6x12_tr);
  drawCenteredText("2/4 - Appoggia peso", 34);
  drawCenteredText("Peso stabile, premi ENTER", 46);

  drawProgressBar(40, 52, 176, 8, progress);

  oled.setFont(u8g2_font_5x8_tr);
  oled.drawStr(4, 62, "CLEAR=annulla");

  oled.sendBuffer();
}

void ui_renderCalStepValue(uint16_t refWeightG) {
  oled.clearBuffer();

  oled.setFont(u8g2_font_logisoso16_tf);
  drawCenteredText("CALIBRAZIONE", 18);

  oled.setFont(u8g2_font_6x12_tr);
  drawCenteredText("3/4 - Peso riferimento", 32);

  // Mostra il peso selezionato grande
  char buf[16];
  formatGramsWithDot((long)refWeightG, buf, sizeof(buf));

  oled.setFont(u8g2_font_logisoso24_tn);
  int16_t numW = oled.getStrWidth(buf);
  oled.setFont(u8g2_font_6x12_tr);
  int16_t gW = oled.getStrWidth("g");

  int16_t totalW = numW + 4 + gW;
  int16_t startX = (256 - totalW) / 2;

  oled.setFont(u8g2_font_logisoso24_tn);
  oled.drawStr(startX, 54, buf);
  oled.setFont(u8g2_font_6x12_tr);
  oled.drawStr(startX + numW + 4, 54, "g");

  // Istruzioni
  oled.setFont(u8g2_font_5x8_tr);
  oled.drawStr(4, 62, "SKIP=+ TARE=- ENTER=ok");

  oled.sendBuffer();
}

void ui_renderCalStepConfirm(uint16_t refWeightG, float cpg) {
  oled.clearBuffer();

  oled.setFont(u8g2_font_logisoso16_tf);
  drawCenteredText("CONFERMA?", 18);

  oled.setFont(u8g2_font_6x12_tr);

  // Mostra i dati calcolati
  char line1[32];
  snprintf(line1, sizeof(line1), "Peso ref: %u g", refWeightG);
  drawCenteredText(line1, 34);

  char line2[32];
  snprintf(line2, sizeof(line2), "CPG: %.2f", cpg);
  drawCenteredText(line2, 48);

  oled.setFont(u8g2_font_5x8_tr);
  oled.drawStr(4, 62, "ENTER=salva  CLEAR=annulla");

  oled.sendBuffer();
}

void ui_renderCalSaveResult(bool ok) {
  oled.clearBuffer();

  oled.setFont(u8g2_font_logisoso16_tf);
  drawCenteredText(ok ? "SALVATA" : "ERRORE", 22);

  oled.setFont(u8g2_font_6x12_tr);
  drawCenteredText(ok ? "NVS verificata" : "Salvataggio fallito", 42);

  oled.setFont(u8g2_font_5x8_tr);
  drawCenteredText(ok ? "Calibrazione attiva" : "Ripeti calibrazione", 60);

  oled.sendBuffer();
}

void ui_renderCalLongPress(uint8_t progress) {
  oled.clearBuffer();

  oled.setFont(u8g2_font_6x12_tr);
  drawCenteredText("Tieni premuto SKIP", 28);
  drawCenteredText("per avviare calibrazione", 42);

  // Barra di progresso
  drawProgressBar(40, 50, 176, 10, progress);

  oled.sendBuffer();
}

// =============================================================================
// WEIGH STACK OVERLAYS
// =============================================================================

void ui_renderStackCompare(long gTotal, long gNet, int count) {
  oled.clearBuffer();

  char regCore[16];
  char effCore[16];
  char diffCore[16];
  char regStr[20];
  char effStr[20];
  char diffStr[20];

  formatGramsWithDot(gTotal, regCore, sizeof(regCore));
  formatGramsWithDot(gNet, effCore, sizeof(effCore));

  long delta = gNet - gTotal;
  formatGramsWithDot((delta < 0) ? -delta : delta, diffCore, sizeof(diffCore));

  strlcpy(regStr, regCore, sizeof(regStr));
  strlcpy(effStr, effCore, sizeof(effStr));
  if (delta > 0) {
    snprintf(diffStr, sizeof(diffStr), "+%s", diffCore);
  } else if (delta < 0) {
    snprintf(diffStr, sizeof(diffStr), "-%s", diffCore);
  } else {
    strlcpy(diffStr, "0", sizeof(diffStr));
  }

  const int16_t Y1 = 18;
  const int16_t Y2 = 40;
  const int16_t Y3 = 62;
  const int16_t RIGHT_X = 254;

  // Labels (Italian)
  oled.setFont(u8g2_font_7x13_tr);
  oled.drawStr(2, Y1, "Registrato:");
  oled.drawStr(2, Y2, "Effettivo:");
  oled.drawStr(2, Y3, "Differenza:");

  // Values large and right aligned with small unit 'g'
  oled.setFont(u8g2_font_logisoso16_tn);
  int16_t wReg = oled.getStrWidth(regStr);
  int16_t wEff = oled.getStrWidth(effStr);
  int16_t wDif = oled.getStrWidth(diffStr);

  oled.setFont(u8g2_font_6x12_tr);
  int16_t wG = oled.getStrWidth("g");
  int16_t gX = RIGHT_X - wG;

  oled.setFont(u8g2_font_logisoso16_tn);
  oled.drawStr(gX - 2 - wReg, Y1, regStr);
  oled.drawStr(gX - 2 - wEff, Y2, effStr);
  oled.drawStr(gX - 2 - wDif, Y3, diffStr);

  oled.setFont(u8g2_font_6x12_tr);
  oled.drawStr(gX, Y1, "g");
  oled.drawStr(gX, Y2, "g");
  oled.drawStr(gX, Y3, "g");

  (void)count;

  oled.sendBuffer();
}

void ui_renderStackClear(const char* msg, int remaining) {
  oled.clearBuffer();

  // Messaggio grande al centro
  oled.setFont(u8g2_font_logisoso24_tf);
  drawCenteredText(msg, 32);

  // Info rimanenti
  oled.setFont(u8g2_font_6x12_tr);
  char info[24];
  snprintf(info, sizeof(info), "Stack: %d", remaining);
  drawCenteredText(info, 52);

  oled.sendBuffer();
}
