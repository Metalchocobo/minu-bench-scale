#pragma once

#include <Arduino.h>

// Driver minimale DFPlayer Mini (UART 9600, protocollo 0x7E..0xEF)
// - Non dipende da librerie esterne.
// - Usa HardwareSerial (ESP32).

namespace DFPlayer {

// Config pin: TX ESP32 -> RX DFPlayer, RX ESP32 <- TX DFPlayer (opzionale)
struct Pins {
  int tx;
  int rx; // può essere -1 se non collegato
};

// Inizializza seriale DFPlayer.
// Ritorna true se inizializzato (non garantisce che SD sia pronta).
bool begin(const Pins& pins, uint32_t baud = 9600);

// Chiude seriale (utile quando si riusa il pin TX per LED sleep).
void end();

// Volume 0..30
void setVolume(uint8_t vol);

// Play track per indice (1..2999 tipicamente). Usa numerazione, non nome file.
void playTrack(uint16_t track);

// Play file in cartella /mp3 con nome 0001.mp3..
// (DFPlayer: comando 0x12, parametro = numero traccia)
void playMp3(uint16_t track);

// Stop
void stop();

// Sleep/Wake del chip
void sleep();
void wake();

// Utility: prepara per light-sleep dell'ESP32 (stop + sleep + end UART)
void prepareForEspLightSleep();

// Utility: ripristina dopo wake ESP32 (begin + wake)
void restoreAfterEspWake(const Pins& pins, uint8_t vol);

} // namespace DFPlayer
