#include "dfplayer_driver.h"

#include <HardwareSerial.h>

namespace DFPlayer {

static HardwareSerial g_ser(1);
static bool g_started = false;
static Pins g_pins{.tx = -1, .rx = -1};

// Frame:
// 0x7E 0xFF 0x06 CMD ACK P1 P2 CHK_H CHK_L 0xEF
// Checksum: 0xFFFF - (0xFF + 0x06 + CMD + ACK + P1 + P2) + 1
static void sendCmd(uint8_t cmd, uint16_t param, uint8_t ack = 0x00) {
  if (!g_started) return;
  uint8_t ver = 0xFF;
  uint8_t len = 0x06;
  uint8_t p1 = (param >> 8) & 0xFF;
  uint8_t p2 = (param >> 0) & 0xFF;
  uint16_t sum = (uint16_t)ver + (uint16_t)len + (uint16_t)cmd + (uint16_t)ack + (uint16_t)p1 + (uint16_t)p2;
  uint16_t chk = 0xFFFF - sum + 1;
  uint8_t buf[10] = {
    0x7E,
    ver,
    len,
    cmd,
    ack,
    p1,
    p2,
    (uint8_t)((chk >> 8) & 0xFF),
    (uint8_t)((chk >> 0) & 0xFF),
    0xEF
  };
  g_ser.write(buf, sizeof(buf));
  g_ser.flush();
}

bool begin(const Pins& pins, uint32_t baud) {
  g_pins = pins;
  // RX può essere -1 se non collegato (Arduino-ESP32 accetta -1)
  g_ser.begin(baud, SERIAL_8N1, pins.rx, pins.tx);
  g_started = true;
  return true;
}

void end() {
  if (!g_started) return;
  g_ser.flush();
  g_ser.end();
  g_started = false;
}

void setVolume(uint8_t vol) {
  if (vol > 30) vol = 30;
  sendCmd(0x06, vol); // Set volume
}

void playTrack(uint16_t track) {
  if (track < 1) track = 1;
  sendCmd(0x03, track); // Play track by index
}

void playMp3(uint16_t track) {
  if (track < 1) track = 1;
  // Play file in /MP3/0001.mp3 ... (command 0x12)
  sendCmd(0x12, track);
}

void stop() {
  sendCmd(0x16, 0x0000); // Stop
}

void sleep() {
  sendCmd(0x0A, 0x0000); // Enter sleep
}

void wake() {
  sendCmd(0x0B, 0x0000); // Wake up
}

void prepareForEspLightSleep() {
  // Cerchiamo di evitare che il DFPlayer interpreti transizioni del pin TX (che diventa LED)
  stop();
  delay(30);
  sleep();
  delay(30);
  end();
}

void restoreAfterEspWake(const Pins& pins, uint8_t vol) {
  begin(pins, 9600);
  delay(30);
  wake();
  delay(80);
  setVolume(vol);
}

} // namespace DFPlayer
