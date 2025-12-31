#include "hx711_driver.h"

static int g_dout = -1;
static int g_sck  = -1;
static uint8_t g_gainPulses = 1;
static uint8_t g_pulseUs    = 1;

// Critical section per leggere i 24 bit senza jitter WiFi/OTA (ESP32)
#if defined(ARDUINO_ARCH_ESP32)
static portMUX_TYPE g_hxMux = portMUX_INITIALIZER_UNLOCKED;
#define HX_CRIT_ENTER() portENTER_CRITICAL(&g_hxMux)
#define HX_CRIT_EXIT()  portEXIT_CRITICAL(&g_hxMux)
#else
#define HX_CRIT_ENTER() noInterrupts()
#define HX_CRIT_EXIT()  interrupts()
#endif

void hx711_begin(int doutPin, int sckPin, HX711Gain gain, uint8_t pulseUs){
  g_dout = doutPin;
  g_sck  = sckPin;
  g_pulseUs = (pulseUs == 0) ? 1 : pulseUs;
  hx711_set_gain(gain);

  pinMode(g_sck, OUTPUT);
  pinMode(g_dout, INPUT);

  // IMPORTANT: SCK LOW = attivo. SCK HIGH > 60us => power down.
  digitalWrite(g_sck, LOW);
}

void hx711_set_gain(HX711Gain gain){
  g_gainPulses = (uint8_t)gain;
  if (g_gainPulses < 1) g_gainPulses = 1;
  if (g_gainPulses > 3) g_gainPulses = 1;
}

bool hx711_is_ready(){
  if (g_dout < 0) return false;
  return digitalRead(g_dout) == LOW;
}

bool hx711_wait_ready(uint32_t timeoutMs){
  uint32_t t0 = millis();
  while (!hx711_is_ready()){
    if ((millis() - t0) > timeoutMs) return false;
    delay(2);
  }
  return true;
}

long hx711_read(){
  // chiamare solo quando hx711_is_ready() è true
  uint32_t value = 0;

  HX_CRIT_ENTER();

  for (uint8_t i = 0; i < 24; i++){
    digitalWrite(g_sck, HIGH);
    delayMicroseconds(g_pulseUs);
    value = (value << 1) | (uint32_t)(digitalRead(g_dout) & 0x01);
    digitalWrite(g_sck, LOW);
    delayMicroseconds(g_pulseUs);
  }

  // extra pulses per selezionare canale/gain
  for (uint8_t i = 0; i < g_gainPulses; i++){
    digitalWrite(g_sck, HIGH);
    delayMicroseconds(g_pulseUs);
    digitalWrite(g_sck, LOW);
    delayMicroseconds(g_pulseUs);
  }

  HX_CRIT_EXIT();

  // Sign-extend 24-bit two's complement
  if (value & 0x800000UL){
    value |= 0xFF000000UL;
  }
  return (long)(int32_t)value;
}

long hx711_read_blocking(uint32_t timeoutMs){
  if (!hx711_wait_ready(timeoutMs)) return 0;
  return hx711_read();
}
