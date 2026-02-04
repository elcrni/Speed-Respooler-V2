// -----------------------------
// Filament Respooler V3 by Layerguru.com
// -----------------------------
#include <Arduino.h>
#include <SPI.h>
#include <Preferences.h>
#include <math.h>
#include <string.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1351.h>

#include "esp_arduino_version.h"

// -----------------------------
// CONFIRMED WORKING PINOUT (XIAO ESP32C6)
// -----------------------------
constexpr int PIN_RPWM = 20; // D9  -> GPIO20 (BTS7960 RPWM)

// Optional BTS7960 pins (only if you wired them; otherwise leave -1)
constexpr int PIN_LPWM = -1;
constexpr int PIN_REN  = -1;
constexpr int PIN_LEN  = -1;

// Pot
constexpr int PIN_POT = 0;  // D0 -> GPIO0 (ADC)

// Safety sensors
constexpr int PIN_FILAMENT  = 16; // active-low, INPUT_PULLUP (loaded = LOW)
constexpr int PIN_SPOOLFULL = 17; // NC wiring, opens -> HIGH (FULL)

// OLED SSD1351 SPI
constexpr int PIN_SPI_SCK   = 19; // D8  -> GPIO19
constexpr int PIN_SPI_MOSI  = 18; // D10 -> GPIO18
constexpr int PIN_OLED_CS   = 2;  // D2  -> GPIO2
constexpr int PIN_OLED_DC   = 1;  // D1  -> GPIO1
constexpr int PIN_OLED_RST  = 21; // D3  -> GPIO21

// -----------------------------
// PWM config
// -----------------------------
constexpr uint32_t PWM_FREQ = 20000;
constexpr uint8_t  PWM_RES_BITS = 10;
constexpr uint32_t PWM_MAX = (1UL << PWM_RES_BITS) - 1;

#if ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 0, 0)
constexpr uint8_t PWM_CH = 0;
#endif

// -----------------------------
// Timings
// -----------------------------
constexpr uint32_t POT_INTERVAL_MS = 80;
constexpr uint32_t SPLASH_MS       = 4000;
constexpr uint32_t DEBOUNCE_MS     = 30;

// -----------------------------
// State
// -----------------------------
static bool  motor_running = false;
static float speed_target  = 30.0f; // persisted
static bool  pot_armed     = false;
static float pot_last_pct  = 0.0f;

static bool     show_splash = true;
static uint32_t splash_start_ms = 0;

// -----------------------------
// ADC smoothing (window 5)
// -----------------------------
constexpr uint8_t POT_WIN = 5;
static uint16_t pot_mv_buf[POT_WIN] = {0};
static uint8_t  pot_mv_i = 0;
static bool     pot_buf_full = false;

// -----------------------------
// Debounce
// -----------------------------
struct DebouncedBool {
  bool stable = false;
  bool last_read = false;
  uint32_t last_change_ms = 0;
};

static DebouncedBool db_filament;
static DebouncedBool db_spool;

// -----------------------------
// Storage
// -----------------------------
Preferences prefs;

// -----------------------------
// Display
// -----------------------------
Adafruit_SSD1351 oled = Adafruit_SSD1351(128, 128, &SPI, PIN_OLED_CS, PIN_OLED_DC, PIN_OLED_RST);

// Colors
static uint16_t C_ORANGE, C_BLUE, C_RED, C_WHITE, C_GREY, C_BLACK;

// -----------------------------
// UI cache (prevents redraw flicker)
// -----------------------------
static bool ui_static_drawn = false;
static bool last_motor_running = false;
static int  last_speed_int = -1;
static bool last_filament = false;
static bool last_spool = false;
static int  last_hint = -999;

// -----------------------------
// Helpers
// -----------------------------
static inline bool filament_loaded() { return db_filament.stable; }
static inline bool spool_full()      { return db_spool.stable; }
static inline bool sensors_ok()      { return filament_loaded() && !spool_full(); }

static void debounce_update(DebouncedBool &db, bool raw_value, uint32_t now_ms) {
  if (raw_value != db.last_read) {
    db.last_read = raw_value;
    db.last_change_ms = now_ms;
  }
  if ((now_ms - db.last_change_ms) >= DEBOUNCE_MS && db.stable != db.last_read) {
    db.stable = db.last_read;
  }
}

// Fixed-width text overwrite (no flicker)
static void print_fixed(int x, int y, uint16_t fg, const char* s, int width_chars) {
  oled.setTextSize(1);
  oled.setTextWrap(false);
  oled.setTextColor(fg, C_BLACK);   // background overwrite
  oled.setCursor(x, y);
  int len = (int)strlen(s);
  oled.print(s);
  for (int i = len; i < width_chars; i++) oled.print(' ');
}

// -----------------------------
// BTS7960 helpers (optional pins)
// -----------------------------
static void bts7960_init_pins() {
  if (PIN_LPWM >= 0) { pinMode(PIN_LPWM, OUTPUT); digitalWrite(PIN_LPWM, LOW); }
  if (PIN_REN  >= 0) { pinMode(PIN_REN,  OUTPUT); digitalWrite(PIN_REN,  HIGH); }
  if (PIN_LEN  >= 0) { pinMode(PIN_LEN,  OUTPUT); digitalWrite(PIN_LEN,  HIGH); }
}

static void bts7960_forward_mode() {
  if (PIN_LPWM >= 0) digitalWrite(PIN_LPWM, LOW);
  if (PIN_REN  >= 0) digitalWrite(PIN_REN,  HIGH);
  if (PIN_LEN  >= 0) digitalWrite(PIN_LEN,  HIGH);
}

// -----------------------------
// PWM
// -----------------------------
static void pwm_begin() {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcAttach(PIN_RPWM, PWM_FREQ, PWM_RES_BITS);
#else
  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES_BITS);
  ledcAttachPin(PIN_RPWM, PWM_CH);
#endif
}

static void pwm_write_duty_float(float duty_0_1) {
  if (duty_0_1 < 0) duty_0_1 = 0;
  if (duty_0_1 > 1) duty_0_1 = 1;
  uint32_t duty = (uint32_t)lroundf(duty_0_1 * (float)PWM_MAX);

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(PIN_RPWM, duty);
#else
  ledcWrite(PWM_CH, duty);
#endif
}

static void motor_apply_speed() {
  float s = speed_target;
  if (s < 0) s = 0;
  if (s > 100) s = 100;

  bts7960_forward_mode();
  pwm_write_duty_float(s / 100.0f);
}

static void motor_stop_disarm() {
  pwm_write_duty_float(0.0f);
  if (PIN_LPWM >= 0) digitalWrite(PIN_LPWM, LOW);
  motor_running = false;
  pot_armed = false;
}

static void motor_stop_arm() {
  pwm_write_duty_float(0.0f);
  if (PIN_LPWM >= 0) digitalWrite(PIN_LPWM, LOW);
  motor_running = false;
  pot_armed = true;
}

static void motor_start_checked() {
  if (sensors_ok()) {
    motor_running = true;
    motor_apply_speed();
  } else {
    motor_stop_disarm();
  }
}

// -----------------------------
// Pot logic (same behavior as ESPHome)
// -----------------------------
static void pot_update() {
  uint16_t mv = (uint16_t)analogReadMilliVolts(PIN_POT);

  pot_mv_buf[pot_mv_i] = mv;
  pot_mv_i = (pot_mv_i + 1) % POT_WIN;
  if (pot_mv_i == 0) pot_buf_full = true;

  uint32_t sum = 0;
  uint8_t n = pot_buf_full ? POT_WIN : pot_mv_i;
  if (n == 0) n = 1;
  for (uint8_t i = 0; i < n; i++) sum += pot_mv_buf[i];
  float mv_avg = (float)sum / (float)n;

  float v = mv_avg / 1000.0f;
  if (v < 0.03f) v = 0.0f;
  if (v > 3.27f) v = 3.3f;

  float pct = (v / 3.3f) * 100.0f;

  if (fabsf(pct - pot_last_pct) < 0.7f) return;

  // 0% region -> STOP and ARM
  if (pct <= 1.0f) {
    speed_target = 0.0f;
    pot_last_pct = pct;
    motor_stop_arm();
    return;
  }

  bool rising_from_zero = (pot_last_pct <= 1.0f);
  pot_last_pct = pct;
  speed_target = pct;

  // Save occasionally
  static float last_saved = -999.0f;
  if (fabsf(speed_target - last_saved) >= 2.0f) {
    prefs.putFloat("speed_target", speed_target);
    last_saved = speed_target;
  }

  // Safety interlock
  if (!sensors_ok()) {
    motor_stop_disarm();
    return;
  }

  // Running -> update speed
  if (motor_running) {
    motor_apply_speed();
    return;
  }

  // Off -> start only if armed and rising from zero
  if (pot_armed && rising_from_zero) {
    motor_start_checked();
  }
}

// -----------------------------
// Display (ORIGINAL sizes/positions, ORANGE kept)
// -----------------------------
static void draw_splash_once() {
  oled.fillScreen(C_BLACK);
  oled.setTextWrap(false);

  oled.setTextSize(2);
  oled.setTextColor(C_ORANGE);
  oled.setCursor(8, 18);  oled.print("FILAMENT");
  oled.setCursor(8, 42);  oled.print("REWINDER");

  oled.setTextSize(1);
  oled.setTextColor(C_WHITE);
  oled.setCursor(8, 70);  oled.print("by Layerguru.com");
}

static void draw_main_static_once() {
  oled.fillScreen(C_BLACK);
  oled.setTextWrap(false);

  // Title (original size/position) in ORANGE
  oled.setTextSize(1);
  oled.setTextColor(C_ORANGE);
  oled.setCursor(0, 0);
  oled.print("FILAMENT REWINDER");

  // Labels
  oled.setTextColor(C_BLUE);
  oled.setCursor(0, 24); oled.print("Motor:");
  oled.setCursor(0, 52); oled.print("Filament:");
  oled.setCursor(0, 80); oled.print("Spool:");

  ui_static_drawn = true;

  // Force first dynamic draw
  last_motor_running = !motor_running;
  last_speed_int = -1;
  last_filament = !filament_loaded();
  last_spool = !spool_full();
  last_hint = -999;
}

static void draw_main_dynamic_if_changed() {
  const bool f = filament_loaded();
  const bool s = spool_full();
  const int speed_i = (int)lroundf(speed_target);

  int hint = 0;
  if (!f) hint = 1;
  else if (s) hint = 2;
  else if (!pot_armed && !motor_running) hint = 3;
  else if (pot_armed && !motor_running) hint = 4;
  else hint = 5;

  // Motor value
  if (motor_running != last_motor_running || speed_i != last_speed_int) {
    char buf[16];
    if (motor_running) snprintf(buf, sizeof(buf), "ON  %3d%%", speed_i);
    else               snprintf(buf, sizeof(buf), "OFF %3d%%", speed_i);

    uint16_t col = motor_running ? C_BLUE : C_GREY;
    print_fixed(54, 24, col, buf, 9);

    last_motor_running = motor_running;
    last_speed_int = speed_i;
  }

  // Filament value
  if (f != last_filament) {
    if (f) print_fixed(72, 52, C_BLUE, "LOADED", 6);
    else   print_fixed(72, 52, C_RED,  "EMPTY",  6);
    last_filament = f;
  }

  // Spool value
  if (s != last_spool) {
    if (s) print_fixed(54, 80, C_RED,  "FULL!", 5);
    else   print_fixed(54, 80, C_BLUE, "OK",    5);
    last_spool = s;
  }

  // Bottom hint
  if (hint != last_hint) {
    switch (hint) {
      case 1: print_fixed(0, 108, C_RED,  "NO FILAMENT - STOP", 21); break;
      case 2: print_fixed(0, 108, C_RED,  "SPOOL FULL - STOP",  21); break;
      case 3: print_fixed(0, 108, C_RED,  "Turn to MIN to ARM", 21); break;
      case 4: print_fixed(0, 108, C_GREY, "ARMED and Ready!",   21); break;
      default:print_fixed(0, 108, C_GREY, "RUNNING...",         21); break;
    }
    last_hint = hint;
  }
}

void setup() {
  prefs.begin("rewinder", false);
  speed_target = prefs.getFloat("speed_target", 30.0f);

  pinMode(PIN_FILAMENT, INPUT_PULLUP);
  pinMode(PIN_SPOOLFULL, INPUT_PULLUP);
  analogSetAttenuation(ADC_11db);

  bts7960_init_pins();
  pwm_begin();
  motor_stop_disarm();

  // OLED immediately (no Wi-Fi)
  SPI.begin(PIN_SPI_SCK, /*MISO*/ -1, PIN_SPI_MOSI);
  oled.begin();

  // Color palette (same vibe as ESPHome)
  C_ORANGE = oled.color565(255, 140, 0);
  C_BLUE   = oled.color565(0, 140, 255);
  C_RED    = oled.color565(255, 0, 0);
  C_WHITE  = oled.color565(255, 255, 255);
  C_GREY   = oled.color565(120, 120, 120);
  C_BLACK  = oled.color565(0, 0, 0);

  // Init debouncers
  uint32_t now = millis();
  bool raw_filament_loaded = (digitalRead(PIN_FILAMENT) == LOW);
  bool raw_spool_full      = (digitalRead(PIN_SPOOLFULL) == HIGH);

  db_filament = { raw_filament_loaded, raw_filament_loaded, now };
  db_spool    = { raw_spool_full,      raw_spool_full,      now };

  // Splash
  show_splash = true;
  splash_start_ms = millis();
  ui_static_drawn = false;
  draw_splash_once();
}

void loop() {
  uint32_t now = millis();

  // Splash timeout -> main screen
  if (show_splash && (now - splash_start_ms) >= SPLASH_MS) {
    show_splash = false;
    ui_static_drawn = false;
  }

  // Debounce sensors
  bool raw_filament_loaded = (digitalRead(PIN_FILAMENT) == LOW);
  bool raw_spool_full      = (digitalRead(PIN_SPOOLFULL) == HIGH);

  bool prev_filament = db_filament.stable;
  bool prev_spool    = db_spool.stable;

  debounce_update(db_filament, raw_filament_loaded, now);
  debounce_update(db_spool, raw_spool_full, now);

  // Events
  if (prev_filament && !db_filament.stable) motor_stop_disarm();
  if (!prev_spool && db_spool.stable)       motor_stop_disarm();
  if (motor_running && !sensors_ok())       motor_stop_disarm();

  // Pot loop
  static uint32_t next_pot = 0;
  if (now >= next_pot) {
    next_pot = now + POT_INTERVAL_MS;
    pot_update();
  }

  // Display: draw only what changed
  if (!show_splash) {
    if (!ui_static_drawn) draw_main_static_once();
    draw_main_dynamic_if_changed();
  }
}
