#include <U8g2lib.h>

#include <Fluxamasynth.h>
#include <PgmChange.h>

#include "digicordion.h"

/*
 * CD4021B Shift Registers
 * https://www.arduino.cc/en/Tutorial/ShftIn22
 * 
 * Fluxamasynth Library reference
 * https://moderndevice.com/documentation/using-the-fluxamasynth-arduino-library/ 
 * 
 */

#define SYNTH_RX_PIN      4
#define SYNTH_TX_PIN      5
#define SYNTH_CHANNEL_RH  0
#define SYNTH_CHANNEL_LF  1
#define SYNTH_PROGRAM_RH  4//21
#define SYNTH_PROGRAM_LH  22
#define SYNTH_VEL         127

#define LATCH_PIN  8
#define DATA_PIN   9
#define CLOCK_PIN  7

#define ROTOR_A_PIN 2
#define ROTOR_B_PIN 3

#define NUM_SHIFT_REGISTERS 6
#define LATCH_DELAY         20 // usec
#define SOFTCLOCK_DELAY     2 // usec

// buttons  0-40: right hand from F3 to A6
// buttons 41-64: stradella bass from ?? to ??
const uint8_t BUTTON_TO_MIDI[] = {
  53,  54,  55,  56,  57,  58,  59,  60,  61,  62,  63,  64,  65,  66,  67,  68, //  0-15 
  69,  70,  71,  72,  73,  74,  75,  76,  77,  78,  79,  80,  81,  82,  83,  84, // 16-31
  85,  86,  87,  88,  89,  90,  91,  92,  93,   0,   0,   0,   0,   0,   0,   0, // 32-47
    0,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 48-63
    0,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  // 64-79
};

const int8_t BUTTON_OFFSETS[] = {
  5, 1, 1, -3, 4, 1, -2, -6,
  11, 8, 5, 2, -1, -4, -7, -5,
  22, 19, 2, 2, -4, -7, -10, -5,
  8, 4, 1, -2, -4, -7, -7, -5,
  3, 6, 3, 5, -5, -7, -10, -6,
};

uint8_t button_values[NUM_SHIFT_REGISTERS] = {0}; // current values
uint8_t button_events[NUM_SHIFT_REGISTERS] = {0}; // buttons that changed state
uint8_t num_keys_pressed = 0;

volatile unsigned long threshold = 5000;
volatile uint8_t  rotaryHalfSteps = 0;
volatile unsigned long int0time = 0;
volatile unsigned long int1time = 0;
volatile uint8_t int0signal = 0;
volatile uint8_t int1signal = 0;
volatile uint8_t int0history = 0;
volatile uint8_t int1history = 0;

Fluxamasynth synth = Fluxamasynth(SYNTH_RX_PIN, SYNTH_TX_PIN);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

uint8_t voice_selection = 0;
uint8_t prev_voice_selection = !voice_selection;
char buffer[64];

void setup() {

  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN,  INPUT);
  digitalWrite(CLOCK_PIN, LOW);

  pinMode(ROTOR_A_PIN, INPUT);
  pinMode(ROTOR_B_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ROTOR_A_PIN), int0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTOR_B_PIN), int1, CHANGE);

  synth.setMasterVolume(255);
  synth.programChange(SYNTH_PROGRAM_RH, SYNTH_CHANNEL_RH);
  synth.programChange(SYNTH_PROGRAM_LH, SYNTH_CHANNEL_LF);

  u8g2.begin();
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);

  Serial.begin(9600);
  Serial.println("booted");
}


void loop() {
  //read_keys();
  //process_key_events();
  //delay(1000);
  delay(10);
  if (voice_selection != prev_voice_selection) {
    prev_voice_selection = voice_selection;
    draw_menu();
  }
}

/*
 * Read one byte from shift registers, call multiple times for multiple bytes.
 * Uses software clock. Transition from high to low triggers a value on the data line.
 */
uint8_t shift_in() {
  uint8_t value = 0;
  
  for (int i=0; i<8; i++) {
    digitalWrite(CLOCK_PIN, LOW);
    delayMicroseconds(SOFTCLOCK_DELAY);
    // TODO: reverse bit order might be simpler
    value = (value << 1) | digitalRead(DATA_PIN);
    digitalWrite(CLOCK_PIN, HIGH);
  }
  // leave clock low to preserve power ?
  digitalWrite(CLOCK_PIN, LOW);
  return value;
}

void read_keys() {
  uint8_t tmp;
  
  // trigger shift registers to read states
  digitalWrite(LATCH_PIN, HIGH);
  delayMicroseconds(LATCH_DELAY);
  digitalWrite(LATCH_PIN, LOW);

  // shift values from registers into memory
  for (int i=0; i<NUM_SHIFT_REGISTERS; i++) {
    tmp = shift_in();
    button_events[i] = button_values[i] ^ tmp;
    button_values[i] = tmp;
  }
}

void process_key_events() {
  uint8_t event, button_id;
  bool    button_state;
  for (int i=0; i<NUM_SHIFT_REGISTERS; i++) {
    event = button_events[i];
    for (int j=0; j<8; j++) {
      button_id = i * 8 + j;
      if (event & (1 << j)) { // button has changed state
        button_state = button_values[i] & (1 << j);
        handle_button(button_id, button_state);
      }
    }
  }
}

void handle_button(uint8_t button_id, bool button_state) {
  if (button_id < 65) { // piano or stradella bass key
    if (button_state) { // button pressed
      Serial.print("Note ");
      Serial.print(button_id, DEC);
      Serial.println(" on");
      synth.noteOff(SYNTH_CHANNEL_RH, BUTTON_TO_MIDI[button_id + BUTTON_OFFSETS[button_id]]);
      num_keys_pressed += 1;
    } else { // button released
      Serial.print("Note ");
      Serial.print(button_id, DEC);
      Serial.println(" off");
      synth.noteOn(SYNTH_CHANNEL_RH, BUTTON_TO_MIDI[button_id + BUTTON_OFFSETS[button_id]], SYNTH_VEL);
      num_keys_pressed -= 1;
    }
  } else {
    // control signals
  }
}

void draw_menu() {
  uint8_t first;
  
  u8g2.clearBuffer();
  
  if (voice_selection < 4) {
    first = 0;
  } else if (voice_selection < 124) {
    first = voice_selection - 4;
  } else {
    first = 120;
  }
  
  for (int i=0; i<8; i++) {
    strcpy_P(buffer, (char *)pgm_read_word(&(voice_names[i+first])));
    if (i + first == voice_selection) {
      u8g2.drawStr(0, i * 8, ">");
      u8g2.drawStr(8, i * 8, buffer);
    } else {
      u8g2.drawStr(8, i * 8, buffer);
    }
  }
  u8g2.sendBuffer();
}

void int0()
{
  if ( micros() - int0time < threshold )
    return;
  int0history = int0signal;
  int0signal = bitRead(PIND, 2);
  if ( int0history == int0signal )
    return;
  int0time = micros();
  if ( int0signal == int1signal )
    rotaryHalfSteps++;
  else
    rotaryHalfSteps--;
  voice_selection = rotaryHalfSteps / 2;
}

void int1()
{
  if ( micros() - int1time < threshold )
    return;
  int1history = int1signal;
  int1signal = bitRead(PIND, 3);
  if ( int1history == int1signal )
    return;
  int1time = micros();
  voice_selection = rotaryHalfSteps / 2;
}
