#include <SD.h>

#include <Fluxamasynth.h>
#include <PgmChange.h>

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
#define SYNTH_PROGRAM_RH  22
#define SYNTH_PROGRAM_LH  22
#define SYNTH_VEL         127

#define LATCH_PIN  8
#define DATA_PIN   9
#define CLOCK_PIN  7

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

uint8_t button_values[NUM_SHIFT_REGISTERS] = {0}; // current values
uint8_t button_events[NUM_SHIFT_REGISTERS] = {0}; // buttons that changed state

Fluxamasynth synth = Fluxamasynth(SYNTH_RX_PIN, SYNTH_TX_PIN);

void setup() {
  Serial.begin(9600);
  
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN,  INPUT);

  digitalWrite(CLOCK_PIN, LOW);

  synth.setMasterVolume(255);
  synth.programChange(SYNTH_PROGRAM_RH, SYNTH_CHANNEL_RH);
  synth.programChange(SYNTH_PROGRAM_LH, SYNTH_CHANNEL_LF);
}

void loop() {
  read_keys();
  process_key_events();
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
  if (button_id < 65) { // piano orstradella bass key
    if (button_state) { // button pressed
      synth.noteOn(SYNTH_CHANNEL_RH, BUTTON_TO_MIDI[button_id], SYNTH_VEL);
    } else { // button released
      synth.noteOff(SYNTH_CHANNEL_RH, BUTTON_TO_MIDI[button_id]);
    }
  } else {
    
  }
}
