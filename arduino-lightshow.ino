/**
 * Lightshow Arduino nano version
 * 
 * D12 - D8 : lights
 * Ref - Voltage reference (Only FREE RUN mode)
 * A0  - Audio input
 * A4 + A5 - Display
 */

#include <fix_fft.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

// #define FREE_RUN_MODE       // Requires the AREF pin. Not working in Mini pro :(
                               // Normal mode uses internal DEFAULT for reference

#define SCR_ROTATION  2

#define SAMPLES       128
#define FRAME_TIME    63       // Desired time per frame in ms (63 is ~15 fps)

// Led control
#define LD_COUNT      5
#define LD_FLASH      0
#define LD_ERROR      4
#define LD_DECAY_REL  4
#define LD_MIN_DECAY  1
#define LD_MAX_DECAY  100

// Energy value increases with the sum of frequencies
// and decreases on the time.
#define ENERGY_MAX    255
#define ENERGY_MULT   3.2
#define ENERGY_KICK   5
#define ENERGY_DECAY  18
#define ENERGY_HIGH   180
#define ENERGY_BASE_FREQ  3
#define ENERGY_SMOOTHNESS 8

// Strobe is increased on high energy. When it reach
// ST_START value, the flash led starts to blink.
// It´s resetted every time a kick is detected
#define ST_START      120
#define ST_INCREMENT  5
#define ST_DECAY      15

// Silence detection
#define SILENCE_MIN_LEVEL 5
#define SILENCE_FRAMES    20

// Trying to keep audio signal high all the time
#define AMP_MAX_MULT  2
#define AMP_MAX_LEVEL 32
#define AMP_RELEASE   .04

const uint8_t LED[] = { 12, 11, 10, 9, 6 };                 // PWD enabled pins

// Frequency is very timing dependant, so we define different
// values for different methods.
#ifdef FREE_RUN_MODE
const uint8_t LED_THRESHOLD[] = { 9, 7, 6, 7, 10 };
const uint8_t FREQ_SPLITS[][2] { 
   { 0, 1 },
   { 1, 4 },
   { 4, 7 },
   { 7, 10 },
   { 10, 32 }
};
#else
const uint8_t LED_THRESHOLD[] = { 9, 7, 6, 7, 10 };
const uint8_t FREQ_SPLITS[][2] { 
   { 0, 2 },
   { 1, 5 },
   { 5, 10 },
   { 10, 18 },
   { 18, 32 }  
};
#endif

// FPS control
double delta = 1;
double lastFrameTime = 0;

// Sampling
int8_t vReal[SAMPLES];
int8_t vImag[SAMPLES];

double led_values[LD_COUNT] = { 0, 0, 0, 0, 0 };
uint8_t decay = 1;
double amplify = AMP_MAX_MULT;
double energy = 0;
uint8_t energySmooth = 0;
uint8_t strobe = 0;
bool strobePos = 0;
uint8_t silence = 0;

// Initialize screen. Following line is for OLED 128x64 connected by I2C
Adafruit_SSD1306 display(128, 64, &Wire, -1);

void fps() {
  while (millis() - lastFrameTime < FRAME_TIME);
  delta = (double)(millis() - lastFrameTime) / FRAME_TIME;
  lastFrameTime = millis();
}
 
void setup() {
  // Serial.begin(115200);
  
  // Setup leds
  for (uint8_t i=0; i<LD_COUNT; i++) {
    pinMode(LED[i], OUTPUT);
    digitalWrite(LED[i], HIGH);
    delay(100);
    digitalWrite(LED[i], LOW);
  }

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Fixed from 0x3D
    digitalWrite(LED[LD_ERROR], HIGH);
    while (1); // Don't proceed, loop forever
  }

  // Rotate the screen
  display.setRotation(SCR_ROTATION);

  #ifdef FREE_RUN_MODE
  // TIMSK0 = 1;            // turn off timer0 for lower jitter
  ADCSRA = 0b11100101;      // set ADC to free running mode and set pre-scalar to 32 (0xe5)
  ADMUX = 0b00000000;       // use pin A0 and external voltage reference
  DIDR0 = 0x01;             // turn off the digital input for adc0
  #else
  analogReference(DEFAULT);
  #endif
  
  delay(50);                // wait to get reference voltage stabilized  
}
 
void loop() {
  uint8_t led_avgs[LD_COUNT] = { 0, 0, 0, 0, 0 };
  uint8_t inputPeak = 0;
  int8_t ampPeak;
  int8_t fundamental = -1;
  int8_t fqPeak = 0;
  uint8_t i, j;
  bool flash_on;

  display.clearDisplay();
  
  // Sampling
  for(i=0; i<SAMPLES; i++) {
    #ifdef FREE_RUN_MODE
    while(!(ADCSRA & 0x10));                // wait for ADC to complete current conversion ie ADIF bit set
    ADCSRA = 0b11110101 ;                   // clear ADIF bit so that ADC can do next operation (0xf5)
    vReal[i]= (ADC - 512) / 8;              // Copy to bins after compressing
    #else
    vReal[i]= (analogRead(A0) - 512) / 8;              // Copy to bins after compressing
    #endif
    vImag[i] = 0;

    if (abs(vReal[i]) > inputPeak) {
      inputPeak = abs(vReal[i]);
    }
  }

  // Silence detection
  if (inputPeak < SILENCE_MIN_LEVEL) {
    if (silence < SILENCE_FRAMES) {
      silence++;
    } else {
      // Flat vReal to 0. We want to keep going with display and leds.
      for (i=0; i<SAMPLES; i++) {
        vReal[i] = 0;
      }
      // also truncate the energy
      energy = 0;
    }
  } else {
    silence = 0;
  }

  // Amplify (skip if silence  
  ampPeak = (double) inputPeak * amplify;
  if (ampPeak < AMP_MAX_LEVEL) {
    if (amplify < AMP_MAX_MULT) {
      amplify += AMP_RELEASE; 
    }
  } else if (ampPeak > AMP_MAX_LEVEL) {
    amplify = (double) AMP_MAX_LEVEL / inputPeak;
  }

  for(i=0; i<SAMPLES; i++) {
    vReal[i] = (double) amplify * vReal[i];
    
    // draw osc 
    display.drawPixel(i*2, 32 + vReal[i] / 4, 1);
  }
  
  // FFT
  fix_fft(vReal, vImag, 7, 0);   
  for (i = 0; i < 64; i++) {
    vReal[i] = sqrt(vReal[i] * vReal[i] + vImag[i] * vImag[i]); // Make values positive
    if (fundamental == -1 || vReal[fundamental] < vReal[i]) {
      fundamental = i;
    }
    if (vReal[i] > fqPeak) {
      fqPeak = vReal[i];
    }
  }
 
  // FFT processing
  energy = max(0, energy - ENERGY_DECAY * delta);
  
  for(i=0; i<32; i++) {
    // Led value calculation
    for (j=0; j<LD_COUNT; j++) {
      if (i >= FREQ_SPLITS[j][0] && i < FREQ_SPLITS[j][1]) {
        led_avgs[j] = led_avgs[j] + vReal[i];
      }
    }

    // energy calculation
    if (i > ENERGY_BASE_FREQ) {
      energy = min(ENERGY_MAX, energy + (double) vReal[i] / fqPeak * ENERGY_MULT);
    }
  }
    
  // Flash only turns on when increases
  flash_on = led_avgs[LD_FLASH] - led_values[LD_FLASH] > LED_THRESHOLD[LD_FLASH];
  
  if (flash_on) {
    energy = min(ENERGY_MAX, energy + ENERGY_KICK);
  }

  // Smoothed energy
  energySmooth += (double) (energy - energySmooth) / ENERGY_SMOOTHNESS;

  /*
  Serial.print(energySmooth);
  Serial.print(" ");
  Serial.print(energy);
  Serial.println();
  */
  
  // led control
  for (j=0; j<LD_COUNT; j++) {
    bool on = j == LD_FLASH 
      ? flash_on 
      : led_avgs[j] > LED_THRESHOLD[j];

    if (j == LD_FLASH) {
       // High energy. Fast leds
      led_values[j] = flash_on ? 255 : 0;
    } else {
      if (on) {
        led_values[j] = min(led_avgs[j], (double) led_values[j] + decay * delta);
      } else {
        led_values[j] = max(0, (double) led_values[j] - decay * delta);
      }
    }

    // Draw pills
    char siz = min(8, (double) led_values[j] / (double) LED_THRESHOLD[j] * 8);
    
    if (on) {
      display.fillRoundRect(j * 13 + 4 - siz / 2, 4 - siz / 2, siz, siz, 2, 1); 
    } else {
      display.drawRoundRect(j * 13 + 4 - siz / 2, 4 - siz / 2, siz, siz, 2, 1);  
    }

    display.drawFastHLine(j * 13, 10, led_avgs[j] / 8, 1);
    
    analogWrite(LED[j], (double) siz / 8 * 128);
  }
  
  // Decay control
  if (flash_on) {
    decay = LD_MAX_DECAY;
    // reset strobe
    strobe = 0;
  } else  if (decay > 1) {
    decay = max(LD_MIN_DECAY, (double) decay - LD_DECAY_REL * delta);
  }

  // Strobe control
  if (energy > ENERGY_HIGH) {
    strobe = min(250, (double) strobe + ST_INCREMENT * delta);
  } else {
    strobe = max(0, (double) strobe - ST_DECAY * delta);
  }

  if (strobe > ST_START) {
    digitalWrite(LED[LD_FLASH], strobePos);
    strobePos = !strobePos;
  }

  // draw columns
  for (uint8_t i=0; i<32; i++) {
    display.drawFastVLine(i * 4, 63 - vReal[i], vReal[i], 1);  
  }

  // peak line
  display.drawFastHLine(64, 0, (double) ampPeak / AMP_MAX_LEVEL * 64, 1);

  // amp line
  display.drawFastHLine(64, 2, amplify / AMP_MAX_MULT * 64, 1);

  // show decay line
  display.drawFastHLine(64, 4, (double) decay / LD_MAX_DECAY * 64, 1);

  // show energy
  display.drawFastHLine(64, 6, (double) energy / 256 * 64, 1);

  // show strobe line
  display.drawFastHLine(64, 8, (double) strobe / 256 * 64, 1);
  display.drawPixel(64 + (double) ST_START / 256 * 64, 8, 1);
  
  display.display();
  // -- send to display according measured value

  fps();
} 
