/**
 * Lightshow Arduino nano version
 * 
 * D12 - D8 : lights
 * Ref - Voltage reference
 * A0  - Audio input
 * A4 + A5 - Display
 */

#include <fix_fft.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

// #define FREE_RUN_MODE

#define SCR_ROTATION  2

#define SAMPLES       128
#define FRAME_TIME    63       // Desired time per frame in ms (63 is ~15 fps)

#define COL_XRES      6        // Total number of  columns in the display, must be <= SAMPLES/2
#define COL_YRES      63       // Total number of  rows in the display
#define COL_WIDTH     21
#define COL_DECAY     2

#define LD_COUNT      5
#define LD_FLASH      0
#define LD_ERROR      4
#define LD_DECAY_REL  4
#define LD_MIN_DECAY  1
#define LD_MAX_DECAY  100

#define ST_TRIGGER_LEDS 4
#define ST_START      80
#define ST_INCREMENT  20
#define ST_DECAY      10

#define AMP_MAX_MULT  5
#define AMP_MAX_LEVEL 64
#define AMP_RELEASE   .02

const uint8_t LED[] = { 12, 11, 10, 9, 6 }; // PWD needed
const uint8_t LED_THRESHOLD[] = { 8, 14, 10, 14, 14  };

#ifdef FREE_RUN_MODE
const uint8_t FREQ_SPLITS[][2] { 
   { 0, 1 },
   { 1, 3 },
   { 3, 5 },
   { 5, 10 },
   { 10, 32 }  
};
#else
const uint8_t FREQ_SPLITS[][2] { 
   { 0, 1 },
   { 1, 5 },
   { 5, 10 },
   { 10, 18 },
   { 18, 64 }  
};
#endif

// FPS control
double delta = 1;
double lastFrameTime = 0;

double led_values[LD_COUNT] = { 0, 0, 0, 0, 0 };

int8_t vReal[SAMPLES];
int8_t vImag[SAMPLES];

uint8_t decay = 1;
uint8_t peaks[COL_XRES];
uint8_t maxPeak = 64;
uint8_t strobe = 0;
double amplify = 1;

bool strobePos = 0;

// Initialize screen. Following line is for OLED 128x64 connected by I2C
Adafruit_SSD1306 display(128, 64, &Wire, -1);

void fps() {
  while (millis() - lastFrameTime < FRAME_TIME);
  delta = (double)(millis() - lastFrameTime) / FRAME_TIME;
  lastFrameTime = millis();
}
 
void setup() {
  // Setup leds
  for (uint8_t i=0; i<LD_COUNT; i++) {
    pinMode(LED[i], OUTPUT);
    digitalWrite(LED[i], HIGH);
  }

  delay(500);
  // Reset leds
  for (uint8_t i=0; i<LD_COUNT; i++) {
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
  char data_avgs[COL_XRES];
  char led_avgs[LD_COUNT] = { 0, 0, 0, 0, 0 };
  
  // ++ Sampling
  uint8_t inputPeak = 0;
  for(uint8_t i=0; i<SAMPLES; i++) {
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
  // -- Sampling
  
  display.clearDisplay();

  double ampPeak = inputPeak * amplify;
  if (ampPeak < AMP_MAX_LEVEL) {
    if (amplify < AMP_MAX_MULT) {
      amplify += AMP_RELEASE; 
    }
  } else if (ampPeak > AMP_MAX_LEVEL) {
    amplify = (double) AMP_MAX_LEVEL / inputPeak;
  }

  // amplify
  for(int i=0; i<SAMPLES; i++) {
    vReal[i] = (double) amplify * vReal[i];
    
    // draw osc 
    display.drawPixel(i*2, 32 + vReal[i] / 4, 1);
  }
  
  // ++ FFT
  fix_fft(vReal, vImag, 7, 0);   
  
  int8_t fundamental = -1;
  for (byte i = 0; i < 32; i++) {
    vReal[i] = sqrt(vReal[i] * vReal[i] + vImag[i] * vImag[i]); // Make values positive
    if (fundamental == -1 || vReal[fundamental] < vReal[i]) {
      fundamental = i;
    }
  }
  // -- FFT
  
  // ++ re-arrange FFT result to match with no. of columns on display ( COL_XRES )
  int step = 64 / COL_XRES; 
  char c=0;
  for(char i=0; i<64; i+=step)  
  {
    data_avgs[c] = 0;
    for (char k=0 ; k<step; k++) {
      char t = i + k;
      data_avgs[c] = data_avgs[c] + vReal[t];
  
      // Leds
      for (char j=0; j<LD_COUNT; j++) {
        if (t >= FREQ_SPLITS[j][0] && t < FREQ_SPLITS[j][1]) {
          led_avgs[j] = led_avgs[j] + vReal[t];
        }
      }
    }
    data_avgs[c] = data_avgs[c]/step; 
    c++;
  }
  // -- re-arrange FFT result to match with no. of columns on display ( COL_XRES )
  
  // ++ send to display according measured value 
  
  // draw fft
  for(char i = 0; i < COL_XRES; i++)
  {
    data_avgs[i] = constrain(data_avgs[i],0,16);              // set max & min values for buckets
    data_avgs[i] = map(data_avgs[i], 0, 16, 0, COL_YRES);     // remap averaged values to COL_YRES
    int8_t yvalue=data_avgs[i];
  
    peaks[i] = max(0, (double) peaks[i] - COL_DECAY * delta);    // decay 
    if (yvalue > peaks[i]) {
      peaks[i] = yvalue;
    }
    yvalue = peaks[i];    
  
    // draw a bar
    display.drawRect(i * COL_WIDTH, 63 - yvalue, COL_WIDTH -1, 1 + yvalue, 1);
  
    maxPeak = max(yvalue, maxPeak);
    maxPeak--;
  }

  // Flash only turns on when increases
  bool flash_on = led_avgs[LD_FLASH] - led_values[LD_FLASH] > LED_THRESHOLD[LD_FLASH];

  // high energy
  bool high_energy = led_avgs[LD_FLASH] > LED_THRESHOLD[LD_FLASH];
  
  // led control
  uint8_t leds_on = 0;
  for (uint8_t j=0; j<LD_COUNT; j++) {
    bool on = j == LD_FLASH 
      ? flash_on 
      : led_avgs[j] > LED_THRESHOLD[j];

    //if (j == LD_FLASH) {
    if (high_energy) {
      // High energy. Fast leds
      led_values[j] = flash_on ? 255 : 0;
    } else {
      if (on) {
        led_values[j] = min(led_avgs[j], (double) led_values[j] + decay * delta);
        leds_on++;
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
  if (leds_on >= ST_TRIGGER_LEDS) {
    strobe = min(250, (double) strobe + ST_INCREMENT * delta);
  } else {
    strobe = max(0, (double) strobe - ST_DECAY * delta);
  }

  if (strobe > ST_START) {
    digitalWrite(LED[LD_FLASH], strobePos);
    strobePos = !strobePos;
  }

  // peak line
  display.drawFastHLine(64, 0, ampPeak / AMP_MAX_LEVEL * 64, 1);

  // amp line
  display.drawFastHLine(64, 2, amplify / AMP_MAX_MULT * 64, 1);

  // show energy line (based on decay)
  display.drawFastHLine(64, 4, (double) decay / LD_MAX_DECAY * 64, 1);

  // show strobe line
  display.drawFastHLine(64, 6, (double) strobe / 256 * 64, 1);
  display.drawPixel(64 + (double) ST_START / 256 * 64, 6, 1);
  
  display.display();
  // -- send to display according measured value

  fps();
} 
