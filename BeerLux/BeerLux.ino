#include <FastLED.h>

// FHT settings
#define LOG_OUT          1 // use the log output function
#define FHT_N            256 // set to 256 point fht
#define FHT_MAG_MIN      96
#define FHT_MAG_MAX      128
#include <FHT.h>

// Serial output
//#define FHT_OUT
#define DEBUG


// HW SPI: Pin 13 = CLK, Pin 11 = MOSI (DATA)
#define PIN_LED_DATA     11       // Yellow wire on Adafruit Pixels
#define PIN_LED_CLK      13       // Green wire on Adafruit Pixels
#define PIN_INT          2
#define PIN_AUDIO_IN     A0

// Globals
#define NUM_LEDS            24
#define HUE_STEP            255 / NUM_LEDS * 2
#define SENSITIVITY_MAX     5
#define SENSITIVITY_MIN     1

CRGBArray<NUM_LEDS> leds;
uint16_t sensitivity = SENSITIVITY_MIN;

void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("setup");
#endif

  pinMode(PIN_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_INT), interrupt, CHANGE);
  
  FastLED.addLeds<WS2801, PIN_LED_DATA, PIN_LED_CLK, RGB>(leds, NUM_LEDS);

  fillRainbow(25);
  delay(250);
  
  // Turn off
  fadeAll(25);

  // Optimize reading audio from ADC
//  TIMSK0 = 0;     // turn off timer0 for lower jitter
  ADCSRA = 0xe5;  // set the adc to free running mode
  ADMUX = 0x40;   // use adc0
  DIDR0 = 0x01;   // turn off the digital input for adc0
}

void loop()
{
  calcFHT();
  updateLEDs();
}

void interrupt() {
  static unsigned long allowInterrupt = 0;
  unsigned long now = millis();
  
  if (now > allowInterrupt) {
    allowInterrupt = now + 500;
    incSensitivity();
  }
}

void incSensitivity() {
  sensitivity++;
  if (sensitivity > SENSITIVITY_MAX)
    sensitivity = SENSITIVITY_MIN;
#ifdef DEBUG
  Serial.println("");
  Serial.print("sensitivity=");
  Serial.println(sensitivity);
#endif
}

void fillRainbow(uint32_t wait) {
  int s=255, v=255;
  for (int i=0, h=0; i < FastLED.size()/2; i++, h += HUE_STEP) {
    CHSV color = CHSV(h, s, v);
    leds[i] = color;
    leds[FastLED.size() - i - 1] = color;
    FastLED.show(); 
    delay(wait);
  }  
}

void updateLEDs() {
  float h=0.0, s=255, v=0.0;
  int st, ed, avg;  
  int avgWindow = (0x1 << sensitivity);
  
  for (int i=0; i < FastLED.size()/2; i++, h += HUE_STEP) {
    st = i * avgWindow;
    ed = st + avgWindow;
    avg = 0, v = 0;
    
    for (int j=st; j<ed; j++) {
      if (j==0 || j==1) { // First 2 values are always too big.
        avg += fht_log_out[j] / 3;
      }
      else {
        avg += fht_log_out[j];
      }
    }
    avg /= avgWindow;

    if ( avg > FHT_MAG_MIN ) {
      v = map(avg, FHT_MAG_MIN, FHT_MAG_MAX, 0, 255);
    }
    
#ifdef DEBUG 
    Serial.print(i);
    Serial.print(", ");
    Serial.print(sensitivity);
    Serial.print(", ");
    Serial.print(avgWindow);
    Serial.print(", ");
    Serial.print(st);
    Serial.print(":");
    Serial.print(ed);
    Serial.print(", ");
    Serial.print(avg);
    Serial.print(", ");
    Serial.println(v);
#endif    

    uint8_t idx = 1;
    leds[i].setHSV(h, s, v);

    idx = FastLED.size() - i - 1;
    leds[idx].setHSV(h, s, v);
  }

  FastLED.show();
}

void fadeAll(uint32_t wait) { 
  for(int i=0; i<20; i++) { 
    for(int j=0; j<FastLED.size(); j++) { 
      leds[j].nscale8(5);
    } 
    FastLED.show();
    delay(wait);
  }
  
  FastLED.clear();
}

void calcFHT() {
  cli();  // UDRE interrupt slows this way down on arduino1.0
  for (int i = 0 ; i < FHT_N ; i++) { // save 256 samples
    while(!(ADCSRA & 0x10)); // wait for adc to be ready
    ADCSRA = 0xf5; // restart adc
    byte m = ADCL; // fetch adc data
    byte j = ADCH;
    int k = (j << 8) | m; // form into an int
    k -= 0x0200; // form into a signed int
    k <<= 6; // form into a 16b signed int
    fht_input[i] = k; // put real data into bins
  }
  fht_window(); // window the data for better frequency response
  fht_reorder(); // reorder the data before doing the fht
  fht_run(); // process the data in the fht
  fht_mag_log(); // take the output of the fht
  sei();
  
#ifdef FHT_OUT
  Serial.write(255); // send a start byte
  Serial.write(fht_log_out, FHT_N/2); // send out the data
#endif
}  


