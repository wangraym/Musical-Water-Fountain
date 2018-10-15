/* MIE 438 Project
   Water fountain auidio visualizer
   
   Currently uses arduinoFFT library:
   -improve by using integers instead of floating point
   -can copy library functions into code instead of importing entire library (to reduce size of code)
   -tune thresholds, frequency bands, and sample rate/buffersize
   
   disable serial commands when done debugging*/

// Threshhold FFT must pass to turn solenoid on
#define lowThreshold 1500
#define midThreshold 1500
#define highThreshold 1500

// Desired Frequency Bands (Hz)
#define F1L 0
#define F1H 230
#define F2L 300
#define F2H 950
#define F3L 1000
#define F3H 2000

//bin index (values set in code)
int B1L = 0;
int B1H = 0;
int B2L = 0;
int B2H = 0;
int B3L = 0;
int B3H = 0;

// digital pins
#define solenoid1 4
#define solenoid2 5
#define solenoid3 6

// DC offset from Mic (input voltage / 2) *1023 / 5V
#define OFFSET 512


// FFT Code---------------------
#include "arduinoFFT.h"

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
const uint16_t samples = 128; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 9600;  // not exact

double vReal[samples]; //array that stores ADC values and contains FFT spectrum after analysis
double vImag[samples]; //used in calculations of FFT

// ADC configuration code-------
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))


void setup() {

  // ADC setup

  // ADC Register settings explained by http://maxembedded.com/2011/06/the-adc-of-the-avr/
  // ------------------------------------------ 
  // set the adc to free running mode
  // register explanation: 
  // 5 => div 32. sample rate 38.4
  // 7 => switch to divider=128, default 9.6khz sampling
  ADCSRA = 0xe0 + 7; // "ADC Enable", "ADC Start Conversion", "ADC Auto Trigger Enable" and divider.
  ADMUX = 0x0; // use a0 pin
  ADMUX |= 0x40; // Use Vcc for analog reference.
  DIDR0 = 0x01; // turn off the digital input for adc0
  analogReference(DEFAULT); // 5V default reference
  // -------------------------------------------

  // serial setup
  Serial.begin(9600);
  //while (!Serial); // Wait untilSerial is ready - Leonardo
  Serial.println("Starting");

  // initialize frequency bands
  int binWidth = (int)samplingFrequency / samples;
  int iMax = samples/2 -1;
  
  B1L = F1L / binWidth;
  B1L = constrain(B1L,0, iMax-5);
  B1H = F1H / binWidth;
  B1H = constrain(B1H,B1L+1, iMax-4);
  
  B2L = F2L / binWidth;
  B2L = constrain(B2L,B1H+1, iMax-3);
  B2H = F2H / binWidth;
  B2H = constrain(B2H,B2L+1, iMax-2);
  
  B3L = F3L / binWidth;
  B3L = constrain(B3L,B2H+1, iMax-1);
  B3H = F3H / binWidth;
  B3H = constrain(B3H,B3L+1, iMax);

}



void loop() {
  //helps with timing
  while (1) {
    MeasureAnalog();
    //printArray("buffer", samples);
    applyFFT();
    //printArray("\nFFT", samples / 2);
    //double fpeak = FFT.MajorPeak(vReal, samples, samplingFrequency); //can check Frequency of largest peak
    //Serial.println(fpeak);

    //vReal currently contains frequency distriibution, must control solenoids
    
    // low frequency band
    if(maxInRange(B1L,B1H) > lowThreshold){ //maxInRange checks vReal for max value between two indecies
      digitalWrite(solenoid1, HIGH);
      Serial.print("ON      "); //temperary visualzer over serial
    }
    else{
      digitalWrite(solenoid1, LOW);
      Serial.print("        ");
    }

    // middle freq band
    if(maxInRange(B2L,B2H) > midThreshold){ //maxInRange checks vReal for max value between two indecies
      digitalWrite(solenoid2, HIGH);
      Serial.print("ON      ");
    }
    else{
      digitalWrite(solenoid2, LOW);
      Serial.print("        ");
    }


    // high freq band
    if(maxInRange(B3L,B3H) > highThreshold){ //maxInRange checks vReal for max value between two indecies
      digitalWrite(solenoid3, HIGH);
      Serial.println("ON      |");
    }
    else{
      digitalWrite(solenoid3, LOW);
      Serial.println("        |");
    }
  }
}

void MeasureAnalog() {
  
  //cli();  // UDRE interrupt slows this way down on arduino1.0

  for (int i = 0; i < samples; i++) {

    //measure from ADC. Code taken from https://github.com/ayavilevich/ArduinoSoundLevelMeter
    //---------------------------------------------------
    //ADMUX = 0x0;
    while (!(ADCSRA & /*0x10*/_BV(ADIF))); // wait for adc to be ready (ADIF)
    sbi(ADCSRA, ADIF); // restart adc
    byte m = ADCL; // fetch adc data
    byte j = ADCH;
    int k = ((int)j << 8) | m; // form into an int
    //---------------------------------------------------

    //Buffer storage
    vReal[i] = k-OFFSET; //stores ADC values into FFT array
    vImag[i] = 0.0; //imaginary component must be set to 0 each loop

  }

  //sei();

}

void printArray(char* title, int numElements) {
  //function that prints each element of the vReal on a new line
  Serial.println(title);
  for (int i = 0; i < numElements; i++) {
    Serial.println(vReal[i]);
  }
  Serial.println();
}

void applyFFT() {
  // Apply FFT to vReal array
  // note that values are stored in first half or vReal after calculation (second half of array is mirrored)
  // frequency bin widths = sampling frequency / # samples

  //long tt = millis(); // can check how long FFT calculations take

  //FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING FFT_FORWARD);//windowing not required but imporves accuracy of frequency distribution
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes*/
  //to ignore DC offset
  vReal[0] = 0;
  vReal[1] *= 0.9; //bin 1 often too high, biased by 0Hz bin

  //Serial.println(millis()-tt);

}

int maxInRange(int start, int finish){
  int maxVal = 0;
  for(int i= start; i<=finish; i++){
    if(vReal[i] > maxVal)
      maxVal = vReal[i];
  }
//  Serial.println("Max val:");
//  Serial.print(start);Serial.println("    ");
//  Serial.println(maxVal);
  return maxVal;
}

