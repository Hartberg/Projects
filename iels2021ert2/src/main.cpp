/*
 * INFO:
 * This code demonstrates how to use the ESP32 DAC in an interrupt based configuration.
 * The sampling frequency and resolution of the samples and the DAC can be adjusted and experimented with
 *
 * Note: the ESP32 DAC has a fixed resolution of 8 bits, so the possibility for varying
 * DAC resolution is simulated by representing the desired resolution within an 8 bit range.
 */
 
/*
 * Global constants for the array of samples to be written to the DAC.
 * Default is a sine wave of 16 values between 0 and 255.
 */
const int sampleArrayLen = 16;
const int sampleArray[sampleArrayLen] = {
  0x80,
  0xb0,
  0xda,
  0xf5,
  0xff,
  0xf5,
  0xda,
  0xb0,
  0x80,
  0x4f,
  0x25,
  0xa,
  0x0,
  0xa,
  0x25,
  0x4f,
};
 
//Change these parameters in order to change the sampling properties of the system
#define SAMPLING_PERIOD 100  //In microseconds, should be at least 50.
#define DAC_RESOLUTION 8     //Resolution of the DAC. Must be between 1 and 8 bits.
 
//Hardware configuration
#define DAC_PIN 25
 
// Variable for storing the timer properties. Leave this alone.
hw_timer_t *timer = NULL;
 
// Constant with a mask for setting the correct DAC resolution. Leave this alone.
const int resolution_mask = ~((1 << (8 - DAC_RESOLUTION)) - 1);
 
/* 
 * Interrupt service routine that runs every with a period SAMPLING_PERIOD. 
 * Used for sampling, signal processing and digital to analog conversion.
 */
void ARDUINO_ISR_ATTR onTimer() {
  //Variable for storing the actual value to be written to the DAC.
  static int sampleDAC;
 
  //Current index of the sampleArray. Defined as static as its only used in this function.
  static int i = 0;
 
  //Using the DAC to convert and write the sampleDAC variable on DAC_PIN with the given resolution for the DAC.
  dacWrite(DAC_PIN, (sampleArray[i] & resolution_mask));
 
  if (i >= sampleArrayLen - 1) {
    //Reached end of sampleArray. Resetting i.
    i = 0;
  } else {
    //Incrementing i.
    i++;
  }
}
 
//--------The code below this point can be left alone--------
 
void setup() {
  //Initializing timer with 1Mhz frequency. This means one tick every microsecond.
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &onTimer);
 
  //Call the onTimer function with the given period (in microseconds), repeating te timer and with unlimited count.
  timerAlarm(timer, SAMPLING_PERIOD, true, 0);
}
 
void loop() {
  //empty loop
}
 