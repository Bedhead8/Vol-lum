// A basic everyday NeoPixel strip test program.

// NEOPIXEL BEST PRACTICES for most reliable operation:
// - Add 1000 uF CAPACITOR between NeoPixel strip's + and - connections.
// - MINIMIZE WIRING LENGTH between microcontroller board and first pixel.
// - NeoPixel strip's DATA-IN should pass through a 300-500 OHM RESISTOR.
// - AVOID connecting NeoPixels on a LIVE CIRCUIT. If you must, ALWAYS
//   connect GROUND (-) first, then +, then data.
// - When using a 3.3V microcontroller with a 5V-powered NeoPixel strip,
//   a LOGIC-LEVEL CONVERTER on the data line is STRONGLY RECOMMENDED.
// (Skipping these may work OK on your workbench but can fail in the field)
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1:
#define LED_PIN    6

// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 8
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
//--------------------------------------------------------
#include <HID-Project.h>                    //include HID_Project library
#include <HID-Settings.h>

#define REVERSED false                      //if your controller is reversed change it to true
int val = 0;
int previousval = 0;
int val2 = 0;

const int PinA = 0;                     // Used for generating interrupts using CLK signal

const int PinB = 1;                     // Used for reading DT signal

const int PinSW = 2;                    // Used for the push button switch

int lastCount = 50;                     // Keep track of last rotary value

volatile int virtualPosition = 50;      // Updated by the ISR (Interrupt Service Routine)

// ------------------------------------------------------------------
// INTERRUPT     INTERRUPT     INTERRUPT     INTERRUPT     INTERRUPT
// ------------------------------------------------------------------
void isr ()  {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();

  if (interruptTime - lastInterruptTime > 5)      // If interrupts come faster than 5ms, assume it's a bounce and ignore
  {
    if (digitalRead(PinB) == LOW)
    {
      virtualPosition-- ; // Could be -5 or -10
    }
    else {
      virtualPosition++ ; // Could be +5 or +10
    }

    // Restrict value from 0 to +100
    //virtualPosition = min(100, max(0, virtualPosition));

  }
  
  lastInterruptTime = interruptTime;        // Keep track of when we were here last (no more than every 5ms)
}

// ------------------------------------------------------------------
// SETUP    SETUP    SETUP    SETUP    SETUP    SETUP    SETUP
// ------------------------------------------------------------------
void setup() {
  
  
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(20); // Set BRIGHTNESS to about 1/5 (max = 255)  
  //colorWipe(strip.Color(255,   0,   0), 50);
  rainbow(10); 
  
  // Just whilst we debug, view output on serial monitor
  Serial.begin(115200);

  // Rotary pulses are INPUTs
  pinMode(PinA, INPUT_PULLUP);
  pinMode(PinB, INPUT_PULLUP);

  pinMode(PinSW, INPUT_PULLUP);          // Switch is floating so use the in-built PULLUP so we don't need a resistor

  attachInterrupt(digitalPinToInterrupt(PinA), isr, LOW);     // Attach the routine to service the interrupts

  Serial.println("Start");     // Ready to go!


  Consumer.begin();                         //initialize computer connection
  delay(1000);                              //wait for computer to connect
    //for(int a = 0; a < 52; a++) {
    // Consumer.write(MEDIA_VOLUME_DOWN);      //set the volume to 0
   // delay(2);
   //   }

  
}

// ------------------------------------------------------------------
// MAIN LOOP     MAIN LOOP     MAIN LOOP     MAIN LOOP     MAIN LOOP
// ------------------------------------------------------------------
void loop() {

  // Is someone pressing the rotary switch?
  if ((!digitalRead(PinSW))) {
    virtualPosition = 50;
    while (!digitalRead(PinSW))
      delay(10);
    Serial.println("Reset");
  }

  // If the current rotary switch position has changed then update everything
  if (virtualPosition != lastCount) {

    // Write out to serial monitor the value and direction
    if (virtualPosition > lastCount) Consumer.write(MEDIA_VOLUME_DOWN);
    if (virtualPosition < lastCount) Consumer.write(MEDIA_VOLUME_UP); 
    
    Serial.println(virtualPosition);

    // Keep track of this new value
    lastCount = virtualPosition ;
  }

  //delay(100);
}

//--------------------------------------------------------------------------------------------------------------------


void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}

void rainbow(int wait) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this outer loop:
  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
      // Offset pixel hue by an amount to make one full revolution of the
      // color wheel (range of 65536) along the length of the strip
      // (strip.numPixels() steps):
      int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
      // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
      // optionally add saturation and value (brightness) (each 0 to 255).
      // Here we're using just the single-argument hue variant. The result
      // is passed through strip.gamma32() to provide 'truer' colors
      // before assigning to each pixel:
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
    }
    strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}
