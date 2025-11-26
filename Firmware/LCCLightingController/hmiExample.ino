#include <ADS1115_WE.h> 
#include <Adafruit_NeoPixel.h>
#include <Wire.h>

#define LED_PIN     D10
#define LED_COUNT   120

ADS1115_WE adc = ADS1115_WE();
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_WRGB + NEO_KHZ800);

// Store current 8-bit color values
uint8_t currentRed   = 0;
uint8_t currentGreen = 0;
uint8_t currentBlue  = 0;
uint8_t currentWhite = 0;

// For round-robin channel selection
const uint8_t NUM_CHANNELS = 4;
ADS1115_MUX channels[NUM_CHANNELS] = {
  ADS1115_COMP_0_GND,
  ADS1115_COMP_1_GND,
  ADS1115_COMP_2_GND,
  ADS1115_COMP_3_GND
};

void setup() {
  Wire.begin();
  Serial.begin(9600);

  if (!adc.init()) {
    Serial.println("ADS1115 not connected!");
  }

  adc.setVoltageRange_mV(ADS1115_RANGE_4096);
  adc.setCompareChannels(ADS1115_COMP_0_GND);
  adc.setConvRate(ADS1115_250_SPS);
  adc.setMeasureMode(ADS1115_CONTINUOUS);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  strip.begin();
  strip.setBrightness(255);

  // Initialize LEDs to off
  strip.fill(strip.Color(0, 0, 0, 0));
  strip.show();
}

void loop() {
  static uint8_t channelIndex = 0;  // which channel to read this iteration
  bool colorChanged = false;

  // Read just one channel this loop
  ADS1115_MUX ch = channels[channelIndex];
  float voltage = readChannel(ch);  // mV, absolute

  // Map 0–3300 mV to 0–255 (clamp first for safety)
  long clamped = voltage;
  if (clamped < 0)    clamped = 0;
  if (clamped > 3300) clamped = 3300;

  uint8_t mapped = map(clamped, 0, 3300, 0, 255);

  // Assign the mapped value to the corresponding color
  switch (channelIndex) {
    case 0: // Red
      Serial.print("Ch 0 (Red) mV: ");
      Serial.println(voltage);
      if (mapped != currentRed) {
        currentRed = mapped;
        colorChanged = true;
      }
      break;

    case 1: // Green
      Serial.print("Ch 1 (Green) mV: ");
      Serial.println(voltage);
      if (mapped != currentGreen) {
        currentGreen = mapped;
        colorChanged = true;
      }
      break;

    case 2: // Blue
      Serial.print("Ch 2 (Blue) mV: ");
      Serial.println(voltage);
      if (mapped != currentBlue) {
        currentBlue = mapped;
        colorChanged = true;
      }
      break;

    case 3: // White
      Serial.print("Ch 3 (White) mV: ");
      Serial.println(voltage);
      if (mapped != currentWhite) {
        currentWhite = mapped;
        colorChanged = true;
      }
      break;
  }

  // Only update the strip if any color actually changed
  if (colorChanged) {
    strip.fill(strip.Color(currentRed, currentGreen, currentBlue, strip.gamma8(currentWhite)));
    strip.show();
  }

  // Next loop will read the next channel
  channelIndex = (channelIndex + 1) % NUM_CHANNELS;

  delay(10);
}

float readChannel(ADS1115_MUX channel) {
  adc.setCompareChannels(channel);
  float voltage = adc.getResult_mV(); 
  return abs(voltage);
}
