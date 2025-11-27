#include <Arduino.h>
#include <Ticker.h>
#include <SPIFFS.h>
#include <OpenMRNLite.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <ADS1115_WE.h>

#include "config.h"
#include "NODEID.h"
#include "RGBWStrip.h"

static constexpr openlcb::ConfigDef cfg(0);
static constexpr uint8_t NUM_RGBW_STRIPS = openlcb::NUM_RGBW_STRIPS;

namespace openlcb {
// Define SNIP_STATIC_DATA (declared in config.h)
const SimpleNodeStaticValues SNIP_STATIC_DATA = {
    4,
    "OpenMRN",
    "LCC RGBW Lighting Controller",
    ARDUINO_VARIANT,
    "1.0.0"
};

const char CDI_FILENAME[] = "/spiffs/cdi.xml";
extern const char CDI_DATA[] = "";
extern const char *const CONFIG_FILENAME = "/spiffs/openlcb_config";
extern const size_t CONFIG_FILE_SIZE = cfg.seg().size() + cfg.seg().offset();
extern const char *const SNIP_DYNAMIC_FILENAME = CONFIG_FILENAME;
}

ADS1115_WE adc = ADS1115_WE();
Esp32HardwareTwai twai(D8, D9);
OpenMRN openmrn(NODE_ID);

openlcb::RGBWStrip *rgbwStrips[NUM_RGBW_STRIPS];

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("\n\n=== LCC RGBW Lighting Controller ===");
  Serial.printf("Node ID: 0x%012llX\n", NODE_ID);

  // Initialize SPIFFS
  if (!SPIFFS.begin()) {
    Serial.println("SPIFFS failed to mount, formatting...");
    if (!SPIFFS.begin(true)) {
      Serial.println("SPIFFS mount failed. Halting.");
      while (1);
    }
  }

  openmrn.create_config_descriptor_xml(cfg, openlcb::CDI_FILENAME);
  
  // Check if we need to create/initialize config file
  bool configExists = SPIFFS.exists(openlcb::CONFIG_FILENAME);
  
  openmrn.stack()->create_config_file_if_needed(
    cfg.seg().internal_config(),
    openlcb::CANONICAL_VERSION,
    openlcb::CONFIG_FILE_SIZE);
  
  // If config was just created, write default event IDs
  if (!configExists) {
    Serial.println("New config file created, initializing event IDs...");
    int fd = ::open(openlcb::CONFIG_FILENAME, O_RDWR);
    if (fd >= 0) {
      cfg.seg().rgbw_strips().entry(0).red_event().write(fd, openlcb::RGBW_EVENT_INIT[0]);
      cfg.seg().rgbw_strips().entry(0).green_event().write(fd, openlcb::RGBW_EVENT_INIT[1]);
      cfg.seg().rgbw_strips().entry(0).blue_event().write(fd, openlcb::RGBW_EVENT_INIT[2]);
      cfg.seg().rgbw_strips().entry(0).white_event().write(fd, openlcb::RGBW_EVENT_INIT[3]);
      cfg.seg().rgbw_strips().entry(0).brightness_event().write(fd, openlcb::RGBW_EVENT_INIT[4]);
      ::close(fd);
    }
  }

  // Initialize ADS1115 (controller only, but harmless if not populated)
  if (!adc.init()) {
    Serial.println("ADS1115 not detected - will run as FOLLOWER");
  } else {
    adc.setVoltageRange_mV(ADS1115_RANGE_4096);
    adc.setCompareChannels(ADS1115_COMP_0_GND);
    adc.setConvRate(ADS1115_250_SPS);
    adc.setMeasureMode(ADS1115_CONTINUOUS);
    Serial.println("ADS1115 detected - can run as CONTROLLER");
  }

  // Create RGBW strip controllers
  for (uint8_t i = 0; i < NUM_RGBW_STRIPS; i++) {
    rgbwStrips[i] = new openlcb::RGBWStrip(
      openmrn.stack()->node(),
      cfg.seg().rgbw_strips().entry(i),
      &adc
    );
  }

  // Initialize OpenMRN stack
  openmrn.begin();
  openmrn.start_executor_thread();
  
  // Initialize CAN/TWAI
  twai.hw_init();
  openmrn.add_can_port_select("/dev/twai/twai0");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println("=== Initialization Complete ===\n");
}

void loop() {
  openmrn.loop();

  // Controller: Poll ADC inputs every 10ms (matches hmiExample.ino timing)
  static unsigned long lastPoll = 0;
  if (millis() - lastPoll >= 10) {
    for (uint8_t i = 0; i < NUM_RGBW_STRIPS; i++) {
      rgbwStrips[i]->poll_adc_inputs();
    }
    lastPoll = millis();
  }

  // Heartbeat LED
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink >= 1000) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    lastBlink = millis();
  }
}
