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

openlcb::RGBWStrip *rgbwStrip = nullptr;
bool isController = false;

// Track when to start the fade animation
static unsigned long initCompleteTime = 0;
static bool fadeStarted = false;


void setup() {
  Serial.begin(115200);
  Wire.begin();

  delay(1000);

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
  
  // Create config file if needed and check if factory reset is required
  Serial.printf("Checking config file (expecting version 0x%04X)...\n", openlcb::CANONICAL_VERSION);
  
  // Read current version before calling create_config_file_if_needed
  int fd_check = ::open(openlcb::CONFIG_FILENAME, O_RDONLY);
  uint16_t current_version = 0;
  bool needsFactoryReset = false;
  if (fd_check >= 0) {
    current_version = cfg.seg().internal_config().version().read(fd_check);
    ::close(fd_check);
    Serial.printf("Current stored version: 0x%04X\n", current_version);
    needsFactoryReset = (current_version != openlcb::CANONICAL_VERSION);
  } else {
    Serial.println("Config file does not exist yet");
    needsFactoryReset = true;
  }
  
  int config_fd = openmrn.stack()->create_config_file_if_needed(
    cfg.seg().internal_config(),
    openlcb::CANONICAL_VERSION,
    openlcb::CONFIG_FILE_SIZE);
  Serial.printf("Config check complete, fd=%d\n", config_fd);

  // Initialize ADS1115 (controller only, but harmless if not populated)
  if (!adc.init()) {
    isController = false;
    Serial.println("ADS1115 not detected - will run as FOLLOWER");
  } else {
    isController = true;
    adc.setVoltageRange_mV(ADS1115_RANGE_4096);
    adc.setCompareChannels(ADS1115_COMP_0_GND);
    adc.setConvRate(ADS1115_250_SPS);
    adc.setMeasureMode(ADS1115_CONTINUOUS);
    Serial.println("ADS1115 detected - can run as CONTROLLER");
  }

  // Create RGBW strip controller and initialize if needed
  rgbwStrip = new openlcb::RGBWStrip(
    openmrn.stack()->node(),
    cfg.seg().rgbw_strips().entry(0),
    &adc
  );
  
  // Only reset RGBW config when config file is new or version changed
  if (needsFactoryReset) {
    Serial.println("Initializing RGBW config defaults...");
    rgbwStrip->factory_reset(config_fd);
    Serial.println("RGBW config initialized");
  } else {
    Serial.println("Config file valid, preserving user settings");
  };

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

  // Record init complete time on first loop iteration
  if (initCompleteTime == 0) {
    initCompleteTime = millis();
  }

  // Controller: Start fade animation after configured delay (allows LCC bus to settle)
  if (isController && !fadeStarted) {
    unsigned long delayMs = rgbwStrip->startup_delay_sec() * 1000UL;
    if (millis() - initCompleteTime >= delayMs) {
      Serial.printf("Starting fade-in animation (after %d sec delay)...\n", rgbwStrip->startup_delay_sec());
      rgbwStrip->run_startup_animation();
      fadeStarted = true;
    }
  }

  // Controller: Poll ADC inputs every 10ms (only if this device is a controller)
  if (isController) {
    static unsigned long lastPoll = 0;
    if (millis() - lastPoll >= 10) {
      rgbwStrip->poll_adc_inputs();
      lastPoll = millis();
    }
  }

  // Heartbeat LED
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink >= 1000) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    lastBlink = millis();
  }
}
