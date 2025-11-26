#include <Arduino.h>
#include <Ticker.h>

#undef HIGH
#undef LOW
#undef OUTPUT

#define ARDUINO_HIGH 1
#define ARDUINO_LOW 0
#define ARDUINO_OUTPUT 3
#define ARDUINO_INPUT 1

#include <SPIFFS.h>
#include <OpenMRNLite.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MCP23X17.h>

#include "config.h"
#include "NODEID.h"
#include "MCPGpio.h"
#include "ServoTurnout.h"
#include "PCAPwm.h"

static constexpr openlcb::ConfigDef cfg(0);
static constexpr uint8_t NUM_TURNOUTS = openlcb::NUM_TURNOUTS;
static constexpr uint32_t PWM_COUNT_PER_MS = 4096 / 20; // Assuming 20ms period (standard for servos)
static constexpr float SERVO_FREQ = 50; // Analog servos run at ~50 Hz updates

namespace openlcb {
const char CDI_FILENAME[] = "/spiffs/cdi.xml";
extern const char CDI_DATA[] = "";
extern const char *const CONFIG_FILENAME = "/spiffs/openlcb_config";
extern const size_t CONFIG_FILE_SIZE = cfg.seg().size() + cfg.seg().offset();
extern const char *const SNIP_DYNAMIC_FILENAME = CONFIG_FILENAME;
}

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Adafruit_MCP23X17 mcp;
Esp32HardwareTwai twai(D8, D9);
OpenMRN openmrn(NODE_ID);


constexpr uint8_t TURNOUT_FROG_PINS[NUM_TURNOUTS][2] = {
  { 8, 9 }, { 10, 11 }, { 12, 13 }, { 14, 15 }, { 0, 1 }, { 2, 3 }, { 4, 5 }, { 6, 7 }
};

PCAPwm *servoPwmWrappers[NUM_TURNOUTS];
MCPGpio *gpioWrappers[NUM_TURNOUTS];
openlcb::ServoTurnout *servoTurnouts[NUM_TURNOUTS];

void setup_frog_gpio() {
  if (!mcp.begin_I2C()) {
    Serial.println("Failed to initialize MCP23017!");
    while (1)
      ;
  }
  for (uint8_t i = 0; i < NUM_TURNOUTS; i++) {
    mcp.pinMode(TURNOUT_FROG_PINS[i][0], ARDUINO_OUTPUT);
    mcp.pinMode(TURNOUT_FROG_PINS[i][1], ARDUINO_OUTPUT);
    mcp.digitalWrite(TURNOUT_FROG_PINS[i][0], ARDUINO_LOW);
    mcp.digitalWrite(TURNOUT_FROG_PINS[i][1], ARDUINO_LOW);
  }
}

void setup() {
  Serial.begin(115200);

  if (!SPIFFS.begin()) {
    Serial.println("SPIFFS failed to mount, formatting...");
    if (!SPIFFS.begin(true)) {
      Serial.println("SPIFFS mount failed. Halting.");
      while (1)
        ;
    }
  }

  openmrn.create_config_descriptor_xml(cfg, openlcb::CDI_FILENAME);
  openmrn.stack()->create_config_file_if_needed(
    cfg.seg().internal_config(),
    openlcb::CANONICAL_VERSION,
    openlcb::CONFIG_FILE_SIZE);

  setup_frog_gpio();

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  for (uint8_t i = 0; i < NUM_TURNOUTS; i++) {
    gpioWrappers[i] = new MCPGpio(&mcp, TURNOUT_FROG_PINS[i][0], TURNOUT_FROG_PINS[i][1], false);
    servoPwmWrappers[i] = new PCAPwm(pwm, i);
    // Create the ServoTurnout object for each turnout.
    // Here we assume that cfg.turnouts[i] returns the TurnoutConfig for the i-th turnout.
    servoTurnouts[i] = new openlcb::ServoTurnout(openmrn.stack()->node(), cfg.seg().turnouts().entry(i), PWM_COUNT_PER_MS, servoPwmWrappers[i], gpioWrappers[i]);
  }

  openmrn.begin();
  openmrn.start_executor_thread();
  twai.hw_init();
  openmrn.add_can_port_select("/dev/twai/twai0");

  pinMode(LED_BUILTIN, ARDUINO_OUTPUT);
  digitalWrite(LED_BUILTIN, ARDUINO_LOW);
}

void loop() {
  openmrn.loop();
}
