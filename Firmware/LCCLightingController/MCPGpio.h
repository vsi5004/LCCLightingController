#ifndef __MCPGPIO_HXX
#define __MCPGPIO_HXX

#include <Arduino.h>
#include <Ticker.h>
#include <Adafruit_MCP23X17.h>
#include "os/Gpio.hxx"

// Define hardware-level macros if not already defined.
#ifndef ARDUINO_HIGH
#define ARDUINO_HIGH 1
#endif
#ifndef ARDUINO_LOW
#define ARDUINO_LOW 0
#endif
#ifndef ARDUINO_OUTPUT
#define ARDUINO_OUTPUT 3
#endif
#ifndef ARDUINO_INPUT
#define ARDUINO_INPUT 1
#endif

/**
 * @brief MCPGpio implements the Gpio interface for an MCP23X17.
 *
 * This version takes two pin numbers (even and odd) so that you can control frog
 * polarity by activating one pin for one polarity and the other for the opposite polarity.
 * A Ticker is used to automatically deactivate the pins after a short pulse.
 */
class MCPGpio : public Gpio {
public:
  /**
   * @brief Construct a new MCPGpio object.
   *
   * @param mcp Pointer to the Adafruit_MCP23X17 instance.
   * @param evenPin The pin number used for one polarity.
   * @param oddPin The pin number used for the opposite polarity.
   */
  MCPGpio(Adafruit_MCP23X17 *mcp, uint8_t evenPin, uint8_t oddPin, bool invertFrog)
    : mcp(mcp),
          evenPin_(evenPin),
          oddPin_(oddPin),
          currentDirection_(Gpio::Direction::DOUTPUT),
          invertFrog_(invertFrog)
  {
    applyFrogInversion();
  }

  void set_frog_inverted(bool invertFrog) {
        if (invertFrog_ != invertFrog) {
            invertFrog_ = invertFrog;
            applyFrogInversion();
        }
    }
  
  // Write a new value (SET or CLR) to the pin.
  virtual void write(Gpio::Value new_state) const override {
    if (new_state == Gpio::SET) {
      activatePin(evenPin_);
    } else {
      activatePin(oddPin_);
    }
  }
  
  // Read the current value from the "even" pin.
  virtual Gpio::Value read() const override {
    int state = mcp->digitalRead(evenPin_);
    return (state == ARDUINO_HIGH) ? Gpio::SET : Gpio::CLR;
  }
  
  // Convenience: set the pin (drive it high).
  virtual void set() const override {
    write(Gpio::SET);
  }
  
  // Convenience: clear the pin (drive it low).
  virtual void clr() const override {
    write(Gpio::CLR);
  }
  
  // Set the pin direction.
  virtual void set_direction(Gpio::Direction dir) const override {
    currentDirection_ = dir;
    mcp->pinMode(evenPin_, ARDUINO_OUTPUT);
    mcp->pinMode(oddPin_, ARDUINO_OUTPUT);
  }
  
  // Get the current pin direction.
  virtual Gpio::Direction direction() const override {
    return currentDirection_;
  }
  
private:
  Adafruit_MCP23X17 *mcp;
  uint8_t evenPin_, oddPin_;
  mutable Gpio::Direction currentDirection_;
  mutable Ticker ticker;
  bool invertFrog_;

  void applyFrogInversion() {
        if (invertFrog_) {
            std::swap(evenPin_, oddPin_);
        }
    }
  
  // Activate a given pin by setting it high, then schedule a deactivation.
  void activatePin(uint8_t pin) const {
    mcp->digitalWrite(pin, ARDUINO_HIGH);
    ticker.once(0.3f, static_callback, (void*)this);
  }
  
  // Static callback for the Ticker.
  static void static_callback(void* arg) {
    MCPGpio* self = static_cast<MCPGpio*>(arg);
    self->deactivatePins();
  }
  
  // Deactivate both pins.
  void deactivatePins() const {
    mcp->digitalWrite(evenPin_, ARDUINO_LOW);
    mcp->digitalWrite(oddPin_, ARDUINO_LOW);
  }
};

#endif // MCPGPIO_HXX
