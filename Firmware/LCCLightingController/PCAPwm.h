#ifndef __PCAPWM_HXX
#define __PCAPWM_HXX

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "freertos_drivers/arduino/PWM.hxx"

/**
 * @brief PCA9685Pwm provides a PWM interface for the Adafruit PCA9685 servo driver.
 * 
 * This class implements the PWM interface for up to 8 servo channels.
 */
class PCAPwm : public PWM {
public:
    /**
     * @brief Construct a new PCA9685Pwm object.
     * 
     * @param pwmDriver Reference to the Adafruit PWM Servo Driver.
     * @param channel The servo channel to control (0-7).
     */
    PCAPwm(Adafruit_PWMServoDriver &pwmDriver, uint8_t channel)
        : pwmDriver_(pwmDriver), channel_(channel) {}
    
    /**
     * @brief Set the PWM duty cycle for the servo.
     * 
     * @param duty PWM value in ticks (should be mapped to 0-4095 for PCA9685).
     */
    void set_duty(uint32_t duty) override {
        duty = constrain(duty, 0, 4095); // Ensure the value is within bounds
        pwmDriver_.setPWM(channel_, 0, duty);
        ticker.once(1.0f, static_callback, (void*)this);
    }

    /**
     * @brief Get the current PWM duty cycle.
     * 
     * @return uint32_t The last set PWM duty cycle.
     */
    uint32_t get_duty() override {
        return lastDutyCycle_;
    }

    /**
     * @brief Set the PWM period (not used for PCA9685, but needed for interface compliance).
     */
    void set_period(uint32_t counts) override {}

    /**
     * @brief Get the PWM period (not used for PCA9685, return a default value).
     */
    uint32_t get_period() override { return 4096; }

    /**
     * @brief Get the max supported period.
     */
    uint32_t get_period_max() override { return 4096; }

    /**
     * @brief Get the min supported period.
     */
    uint32_t get_period_min() override { return 1; }

    /**
     * @brief Disable the servo by turning off the PWM signal.
     */
    void disable() {
        pwmDriver_.setPWM(channel_, 0, 0); // Turns off the PWM signal
        lastDutyCycle_ = 0;
    }

private:
    Adafruit_PWMServoDriver &pwmDriver_; ///< Reference to the PCA9685 driver
    uint8_t channel_; ///< The servo channel being controlled
    uint32_t lastDutyCycle_ = 0; ///< Stores the last duty cycle value
    mutable Ticker ticker;

    // Static callback for the Ticker.
    static void static_callback(void* arg) {
      PCAPwm* self = static_cast<PCAPwm*>(arg);
      self->disable();
    }
};

#endif // __PCAPWM_HXX