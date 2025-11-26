#ifndef __SERVO_GPIO_BIT_HXX
#define __SERVO_GPIO_BIT_HXX

#include "os/Gpio.hxx"
#include "freertos_drivers/arduino/PWM.hxx"

namespace openlcb
{

/**
 * @brief An extended version of GPIOBit that also controls a servo.
 *
 * This class handles both:
 * - The frog relay (via GPIO)
 * - The turnout servo position (via PWM)
 */
class ServoGPIOBit : public BitEventInterface
{
public:
    /**
     * @brief Construct a new ServoGPIOBit object.
     *
     * @param node Pointer to the OpenMRN node.
     * @param event_on LCC event that sets the turnout to the normal position.
     * @param event_off LCC event that sets the turnout to the reversed position.
     * @param gpio Pointer to the frog relay GPIO.
     * @param pwm Pointer to the servo PWM output.
     * @param pwm_min_ticks PWM value (in ticks) for the normal position.
     * @param pwm_max_ticks PWM value (in ticks) for the reversed position.
     */
    ServoGPIOBit(Node *node, EventId event_on, EventId event_off, Gpio *gpio,
                 PWM *pwm, uint32_t pwm_min_ticks, uint32_t pwm_max_ticks)
        : BitEventInterface(event_on, event_off),
          node_(node),
          gpio_(gpio),
          pwm_(pwm),
          pwm_min_ticks_(pwm_min_ticks),
          pwm_max_ticks_(pwm_max_ticks)
    {
        Serial.println("ServoGPIOBit: Initialized");
    }

    /**
     * @brief Get the current event state (whether the turnout is set).
     *
     * @return EventState::VALID if the turnout is in the normal position,
     *         EventState::INVALID if in the reversed position.
     */
    EventState get_current_state() OVERRIDE
    {
        return gpio_->is_set() ? EventState::VALID : EventState::INVALID;
    }

    /**
     * @brief Set the turnout state (move the servo and switch the frog).
     *
     * @param new_value True for normal position, false for reversed position.
     */
    void set_state(bool new_value) OVERRIDE
    {
        if (new_value)
        {
            gpio_->write(Gpio::SET);
            pwm_->set_duty(pwm_max_ticks_);
            Serial.println("ServoGPIOBit: Turnout moved to NORMAL position");
        }
        else
        {
            gpio_->write(Gpio::CLR);
            pwm_->set_duty(pwm_min_ticks_);
            Serial.println("ServoGPIOBit: Turnout moved to REVERSED position");
        }
    }

    /**
     * @brief Get the OpenMRN node associated with this object.
     */
    Node *node() OVERRIDE
    {
        return node_;
    }

public:
    Node *node_; ///< OpenMRN node
    Gpio *gpio_; ///< Frog relay GPIO
    PWM *pwm_;   ///< Servo PWM control
    uint32_t pwm_min_ticks_; ///< PWM value for the normal position
    uint32_t pwm_max_ticks_; ///< PWM value for the reversed position
};

} // namespace openlcb

#endif // __SERVO_GPIO_BIT_HXX
