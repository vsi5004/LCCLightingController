#ifndef __SERVOTURNOUT_HXX
#define __SERVOTURNOUT_HXX

#if defined(ARDUINO) || defined(ESP32)
#include "freertos_drivers/arduino/DummyGPIO.hxx"
#include "freertos_drivers/arduino/PWM.hxx"
#else
#include "freertos_drivers/common/DummyGPIO.hxx"
#include "freertos_drivers/common/PWM.hxx"
#endif
#include "os/MmapGpio.hxx"
#include "TurnoutConfig.h"
#include "ServoGPIOBit.h"
#include <memory>

namespace openlcb
{

class ServoTurnout : public DefaultConfigUpdateListener
{
public:
    ServoTurnout(Node *node, const TurnoutConfig &cfg,
        const uint32_t pwmCountPerMs, PWM *pwm, Gpio *gpio)
        : DefaultConfigUpdateListener()
        , pwmCountPerMs_(pwmCountPerMs)
        , gpioImpl_(node, 0, 0, gpio, pwm, 0, 0)
        , consumer_(&gpioImpl_)
        , cfg_(cfg)
        , frogInverted_(0xFF)
    {
    }

    UpdateAction apply_configuration(
        int fd, bool initial_load, BarrierNotifiable *done) OVERRIDE
    {
        AutoNotify n(done);

        const EventId cfg_event_min = cfg_.event_rotate_min().read(fd);
        const EventId cfg_event_max = cfg_.event_rotate_max().read(fd);
        const int16_t cfg_servo_min_pct = cfg_.servo_min_percent().read(fd);
        const int16_t cfg_servo_max_pct = cfg_.servo_max_percent().read(fd);
        const uint8_t cfg_invert_frog = cfg_.frog_inverted().read(fd);

        // 1ms duty cycle
        const uint32_t servo_ticks_0 = pwmCountPerMs_ * 1;
        // 2ms duty cycle
        const uint32_t servo_ticks_180 = pwmCountPerMs_ * 2;

        // Use a weighted average to determine num ticks for max/min.
        const uint32_t cfg_srv_ticks_min =
            ((100 - cfg_servo_min_pct) * servo_ticks_0 +
                cfg_servo_min_pct * servo_ticks_180) /
            100;
        const uint32_t cfg_srv_ticks_max =
            ((100 - cfg_servo_max_pct) * servo_ticks_0 +
                cfg_servo_max_pct * servo_ticks_180) /
            100;
        bool frogModified = false;

        if (0xFF == frogInverted_ || cfg_invert_frog != frogInverted_)
        {
          frogInverted_ = cfg_invert_frog;

          // Cast gpio_ to MCPGpio* and update inversion
          MCPGpio* gpio = static_cast<MCPGpio*>(gpioImpl_.gpio_);
          if (gpio) {
              gpio->set_frog_inverted(frogInverted_);
              frogModified = true;
          } else {
              Serial.println("Error: gpio_ is not an MCPGpio instance!");
          }
        }

        if (frogModified ||
            cfg_event_min != gpioImpl_.event_off() ||
            cfg_event_max != gpioImpl_.event_on() ||
            cfg_srv_ticks_min != gpioImpl_.pwm_min_ticks_ ||
            cfg_srv_ticks_max != gpioImpl_.pwm_max_ticks_)
        {
            auto saved_node = gpioImpl_.node();
            auto saved_gpio = gpioImpl_.gpio_;
            auto saved_pwm = gpioImpl_.pwm_;

            consumer_.~BitEventConsumer();
            gpioImpl_.~ServoGPIOBit();

            new (&gpioImpl_) ServoGPIOBit(
                saved_node, cfg_event_min, cfg_event_max, saved_gpio, saved_pwm, cfg_srv_ticks_max, cfg_srv_ticks_min );
            new (&consumer_) BitEventConsumer(&gpioImpl_);

            return REINIT_NEEDED;
        }
        return UPDATED;
    }

    void factory_reset(int fd) OVERRIDE
    {
        cfg_.description().write(fd, "");
        CDI_FACTORY_RESET(cfg_.servo_min_percent);
        CDI_FACTORY_RESET(cfg_.servo_max_percent);
        CDI_FACTORY_RESET(cfg_.frog_inverted);
    }

private:
    /// Used to compute PWM ticks for max/min servo rotation.
    const uint32_t pwmCountPerMs_;
    /// all the rest are owned and must be reset on config change.
    ServoGPIOBit gpioImpl_;          /// has on/off events, Node*, and Gpio*
    BitEventConsumer consumer_; /// has GPIOBit*
    const TurnoutConfig cfg_;
    uint8_t frogInverted_;
};

} // namespace openlcb

#endif // SERVOTURNOUT_HXX_