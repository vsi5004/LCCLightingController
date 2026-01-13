#ifndef __RGBWSTRIP_HXX
#define __RGBWSTRIP_HXX

#include <Adafruit_NeoPixel.h>
#include <ADS1115_WE.h>
#include "openlcb/EventHandlerTemplates.hxx"
#include "openlcb/EventHandler.hxx"
#include "openlcb/Convert.hxx"
#include "utils/ConfigUpdateListener.hxx"
#include "RGBWConfig.h"

// Hardware configuration - NeoPixel GPIO pin on PCB
#define NEOPIXEL_PIN D10

namespace openlcb {

/// Forward declaration
class RGBWStrip;

/// Event consumer for receiving RGBW channel updates from controller devices
class RGBWEventHandler : public SimpleEventHandler {
public:
    RGBWEventHandler(RGBWStrip *parent, int channel);
    
    /// Handle incoming channel value event
    void handle_event_report(const EventRegistryEntry &entry, EventReport *event,
                             BarrierNotifiable *done) override;
    
    void handle_identify_global(const EventRegistryEntry &entry, EventReport *event,
                                BarrierNotifiable *done) override;
    
    void handle_identify_consumer(const EventRegistryEntry &entry, EventReport *event,
                                   BarrierNotifiable *done) override;
    
    uint64_t base_event_id() const;
    
private:
    RGBWStrip *parent_;
    int channel_;  // 0=Red, 1=Green, 2=Blue, 3=White, 4=Brightness
};

/// Main RGBW strip controller
class RGBWStrip : public DefaultConfigUpdateListener {
public:
    RGBWStrip(Node *node, const RGBWConfig &cfg, ADS1115_WE *adc);
    ~RGBWStrip();

    UpdateAction apply_configuration(int fd, bool initial_load, 
                                     BarrierNotifiable *done) OVERRIDE;
    
    void factory_reset(int fd) OVERRIDE;

    /// Controller: Poll ADC channels and send events if changed
    void poll_adc_inputs();

    /// Controller: Run startup animation (fade from black to target colors)
    void run_startup_animation();
    
    /// Controller: Non-blocking startup animation state machine
    void poll_startup_animation();

    /// Follower: Handle incoming channel value event
    void handle_channel_event(int channel, uint8_t value);

    /// Controller: Send individual channel event
    void send_channel_event(int channel, uint8_t value);
    
    /// Flush pending strip updates (rate-limited)
    void flush_strip();

    /// Get node pointer
    Node* node() { return node_; }
    
    /// Get event ID for specific channel (0=R, 1=G, 2=B, 3=W, 4=Brightness)
    uint64_t event_id(int channel) { return eventIds_[channel]; }
    
    /// Get startup delay in seconds (controller only)
    uint16_t startup_delay_sec() { return startupDelaySec_; }

private:
    void update_strip(uint8_t r, uint8_t g, uint8_t b, uint8_t w);

    Node *node_;
    const RGBWConfig cfg_;
    ADS1115_WE *adc_;
    Adafruit_NeoPixel *strip_;
    
    bool isController_;
    uint64_t eventIds_[5];  // Event IDs: [R, G, B, W, Brightness]
    
    uint8_t currentR_, currentG_, currentB_, currentW_;
    uint8_t currentBrightness_;
    uint8_t lastSentR_, lastSentG_, lastSentB_, lastSentW_, lastSentBrightness_;
    
    uint8_t adcChannelIndex_;
    unsigned long lastEventSendTime_;  // Rate limiting for CAN bus
    bool startupAnimationComplete_;    // Track if startup fade-in is done
    
    // NeoPixel rate limiting (minimum ~16ms between show() calls = 60fps)
    static constexpr unsigned long MIN_SHOW_INTERVAL_MS = 16;
    unsigned long lastShowTime_;       // Last time show() was called
    bool stripDirty_;                  // True if strip needs updating
    
    // Startup animation state machine
    enum AnimationState { ANIM_IDLE, ANIM_READ_ADC, ANIM_SEND_COLORS, ANIM_FADE_BRIGHTNESS };
    AnimationState animState_;
    uint8_t animTargetR_, animTargetG_, animTargetB_, animTargetW_;
    int animBrightness_;
    unsigned long animLastUpdate_;
    int animStep_;
    
    // Periodic sync for controller
    uint16_t syncIntervalSec_;        // Sync interval in seconds (0 = disabled)
    unsigned long lastSyncTime_;       // Last time we sent a full sync
    int syncStep_;                     // Current step in sync sequence (-1 = idle)
    unsigned long lastSyncStepTime_;   // Time of last sync step
    uint16_t startupDelaySec_;        // Startup delay before fade animation
    
    RGBWEventHandler *eventHandlers_[5];  // One handler per channel
    
    friend class RGBWEventHandler;
};

} // namespace openlcb

#endif // __RGBWSTRIP_HXX
