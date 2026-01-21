#include "RGBWStrip.h"
#include "config.h"

namespace openlcb {

// ============================================================================
// RGBWStrip Implementation
// ============================================================================

RGBWStrip::RGBWStrip(Node *node, const RGBWConfig &cfg, ADS1115_WE *adc)
    : node_(node), cfg_(cfg), adc_(adc),
      strip_(nullptr), isController_(false),
      currentR_(0), currentG_(0), currentB_(0), currentW_(0), currentBrightness_(255),
      pendingR_(0), pendingG_(0), pendingB_(0), pendingW_(0), pendingBrightness_(255),
      fadeInProgress_(false), fadeStartTime_(0), fadeDurationMs_(0),
      fadeStartR_(0), fadeStartG_(0), fadeStartB_(0), fadeStartW_(0), fadeStartBrightness_(255),
      fadeTargetR_(0), fadeTargetG_(0), fadeTargetB_(0), fadeTargetW_(0), fadeTargetBrightness_(255),
      lastSentR_(0), lastSentG_(0), lastSentB_(0), lastSentW_(0), lastSentBrightness_(255),
      adcChannelIndex_(0), lastEventSendTime_(0), startupAnimationComplete_(false),
      lastShowTime_(0), stripDirty_(false),
      animState_(ANIM_IDLE), animTargetR_(0), animTargetG_(0), animTargetB_(0), animTargetW_(0),
      animBrightness_(0), animLastUpdate_(0), animStep_(0),
      syncIntervalSec_(3), lastSyncTime_(0), syncStep_(-1), lastSyncStepTime_(0),
      startupDelaySec_(5) {
    for (int i = 0; i < 6; i++) {
        eventIds_[i] = 0;
        eventHandlers_[i] = nullptr;
    }
}

RGBWStrip::~RGBWStrip() {
    if (strip_) delete strip_;
    for (int i = 0; i < 6; i++) {
        if (eventHandlers_[i]) delete eventHandlers_[i];
    }
}

ConfigUpdateListener::UpdateAction RGBWStrip::apply_configuration(int fd, bool initial_load, 
                                            BarrierNotifiable *done) {
    AutoNotify n(done);

    // Default values to use if config cannot be read
    uint16_t ledCount = DEFAULT_LED_COUNT;
    bool useDefaults = false;

    // Check if file descriptor is valid
    if (fd < 0) {
        Serial.printf("WARNING: Invalid file descriptor (fd=%d), using default configuration\n", fd);
        useDefaults = true;
    }

    if (!useDefaults) {
        ledCount = cfg_.led_count().read(fd);
        // Sanity check - use default if invalid
        if (ledCount == 0 || ledCount == 0xFFFF || ledCount > 1000) {
            ledCount = DEFAULT_LED_COUNT;
            Serial.printf("Invalid LED count, using default: %d\n", ledCount);
        }
    }
    
    // Auto-detect controller mode based on ADC presence
    if (adc_ && !adc_->isDisconnected()) {
        isController_ = true;
        Serial.println("Auto-detect: ADS1115 detected - configured as CONTROLLER");
    } else {
        isController_ = false;
        Serial.println("Auto-detect: No ADS1115 - configured as FOLLOWER");
    }
    
    // Read event IDs for each channel (use defaults if fd invalid)
    if (!useDefaults) {
        eventIds_[0] = cfg_.red_event().read(fd);
        eventIds_[1] = cfg_.green_event().read(fd);
        eventIds_[2] = cfg_.blue_event().read(fd);
        eventIds_[3] = cfg_.white_event().read(fd);
        eventIds_[4] = cfg_.brightness_event().read(fd);
        eventIds_[5] = cfg_.duration_event().read(fd);
    } else {
        // Use default event IDs from config.h
        eventIds_[0] = RGBW_EVENT_INIT[0];
        eventIds_[1] = RGBW_EVENT_INIT[1];
        eventIds_[2] = RGBW_EVENT_INIT[2];
        eventIds_[3] = RGBW_EVENT_INIT[3];
        eventIds_[4] = RGBW_EVENT_INIT[4];
        eventIds_[5] = RGBW_EVENT_INIT[5];
    }
    
    Serial.printf("Event IDs - R:0x%016llX G:0x%016llX B:0x%016llX W:0x%016llX Br:0x%016llX Dur:0x%016llX\n",
                 eventIds_[0], eventIds_[1], eventIds_[2], eventIds_[3], eventIds_[4], eventIds_[5]);

    // Reinitialize NeoPixel strip if parameters changed
    if (!strip_ || strip_->numPixels() != ledCount) {
        if (strip_) delete strip_;
        strip_ = new Adafruit_NeoPixel(ledCount, NEOPIXEL_PIN, NEO_WRGB + NEO_KHZ800);
        strip_->begin();
        //strip_->setBrightness(0);  // Start with brightness 0 for clean fade-in
        //strip_->fill(strip_->Color(0, 0, 0, 0));
        //strip_->show();
        Serial.printf("NeoPixel initialized: %d LEDs on pin %d\n", ledCount, NEOPIXEL_PIN);
    }

    // Register event handlers only for followers (controller only sends, doesn't receive)
    if (!isController_) {
        for (int i = 0; i < 6; i++) {
            if (!eventHandlers_[i]) {
                eventHandlers_[i] = new RGBWEventHandler(this, i);
            }
        }
        Serial.println("Event handlers registered for all 6 channels (RGBW+Br+Dur)");
    } else {
        // Controller: read sync interval and startup delay config
        if (!useDefaults) {
            syncIntervalSec_ = cfg_.sync_interval().read(fd);
            if (syncIntervalSec_ > 60) syncIntervalSec_ = 3; // Sanity check
            startupDelaySec_ = cfg_.startup_delay().read(fd);
            if (startupDelaySec_ > 30) startupDelaySec_ = 5; // Sanity check
        }
        // If useDefaults, keep constructor default values (syncIntervalSec_=3, startupDelaySec_=5)
        Serial.printf("Controller sync interval: %d seconds\n", syncIntervalSec_);
        Serial.printf("Controller startup delay: %d seconds\n", startupDelaySec_);
        Serial.println("Controller mode - event handlers not registered (send only)");
    }

    if (isController_) {
        Serial.println("Running as CONTROLLER (HMI device)");
    } else {
        Serial.println("Running as FOLLOWER");
    }

    return UPDATED;
}

void RGBWStrip::factory_reset(int fd) {
    cfg_.description().write(fd, "");
    CDI_FACTORY_RESET(cfg_.led_count);
    cfg_.red_event().write(fd, RGBW_EVENT_INIT[0]);
    cfg_.green_event().write(fd, RGBW_EVENT_INIT[1]);
    cfg_.blue_event().write(fd, RGBW_EVENT_INIT[2]);
    cfg_.white_event().write(fd, RGBW_EVENT_INIT[3]);
    cfg_.brightness_event().write(fd, RGBW_EVENT_INIT[4]);
    cfg_.duration_event().write(fd, RGBW_EVENT_INIT[5]);
}

void RGBWStrip::run_startup_animation() {
    if (!isController_ || !strip_) return;
    
    // Start the non-blocking animation state machine
    animState_ = ANIM_READ_ADC;
    animStep_ = 0;
    animLastUpdate_ = millis();
    Serial.println("Starting startup animation...");
}

void RGBWStrip::poll_startup_animation() {
    static const ADS1115_MUX channels[4] = {
        ADS1115_COMP_0_GND, ADS1115_COMP_1_GND,
        ADS1115_COMP_2_GND, ADS1115_COMP_3_GND
    };
    
    switch (animState_) {
        case ANIM_READ_ADC:
            // Read all 4 ADC channels
            if (animStep_ < 4) {
                adc_->setCompareChannels(channels[animStep_]);
                float voltage = adc_->getResult_mV();
                long clamped = constrain((long)voltage, 0, 3300);
                uint8_t mapped = map(clamped, 0, 3300, 0, 255);
                
                switch(animStep_) {
                    case 0: animTargetR_ = mapped; break;
                    case 1: animTargetG_ = mapped; break;
                    case 2: animTargetB_ = mapped; break;
                    case 3: animTargetW_ = mapped; break;
                }
                animStep_++;
            } else {
                Serial.printf("Startup animation: Fading to R=%d G=%d B=%d W=%d\n", 
                             animTargetR_, animTargetG_, animTargetB_, animTargetW_);
                
                // Set brightness to 0 and prepare colors - update local LEDs first, then send event
                strip_->fill(strip_->Color(animTargetR_, animTargetG_, animTargetB_, animTargetW_));
                strip_->setBrightness(0);
                stripDirty_ = true;
                flush_strip();
                send_channel_event(4, 0);
                
                animState_ = ANIM_SEND_COLORS;
                animStep_ = 0;
                animLastUpdate_ = millis();
            }
            break;
            
        case ANIM_SEND_COLORS:
            // Send color events one at a time 
            if (millis() - animLastUpdate_ >= 10) {
                switch(animStep_) {
                    case 0: send_channel_event(0, animTargetR_); break;
                    case 1: send_channel_event(1, animTargetG_); break;
                    case 2: send_channel_event(2, animTargetB_); break;
                    case 3: send_channel_event(3, animTargetW_); break;
                }
                animStep_++;
                animLastUpdate_ = millis();
                
                if (animStep_ >= 4) {
                    animState_ = ANIM_FADE_BRIGHTNESS;
                    animBrightness_ = 0;
                    animLastUpdate_ = millis();
                }
            }
            break;
            
        case ANIM_FADE_BRIGHTNESS:
            // Ramp brightness 0→255 over ~5 seconds (40ms per step, increment by 2)
            if (millis() - animLastUpdate_ >= 40) {
                // Update LEDs first, then send event to reduce interrupt conflicts
                strip_->fill(strip_->Color(animTargetR_, animTargetG_, animTargetB_, animTargetW_));
                strip_->setBrightness(animBrightness_);
                stripDirty_ = true;
                flush_strip();
                send_channel_event(4, animBrightness_);
                animLastUpdate_ = millis();
                
                // Increment brightness, capping at 255
                if (animBrightness_ + ANIM_BRIGHTNESS_STEP <= 255) {
                    animBrightness_ += ANIM_BRIGHTNESS_STEP;
                } else {
                    animBrightness_ = 255;
                    
                    // Animation complete - sync current values with what was sent
                    currentR_ = animTargetR_;
                    currentG_ = animTargetG_;
                    currentB_ = animTargetB_;
                    currentW_ = animTargetW_;
                    lastSentR_ = animTargetR_;
                    lastSentG_ = animTargetG_;
                    lastSentB_ = animTargetB_;
                    lastSentW_ = animTargetW_;
                    lastSentBrightness_ = 255;
                    
                    // Reset ADC for normal polling
                    adcChannelIndex_ = 0;
                    adc_->setCompareChannels(channels[0]);
                    startupAnimationComplete_ = true;
                    animState_ = ANIM_IDLE;
                    Serial.println("Startup animation complete");
                }
            }
            break;
            
        case ANIM_IDLE:
        default:
            break;
    }
}

void RGBWStrip::poll_adc_inputs() {
    if (!isController_ || !adc_ || adc_->isDisconnected()) return;
    
    // Run startup animation state machine if active
    if (!startupAnimationComplete_) {
        poll_startup_animation();
        return;
    }

    static const ADS1115_MUX channels[4] = {
        ADS1115_COMP_0_GND, ADS1115_COMP_1_GND,
        ADS1115_COMP_2_GND, ADS1115_COMP_3_GND
    };
    
    static bool initialized = false;
    
    // On first call, just set up the first channel
    if (!initialized) {
        adc_->setCompareChannels(channels[0]);
        initialized = true;
        return;
    }

    // Read the current channel (was set in previous call)
    float voltage = adc_->getResult_mV();
    
    long clamped = constrain((long)voltage, 0, 3300);
    uint8_t mapped = map(clamped, 0, 3300, 0, 255);

    // Hysteresis: ignore changes < 2 to reduce jitter
    bool changed = false;
    switch (adcChannelIndex_) {
        case 0: if (abs((int)mapped - (int)currentR_) >= 2) { currentR_ = mapped; changed = true; } break;
        case 1: if (abs((int)mapped - (int)currentG_) >= 2) { currentG_ = mapped; changed = true; } break;
        case 2: if (abs((int)mapped - (int)currentB_) >= 2) { currentB_ = mapped; changed = true; } break;
        case 3: if (abs((int)mapped - (int)currentW_) >= 2) { currentW_ = mapped; changed = true; } break;
    }

    // Move to next channel and set it up for next read
    adcChannelIndex_ = (adcChannelIndex_ + 1) % 4;
    adc_->setCompareChannels(channels[adcChannelIndex_]);
    
    // Update strip only when values actually change
    if (changed) {
        // Update LEDs first, then send events to reduce interrupt conflicts
        update_strip(currentR_, currentG_, currentB_, currentW_);
        flush_strip();
        
        // Send events for any changed channels (rate limited to prevent CAN bus flooding)
        // Maximum 1 event burst every 50ms
        if (millis() - lastEventSendTime_ >= 50) {
            if (currentR_ != lastSentR_) {
                send_channel_event(0, currentR_);
                lastSentR_ = currentR_;
            }
            if (currentG_ != lastSentG_) {
                send_channel_event(1, currentG_);
                lastSentG_ = currentG_;
            }
            if (currentB_ != lastSentB_) {
                send_channel_event(2, currentB_);
                lastSentB_ = currentB_;
            }
            if (currentW_ != lastSentW_) {
                send_channel_event(3, currentW_);
                lastSentW_ = currentW_;
            }
            lastEventSendTime_ = millis();
            Serial.printf("RGBW Update: R=%d G=%d B=%d W=%d Brightness=%d\n",
                         currentR_, currentG_, currentB_, currentW_, currentBrightness_);
        }
    }
    
    // Periodic sync for followers (controller only)
    if (syncIntervalSec_ > 0) {
        unsigned long syncIntervalMs = syncIntervalSec_ * 1000UL;
        
        // Check if it's time to start a new sync sequence
        if (syncStep_ < 0 && (millis() - lastSyncTime_ >= syncIntervalMs)) {
            syncStep_ = 0;
            lastSyncStepTime_ = millis();
        }
        
        // Run sync sequence - send one channel every 20ms
        if (syncStep_ >= 0 && (millis() - lastSyncStepTime_ >= 20)) {
            switch (syncStep_) {
                case 0: send_channel_event(0, currentR_); break;
                case 1: send_channel_event(1, currentG_); break;
                case 2: send_channel_event(2, currentB_); break;
                case 3: send_channel_event(3, currentW_); break;
                case 4: send_channel_event(4, currentBrightness_); break;
            }
            lastSyncStepTime_ = millis();
            syncStep_++;
            
            if (syncStep_ > 4) {
                // Sync sequence complete
                syncStep_ = -1;
                lastSyncTime_ = millis();
            }
        }
    }
    
    // Flush any pending strip updates (rate-limited)
    flush_strip();
}

void RGBWStrip::send_channel_event(int channel, uint8_t value) {
    // Encode value into lower byte of event ID
    uint64_t base_event = eventIds_[channel] & 0xFFFFFFFFFFFFFF00ULL;
    uint64_t encoded_event = base_event | value;
    
    // Send as global event report
    auto *msg = node_->iface()->global_message_write_flow()->alloc();
    msg->data()->reset(Defs::MTI_EVENT_REPORT, node_->node_id(), 
                      eventid_to_buffer(encoded_event));
    node_->iface()->global_message_write_flow()->send(msg);
}

void RGBWStrip::handle_channel_event(int channel, uint8_t value) {
    const char* names[] = {"Red", "Green", "Blue", "White", "Brightness", "Duration"};
    
    switch (channel) {
        case 0: pendingR_ = value; break;
        case 1: pendingG_ = value; break;
        case 2: pendingB_ = value; break;
        case 3: pendingW_ = value; break;
        case 4: pendingBrightness_ = value; break;
        case 5: 
            // Duration event triggers the fade
            // Capture current actual values as fade start
            fadeStartR_ = currentR_;
            fadeStartG_ = currentG_;
            fadeStartB_ = currentB_;
            fadeStartW_ = currentW_;
            fadeStartBrightness_ = currentBrightness_;
            
            // Pending values become fade targets
            fadeTargetR_ = pendingR_;
            fadeTargetG_ = pendingG_;
            fadeTargetB_ = pendingB_;
            fadeTargetW_ = pendingW_;
            fadeTargetBrightness_ = pendingBrightness_;
            
            // Duration in seconds (0 = instant)
            fadeDurationMs_ = (unsigned long)value * 1000UL;
            fadeStartTime_ = millis();
            
            // Clear any pending strip writes to prevent flash to stale buffer
            stripDirty_ = false;
            
            if (value == 0) {
                // Instant apply
                currentR_ = fadeTargetR_;
                currentG_ = fadeTargetG_;
                currentB_ = fadeTargetB_;
                currentW_ = fadeTargetW_;
                currentBrightness_ = fadeTargetBrightness_;
                if (strip_) strip_->setBrightness(currentBrightness_);
                update_strip(currentR_, currentG_, currentB_, currentW_);
                flush_strip();
                fadeInProgress_ = false;
                Serial.printf("Instant apply: R=%d G=%d B=%d W=%d Br=%d\n",
                             currentR_, currentG_, currentB_, currentW_, currentBrightness_);
            } else {
                fadeInProgress_ = true;
                Serial.printf("Starting %d sec fade: R=%d->%d G=%d->%d B=%d->%d W=%d->%d Br=%d->%d\n",
                             value,
                             fadeStartR_, fadeTargetR_, fadeStartG_, fadeTargetG_,
                             fadeStartB_, fadeTargetB_, fadeStartW_, fadeTargetW_,
                             fadeStartBrightness_, fadeTargetBrightness_);
            }
            return;  // Don't print redundant message below
    }
    
    Serial.printf("Received %s event: value=%d (pending)\n", names[channel], value);
}

void RGBWStrip::update_strip(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
    if (!strip_) return;
    strip_->fill(strip_->Color(r, g, b, w));
    stripDirty_ = true;
}

void RGBWStrip::flush_strip() {
    if (!strip_ || !stripDirty_) return;
    
    // Rate limit show() calls to prevent green glitches
    // NeoPixel needs time to complete transmission to LEDs
    unsigned long now = millis();
    if (now - lastShowTime_ >= MIN_SHOW_INTERVAL_MS) {
        strip_->show();
        lastShowTime_ = now;
        stripDirty_ = false;
    }
    // If rate limited, stripDirty_ stays true for next poll_fade() call
}

void RGBWStrip::poll_fade() {
    if (!fadeInProgress_ || !strip_) return;
    
    unsigned long now = millis();
    unsigned long elapsed = now - fadeStartTime_;
    
    // Calculate progress (0.0 to 1.0)
    float progress;
    if (elapsed >= fadeDurationMs_) {
        progress = 1.0f;
    } else {
        progress = (float)elapsed / (float)fadeDurationMs_;
    }
    
    // Interpolate all channels
    uint8_t newR = fadeStartR_ + (int16_t)(fadeTargetR_ - fadeStartR_) * progress;
    uint8_t newG = fadeStartG_ + (int16_t)(fadeTargetG_ - fadeStartG_) * progress;
    uint8_t newB = fadeStartB_ + (int16_t)(fadeTargetB_ - fadeStartB_) * progress;
    uint8_t newW = fadeStartW_ + (int16_t)(fadeTargetW_ - fadeStartW_) * progress;
    uint8_t newBr = fadeStartBrightness_ + (int16_t)(fadeTargetBrightness_ - fadeStartBrightness_) * progress;
    
    // Only update if any value changed (avoid redundant writes)
    bool changed = (newR != currentR_ || newG != currentG_ || newB != currentB_ || 
                    newW != currentW_ || newBr != currentBrightness_);
    
    if (changed) {
        currentR_ = newR;
        currentG_ = newG;
        currentB_ = newB;
        currentW_ = newW;
        currentBrightness_ = newBr;
        
        strip_->setBrightness(currentBrightness_);
        update_strip(currentR_, currentG_, currentB_, currentW_);
        flush_strip();
    }
    
    // Check if fade is complete
    if (progress >= 1.0f) {
        fadeInProgress_ = false;
        Serial.printf("Fade complete: R=%d G=%d B=%d W=%d Br=%d\n",
                     currentR_, currentG_, currentB_, currentW_, currentBrightness_);
    }
}

// ============================================================================
// RGBWEventHandler Implementation
// ============================================================================

RGBWEventHandler::RGBWEventHandler(RGBWStrip *parent, int channel)
    : parent_(parent), channel_(channel) {
    // Register with base event (lower byte = 0) and mask 8 bits (256 values)
    // The mask parameter is the NUMBER OF BITS to mask, not a bitmask
    uint64_t base = parent_->event_id(channel) & 0xFFFFFFFFFFFFFF00ULL;
    EventRegistry::instance()->register_handler(
        EventRegistryEntry(this, base), 8);
}

uint64_t RGBWEventHandler::base_event_id() const {
    return parent_->event_id(channel_) & 0xFFFFFFFFFFFFFF00ULL;
}

void RGBWEventHandler::handle_event_report(const EventRegistryEntry &entry,
                                           EventReport *event,
                                           BarrierNotifiable *done) {
    AutoNotify an(done);
    
    // Extract value from lower byte
    uint8_t value = event->event & 0xFF;
    
    // Check if this event matches our base (top 56 bits)
    uint64_t received_base = event->event & 0xFFFFFFFFFFFFFF00ULL;
    uint64_t expected_base = base_event_id();
    
    if (received_base == expected_base) {
        parent_->handle_channel_event(channel_, value);
    }
}

void RGBWEventHandler::handle_identify_global(const EventRegistryEntry &entry,
                                               EventReport *event,
                                               BarrierNotifiable *done) {
    if (parent_->node()->is_initialized()) {
        event->event_write_helper<1>()->WriteAsync(
            parent_->node(), 
            Defs::MTI_CONSUMER_IDENTIFIED_VALID,
            WriteHelper::global(),
            eventid_to_buffer(parent_->event_id(channel_)),
            done->new_child());
    }
    done->maybe_done();
}

void RGBWEventHandler::handle_identify_consumer(const EventRegistryEntry &entry,
                                                 EventReport *event,
                                                 BarrierNotifiable *done) {
    uint64_t received_base = event->event & 0xFFFFFFFFFFFFFF00ULL;
    if (received_base == base_event_id()) {
        event->event_write_helper<1>()->WriteAsync(
            parent_->node(),
            Defs::MTI_CONSUMER_IDENTIFIED_VALID,
            WriteHelper::global(),
            eventid_to_buffer(parent_->event_id(channel_)),
            done->new_child());
    }
    done->maybe_done();
}

} // namespace openlcb
