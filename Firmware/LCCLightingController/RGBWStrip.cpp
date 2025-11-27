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
      lastSentR_(0), lastSentG_(0), lastSentB_(0), lastSentW_(0), lastSentBrightness_(255),
      adcChannelIndex_(0) {
    for (int i = 0; i < 5; i++) {
        eventIds_[i] = 0;
        eventHandlers_[i] = nullptr;
    }
}

RGBWStrip::~RGBWStrip() {
    if (strip_) delete strip_;
    for (int i = 0; i < 5; i++) {
        if (eventHandlers_[i]) delete eventHandlers_[i];
    }
}

ConfigUpdateListener::UpdateAction RGBWStrip::apply_configuration(int fd, bool initial_load, 
                                            BarrierNotifiable *done) {
    AutoNotify n(done);

    uint16_t ledCount = cfg_.led_count().read(fd);
    // Sanity check - use default if invalid
    if (ledCount == 0 || ledCount == 0xFFFF || ledCount > 1000) {
        ledCount = 120; // default value
        Serial.printf("Invalid LED count, using default: %d\n", ledCount);
    }
    
    uint8_t configuredController = cfg_.is_controller().read(fd);
    // Auto-detect: if config is at default (255/0xFF) and ADS1115 is present, become controller
    if (configuredController == 0xFF || configuredController > 1) {
        // Auto-detect mode (ADC is already initialized in main setup)
        if (adc_) {
            isController_ = true;
            Serial.println("Auto-detect: ADS1115 found - configured as CONTROLLER");
        } else {
            isController_ = false;
            Serial.println("Auto-detect: No ADS1115 - configured as FOLLOWER");
        }
    } else {
        // Explicit configuration: 0=follower, 1=controller
        isController_ = (configuredController == 1);
    }
    
    // Read event IDs for each channel
    eventIds_[0] = cfg_.red_event().read(fd);
    eventIds_[1] = cfg_.green_event().read(fd);
    eventIds_[2] = cfg_.blue_event().read(fd);
    eventIds_[3] = cfg_.white_event().read(fd);
    eventIds_[4] = cfg_.brightness_event().read(fd);
    
    Serial.printf("Event IDs - R:0x%012llX G:0x%012llX B:0x%012llX W:0x%012llX Br:0x%012llX\n",
                 eventIds_[0], eventIds_[1], eventIds_[2], eventIds_[3], eventIds_[4]);

    // Reinitialize NeoPixel strip if parameters changed
    if (!strip_ || strip_->numPixels() != ledCount) {
        if (strip_) delete strip_;
        strip_ = new Adafruit_NeoPixel(ledCount, NEOPIXEL_PIN, NEO_WRGB + NEO_KHZ800);
        strip_->begin();
        strip_->setBrightness(currentBrightness_);
        strip_->fill(strip_->Color(0, 0, 0, 0));
        strip_->show();
        Serial.printf("NeoPixel initialized: %d LEDs on pin %d\n", ledCount, NEOPIXEL_PIN);
    }

    // Register event handlers only for followers (controller only sends, doesn't receive)
    if (!isController_) {
        for (int i = 0; i < 5; i++) {
            if (!eventHandlers_[i]) {
                eventHandlers_[i] = new RGBWEventHandler(this, i);
            }
        }
        Serial.println("Event handlers registered for all channels");
    } else {
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
    CDI_FACTORY_RESET(cfg_.is_controller);
    cfg_.red_event().write(fd, RGBW_EVENT_INIT[0]);
    cfg_.green_event().write(fd, RGBW_EVENT_INIT[1]);
    cfg_.blue_event().write(fd, RGBW_EVENT_INIT[2]);
    cfg_.white_event().write(fd, RGBW_EVENT_INIT[3]);
    cfg_.brightness_event().write(fd, RGBW_EVENT_INIT[4]);
}

void RGBWStrip::poll_adc_inputs() {
    if (!isController_ || !adc_) return;

    static const ADS1115_MUX channels[4] = {
        ADS1115_COMP_0_GND, ADS1115_COMP_1_GND,
        ADS1115_COMP_2_GND, ADS1115_COMP_3_GND
    };
    
    static bool firstCycle = true;
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

    bool changed = false;
    switch (adcChannelIndex_) {
        case 0: if (mapped != currentR_) { currentR_ = mapped; changed = true; } break;
        case 1: if (mapped != currentG_) { currentG_ = mapped; changed = true; } break;
        case 2: if (mapped != currentB_) { currentB_ = mapped; changed = true; } break;
        case 3: if (mapped != currentW_) { currentW_ = mapped; changed = true; } break;
    }

    // Move to next channel and set it up for next read
    adcChannelIndex_ = (adcChannelIndex_ + 1) % 4;
    adc_->setCompareChannels(channels[adcChannelIndex_]);
    
    // Update strip after reading all 4 channels (even if no change, for first cycle)
    if (adcChannelIndex_ == 0 || changed) {
        if (firstCycle && adcChannelIndex_ == 0) {
            Serial.printf("Initial RGBW: R=%d G=%d B=%d W=%d\n", 
                         currentR_, currentG_, currentB_, currentW_);
            firstCycle = false;
        }
        
        update_strip(currentR_, currentG_, currentB_, currentW_);
        
        // Send events for any changed channels
        if (changed) {
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
            Serial.printf("RGBW Update: R=%d G=%d B=%d W=%d Brightness=%d\n",
                         currentR_, currentG_, currentB_, currentW_, currentBrightness_);
        }
    }
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
    
    const char* names[] = {"Red", "Green", "Blue", "White", "Brightness"};
    Serial.printf("Sent %s event: 0x%012llX (value=%d)\n", names[channel], encoded_event, value);
}

void RGBWStrip::handle_channel_event(int channel, uint8_t value) {
    const char* names[] = {"Red", "Green", "Blue", "White", "Brightness"};
    
    switch (channel) {
        case 0: currentR_ = value; break;
        case 1: currentG_ = value; break;
        case 2: currentB_ = value; break;
        case 3: currentW_ = value; break;
        case 4: 
            currentBrightness_ = value;
            if (strip_) {
                strip_->setBrightness(value);
            }
            break;
    }
    
    update_strip(currentR_, currentG_, currentB_, currentW_);
    Serial.printf("Received %s event: value=%d\n", names[channel], value);
}

void RGBWStrip::update_strip(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
    if (!strip_) return;
    strip_->fill(strip_->Color(r, g, b, strip_->gamma8(w)));
    strip_->show();
}

// ============================================================================
// RGBWEventHandler Implementation
// ============================================================================

RGBWEventHandler::RGBWEventHandler(RGBWStrip *parent, int channel)
    : parent_(parent), channel_(channel) {
    // Register this handler for a range of 256 events (base + 0-255)
    // The mask is applied to the lower 32 bits only in OpenMRN
    // We use 0xFFFFFF00 to mask the lower byte, allowing values 0-255
    EventRegistry::instance()->register_handler(
        EventRegistryEntry(this, parent_->event_id(channel), 0xFFFFFF00), 0);
}

uint64_t RGBWEventHandler::base_event_id() const {
    return parent_->event_id(channel_) & 0xFFFFFFFFFFFFFF00ULL;
}

void RGBWEventHandler::handle_event_report(const EventRegistryEntry &entry,
                                           EventReport *event,
                                           BarrierNotifiable *done) {
    AutoNotify an(done);
    
    // Check if this event matches our base (top 56 bits)
    uint64_t received_base = event->event & 0xFFFFFFFFFFFFFF00ULL;
    
    if (received_base == base_event_id()) {
        // Extract value from lower byte
        uint8_t value = event->event & 0xFF;
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
