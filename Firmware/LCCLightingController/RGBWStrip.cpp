#include "RGBWStrip.h"

namespace openlcb {

// Datagram protocol ID for RGBW updates
static constexpr uint8_t RGBW_DATAGRAM_ID = 0x80;

// ============================================================================
// RGBWStrip Implementation
// ============================================================================

RGBWStrip::RGBWStrip(Node *node, const RGBWConfig &cfg, ADS1115_WE *adc,
                     DatagramService *dg_service)
    : node_(node), cfg_(cfg), adc_(adc), dgService_(dg_service),
      strip_(nullptr), isController_(false), controllerNodeId_(0),
      currentR_(0), currentG_(0), currentB_(0), currentW_(0), currentBrightness_(255),
      lastSentR_(0), lastSentG_(0), lastSentB_(0), lastSentW_(0), lastSentBrightness_(255),
      adcChannelIndex_(0), datagramHandler_(nullptr), sendFlow_(nullptr) {}

RGBWStrip::~RGBWStrip() {
    if (strip_) delete strip_;
    if (datagramHandler_) {
        // Note: OpenMRN doesn't provide unregister, handler will be cleaned up on destruction
        delete datagramHandler_;
    }
    if (sendFlow_) {
        delete sendFlow_;
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
    
    controllerNodeId_ = cfg_.controller_node_id().read(fd);

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

    // Register datagram handler (for receiving datagrams)
    // Note: Handler registers itself in its constructor
    if (!datagramHandler_) {
        datagramHandler_ = new RGBWDatagramHandler(this, dgService_);
        Serial.println("Datagram handler registered");
    }
    
    // Create send flow for controller (for sending datagrams)
    if (isController_ && !sendFlow_) {
        sendFlow_ = new RGBWSendFlow(this);
        Serial.println("Datagram send flow created");
    }

    if (isController_) {
        Serial.println("Running as CONTROLLER (HMI device)");
    } else {
        Serial.printf("Running as FOLLOWER (listening to Node ID: 0x%012llX)\n", 
                     controllerNodeId_);
    }

    return UPDATED;
}

void RGBWStrip::factory_reset(int fd) {
    cfg_.description().write(fd, "");
    CDI_FACTORY_RESET(cfg_.led_count);
    CDI_FACTORY_RESET(cfg_.is_controller);
    CDI_FACTORY_RESET(cfg_.controller_node_id);
}

void RGBWStrip::poll_adc_inputs() {
    if (!isController_ || !adc_ || !sendFlow_) return;

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
        
        // Log and send on any change
        if (changed) {
            Serial.printf("RGBW Update: R=%d G=%d B=%d W=%d Brightness=%d\n",
                         currentR_, currentG_, currentB_, currentW_, currentBrightness_);
            sendFlow_->send_datagram(currentR_, currentG_, currentB_, currentW_, currentBrightness_);
            lastSentR_ = currentR_;
            lastSentG_ = currentG_;
            lastSentB_ = currentB_;
            lastSentW_ = currentW_;
            lastSentBrightness_ = currentBrightness_;
        }
    }
}

void RGBWStrip::handle_datagram(NodeHandle src, const DatagramPayload &payload) {
    // Check if we should accept (filter by source node ID)
    if (controllerNodeId_ != 0 && src.id != controllerNodeId_) {
        return; // Ignore datagrams from other controllers
    }
    
    // Verify protocol and size (now includes brightness byte)
    if (payload.size() != 6 || (uint8_t)payload[0] != RGBW_DATAGRAM_ID) {
        return; // Invalid datagram
    }
    
    // Extract RGBW values and brightness
    uint8_t r = (uint8_t)payload[1];
    uint8_t g = (uint8_t)payload[2];
    uint8_t b = (uint8_t)payload[3];
    uint8_t w = (uint8_t)payload[4];
    uint8_t brightness = (uint8_t)payload[5];
    
    currentR_ = r;
    currentG_ = g;
    currentB_ = b;
    currentW_ = w;
    currentBrightness_ = brightness;
    
    // Apply brightness to strip
    if (strip_) {
        strip_->setBrightness(brightness);
    }
    update_strip(r, g, b, w);
    
    Serial.printf("Received RGBW: R=%d G=%d B=%d W=%d Brightness=%d from 0x%012llX\n", 
                 r, g, b, w, brightness, src.id);
}

void RGBWStrip::update_strip(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
    if (!strip_) return;
    strip_->fill(strip_->Color(r, g, b, strip_->gamma8(w)));
    strip_->show();
}

// ============================================================================
// RGBWDatagramHandler Implementation (Receiving)
// ============================================================================

RGBWDatagramHandler::RGBWDatagramHandler(RGBWStrip *parent, 
                                         DatagramService *dg_service)
    : DefaultDatagramHandler(dg_service), parent_(parent) {}

StateFlowBase::Action RGBWDatagramHandler::entry() {
    // Get the incoming datagram
    auto *dg = message()->data();
    
    // Pass to parent for handling
    parent_->handle_datagram(dg->src, dg->payload);
    
    // Send positive acknowledgement (no reply pending)
    return respond_ok(0);
}

// ============================================================================
// RGBWSendFlow Implementation (Sending)
// ============================================================================

RGBWSendFlow::RGBWSendFlow(RGBWStrip *parent)
    : StateFlowBase(parent->dg_service()), 
      parent_(parent), 
      dgClient_(nullptr),
      pendingR_(0), pendingG_(0), pendingB_(0), pendingW_(0), pendingBrightness_(255) {}

void RGBWSendFlow::send_datagram(uint8_t r, uint8_t g, uint8_t b, uint8_t w, uint8_t brightness) {
    // Store values to send
    pendingR_ = r;
    pendingG_ = g;
    pendingB_ = b;
    pendingW_ = w;
    pendingBrightness_ = brightness;
    
    // Trigger the flow
    start_flow(STATE(entry));
}

StateFlowBase::Action RGBWSendFlow::entry() {
    return allocate_and_call(STATE(send_datagram_message),
                             parent_->dg_service()->client_allocator());
}

StateFlowBase::Action RGBWSendFlow::send_datagram_message() {
    dgClient_ = full_allocation_result(parent_->dg_service()->client_allocator());
    
    // NOTE: LCC datagrams are point-to-point only and cannot be broadcast.
    // For now, controller only updates its own strip locally.
    // TODO: Implement proper follower support either by:
    //   1. Sending individual datagrams to each configured follower node ID
    //   2. Using LCC Producer-Consumer events with RGBW data in event payload
    
    Serial.printf("Local update only (broadcast not supported): R=%d G=%d B=%d W=%d Brightness=%d\n", 
                 pendingR_, pendingG_, pendingB_, pendingW_, pendingBrightness_);
    
    // Return client and exit immediately
    parent_->dg_service()->client_allocator()->typed_insert(dgClient_);
    dgClient_ = nullptr;
    
    return exit();
}

StateFlowBase::Action RGBWSendFlow::datagram_sent() {
    // Check result (optional - could log errors)
    uint32_t result = dgClient_->result();
    if (!(result & DatagramClient::OPERATION_SUCCESS)) {
        Serial.printf("Datagram send failed: 0x%04X\n", result);
    }
    
    // Return client to pool
    parent_->dg_service()->client_allocator()->typed_insert(dgClient_);
    dgClient_ = nullptr;
    
    return exit();
}

} // namespace openlcb
