#ifndef __RGBWSTRIP_HXX
#define __RGBWSTRIP_HXX

#include <Adafruit_NeoPixel.h>
#include <ADS1115_WE.h>
#include "openlcb/DatagramHandlerDefault.hxx"
#include "openlcb/Datagram.hxx"
#include "utils/ConfigUpdateListener.hxx"
#include "RGBWConfig.h"

// Hardware configuration - NeoPixel GPIO pin on PCB
#define NEOPIXEL_PIN D10

namespace openlcb {

/// Forward declaration
class RGBWStrip;

/// Datagram handler for receiving RGBW updates from controller devices
class RGBWDatagramHandler : public DefaultDatagramHandler {
public:
    RGBWDatagramHandler(RGBWStrip *parent, DatagramService *dg_service);
    
    /// Entry point when a datagram arrives
    Action entry() override;
    
private:
    RGBWStrip *parent_;
};

/// Flow for sending datagrams (used by controller)
class RGBWSendFlow : public StateFlowBase {
public:
    RGBWSendFlow(RGBWStrip *parent);
    
    /// Trigger sending a datagram with RGBW values and brightness
    void send_datagram(uint8_t r, uint8_t g, uint8_t b, uint8_t w, uint8_t brightness);
    
private:
    Action entry();
    Action send_datagram_message();
    Action datagram_sent();
    
    RGBWStrip *parent_;
    DatagramClient *dgClient_;
    uint8_t pendingR_, pendingG_, pendingB_, pendingW_, pendingBrightness_;
    BarrierNotifiable bn_;
};

/// Main RGBW strip controller
class RGBWStrip : public DefaultConfigUpdateListener {
public:
    RGBWStrip(Node *node, const RGBWConfig &cfg, ADS1115_WE *adc, 
              DatagramService *dg_service);
    ~RGBWStrip();

    UpdateAction apply_configuration(int fd, bool initial_load, 
                                     BarrierNotifiable *done) OVERRIDE;
    
    void factory_reset(int fd) OVERRIDE;

    /// Controller: Poll ADC channels and send datagram if changed
    void poll_adc_inputs();

    /// Follower: Receive RGBW values and brightness from datagram
    void handle_datagram(NodeHandle src, const DatagramPayload &payload);

    /// Get node pointer
    Node* node() { return node_; }
    
    /// Get datagram service
    DatagramService* dg_service() { return dgService_; }

    /// Get controller node ID filter (0 = accept any)
    uint64_t controller_node_id() { return controllerNodeId_; }

private:
    void update_strip(uint8_t r, uint8_t g, uint8_t b, uint8_t w);

    Node *node_;
    const RGBWConfig cfg_;
    ADS1115_WE *adc_;
    DatagramService *dgService_;
    Adafruit_NeoPixel *strip_;
    
    bool isController_;
    uint64_t controllerNodeId_;
    
    uint8_t currentR_, currentG_, currentB_, currentW_;
    uint8_t currentBrightness_;
    uint8_t lastSentR_, lastSentG_, lastSentB_, lastSentW_, lastSentBrightness_;
    
    uint8_t adcChannelIndex_;
    
    RGBWDatagramHandler *datagramHandler_;
    RGBWSendFlow *sendFlow_;
    
    friend class RGBWDatagramHandler;
    friend class RGBWSendFlow;
};

} // namespace openlcb

#endif // __RGBWSTRIP_HXX
