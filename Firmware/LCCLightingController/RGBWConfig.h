#ifndef __RGBWCONFIG_HXX
#define __RGBWCONFIG_HXX

#include "openlcb/ConfigRepresentation.hxx"

CDI_GROUP(RGBWConfig);
CDI_GROUP_ENTRY(description, openlcb::StringConfigEntry<16>, 
    Name("Description"),
    Description("User name of this RGBW light strip."));

CDI_GROUP_ENTRY(is_controller, openlcb::Uint8ConfigEntry, 
    Default(255), Min(0), Max(1),
    Name("Controller Device"),
    Description("Set to 1 for controller (HMI inputs), 0 for follower. Leave at 255 for auto-detect based on ADS1115 presence."));

// For followers: which controller to listen to
CDI_GROUP_ENTRY(controller_node_id, openlcb::Uint64ConfigEntry,
    Default(0),
    Name("Controller Node ID"),
    Description("Node ID of the controller to listen to (followers only). "
                "Set to 0 to listen to any controller (broadcast)."));

CDI_GROUP_ENTRY(led_count, openlcb::Uint16ConfigEntry,
    Default(120), Min(1), Max(1000),
    Name("LED Count"),
    Description("Number of LEDs in the NeoPixel strip."));

CDI_GROUP_END();

#endif // __RGBWCONFIG_HXX
