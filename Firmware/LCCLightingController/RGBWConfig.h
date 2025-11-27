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

CDI_GROUP_ENTRY(red_event, openlcb::EventConfigEntry,
    Name("Red Channel Event"),
    Description("Event ID base for red channel (0-255). Controller produces, followers consume. Must end in 00."));

CDI_GROUP_ENTRY(green_event, openlcb::EventConfigEntry,
    Name("Green Channel Event"),
    Description("Event ID base for green channel (0-255). Controller produces, followers consume. Must end in 00."));

CDI_GROUP_ENTRY(blue_event, openlcb::EventConfigEntry,
    Name("Blue Channel Event"),
    Description("Event ID base for blue channel (0-255). Controller produces, followers consume. Must end in 00."));

CDI_GROUP_ENTRY(white_event, openlcb::EventConfigEntry,
    Name("White Channel Event"),
    Description("Event ID base for white channel (0-255). Controller produces, followers consume. Must end in 00."));

CDI_GROUP_ENTRY(brightness_event, openlcb::EventConfigEntry,
    Name("Brightness Event"),
    Description("Event ID base for overall brightness (0-255). Controller produces, followers consume. Must end in 00."));

CDI_GROUP_ENTRY(led_count, openlcb::Uint16ConfigEntry,
    Default(120), Min(1), Max(1000),
    Name("LED Count"),
    Description("Number of LEDs in the NeoPixel strip."));

CDI_GROUP_END();

#endif // __RGBWCONFIG_HXX
