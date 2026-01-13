#ifndef __RGBWCONFIG_H
#define __RGBWCONFIG_H

#include "openlcb/ConfigRepresentation.hxx"

CDI_GROUP(RGBWConfig);
CDI_GROUP_ENTRY(description, openlcb::StringConfigEntry<16>, 
    Name("Description"),
    Description("User name of this RGBW light strip."));

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

CDI_GROUP_ENTRY(sync_interval, openlcb::Uint16ConfigEntry,
    Default(3), Min(0), Max(60),
    Name("Sync Interval (seconds)"),
    Description("Controller only: How often to broadcast current RGBW state to keep followers in sync. Set to 0 to disable."));

CDI_GROUP_ENTRY(startup_delay, openlcb::Uint16ConfigEntry,
    Default(5), Min(0), Max(30),
    Name("Startup Delay (seconds)"),
    Description("Controller only: Delay before starting fade-in animation. Allows LCC bus to settle after power-on. Set to 0 to disable."));

CDI_GROUP_END();

#endif // __RGBWCONFIG_H
