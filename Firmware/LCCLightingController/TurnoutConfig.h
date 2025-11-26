#ifndef __TURNOUTCONFIG_HXX
#define __TURNOUTCONFIG_HXX

#include "openlcb/ConfigRepresentation.hxx"

CDI_GROUP(TurnoutConfig);
CDI_GROUP_ENTRY(description, openlcb::StringConfigEntry<16>, Name("Description"),
                Description("User name of this turnout."));
CDI_GROUP_ENTRY(event_rotate_min, openlcb::EventConfigEntry, //
    Name("Normal Event"),
    Description("Receiving this event ID will rotate the servo to its normal "
                "configured point."));
CDI_GROUP_ENTRY(event_rotate_max, openlcb::EventConfigEntry, //
    Name("Reversed Event"),
    Description("Receiving this event ID will rotate the servo to its reversed "
                "configured point."));
#define SERVO_DESCRIPTION_SUFFIX                                               \
    "stop point of the servo, as a percentage: generally 0-100. "              \
    "May be under/over-driven by setting a percentage value "                  \
    "of -99 to 200, respectively."
CDI_GROUP_ENTRY(servo_min_percent, openlcb::Int16ConfigEntry, Default(0), Min(-99),
    Max(200), Name("Servo Normal Stop Point Percentage"),
    Description("Normal-end " SERVO_DESCRIPTION_SUFFIX));
CDI_GROUP_ENTRY(servo_max_percent, openlcb::Int16ConfigEntry, Default(100), Min(-99),
    Max(200), Name("Servo Reversed Stop Point Percentage"),
    Description("Reversed-end " SERVO_DESCRIPTION_SUFFIX));
CDI_GROUP_ENTRY(frog_inverted, openlcb::Uint8ConfigEntry, Default(0), Min(0),
    Max(1), Name("Invert Frog Polarity"),
    Description("Set to 0 for normal frog polarity, 1 to swap what dcc rail it gets set to"));
CDI_GROUP_END(); // TurnoutConfig


#endif // __TURNOUTCONFIG_HXX
