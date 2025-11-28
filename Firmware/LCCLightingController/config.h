#ifndef __CONFIG_H
#define __CONFIG_H

#include "openlcb/ConfiguredConsumer.hxx"
#include "openlcb/ConfiguredProducer.hxx"
#include "openlcb/ConfigRepresentation.hxx"
#include "openlcb/MemoryConfig.hxx"

#include "RGBWConfig.h"

namespace openlcb
{

/// Defines the identification information for the node. The arguments are:
///
/// - 4 (version info, always 4 by the standard
/// - Manufacturer name
/// - Model name
/// - Hardware version
/// - Software version
///
/// This data will be used for all purposes of the identification:
///
/// - the generated cdi.xml will include this data
/// - the Simple Node Ident Info Protocol will return this data
/// - the ACDI memory space will contain this data.
extern const SimpleNodeStaticValues SNIP_STATIC_DATA;

constexpr uint8_t NUM_RGBW_STRIPS = 1;

/// Declares a repeated group of RGBW strip configurations
using RGBWGroup = RepeatedGroup<RGBWConfig, NUM_RGBW_STRIPS>;

/// Initial values for RGBW configuration
constexpr uint64_t RGBW_EVENT_INIT[] = {
    0x0501010122600000ULL,  // Red base
    0x0501010122600100ULL,  // Green base
    0x0501010122600200ULL,  // Blue base
    0x0501010122600300ULL,  // White base
    0x0501010122600400ULL   // Brightness base
};

/// Modify this value every time the EEPROM needs to be cleared on the node
/// after an update. Increment when config structure changes (fields added/removed).
static constexpr uint16_t CANONICAL_VERSION = 0x104;

/// Defines the main segment in the configuration CDI. This is laid out at
/// origin 128 to give space for the ACDI user data at the beginning.
CDI_GROUP(IoBoardSegment, Segment(MemoryConfigDefs::SPACE_CONFIG), Offset(128));
/// Each entry declares the name of the current entry, then the type and then
/// optional arguments list.
CDI_GROUP_ENTRY(rgbw_strips, RGBWGroup, Name("RGBW Light Strips"), RepName("Strip"));
CDI_GROUP_ENTRY(internal_config, InternalConfigData);
CDI_GROUP_END();

/// This segment is only needed temporarily until there is program code to set
/// the ACDI user data version byte.
CDI_GROUP(VersionSeg, Segment(MemoryConfigDefs::SPACE_CONFIG),
    Name("Version information"));
CDI_GROUP_ENTRY(acdi_user_version, Uint8ConfigEntry,
    Name("ACDI User Data version"), Description("Set to 2 and do not change."));
CDI_GROUP_END();

/// The main structure of the CDI. ConfigDef is the symbol we use in main.cxx
/// to refer to the configuration defined here.
CDI_GROUP(ConfigDef, MainCdi());
/// Adds the <identification> tag with the values from SNIP_STATIC_DATA above.
CDI_GROUP_ENTRY(ident, Identification);
/// Adds an <acdi> tag.
CDI_GROUP_ENTRY(acdi, Acdi);
/// Adds a segment for changing the values in the ACDI user-defined
/// space. UserInfoSegment is defined in the system header.
CDI_GROUP_ENTRY(userinfo, UserInfoSegment);
/// Adds the main configuration segment.
CDI_GROUP_ENTRY(seg, IoBoardSegment);
/// Adds the versioning segment.
CDI_GROUP_ENTRY(version, VersionSeg);
CDI_GROUP_END();

} // namespace openlcb



#endif // __CONFIG_H
