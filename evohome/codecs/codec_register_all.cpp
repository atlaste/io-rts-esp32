/**
 * @file codec_register_all.cpp
 * @brief Thin facade that wires every category's register_xxx_codecs()
 *        into the global registry.
 *
 * Codec definitions live in category-specific files (codecs_heat.cpp,
 * codecs_hvac.cpp, codecs_zoneconfig.cpp, codecs_opentherm.cpp) so each
 * file stays small and the dependency graph between codecs and field
 * types remains obvious.
 *
 * To add a NEW codec, follow the recipe at the top of the appropriate
 * category file. To add a whole new CATEGORY, add another register_xxx
 * declaration here and a new codecs_xxx.cpp source file with the
 * matching definition (and update main/CMakeLists.txt).
 */

#include "../ramses_codec.h"

namespace evohome::codecs
{
    void register_heat_codecs(CodecRegistry &reg);
    void register_hvac_codecs(CodecRegistry &reg);
    void register_zoneconfig_codecs(CodecRegistry &reg);
    void register_opentherm_codecs(CodecRegistry &reg);
} // namespace evohome::codecs

namespace evohome
{
    void register_all_codecs(CodecRegistry &reg)
    {
        codecs::register_heat_codecs(reg);
        codecs::register_hvac_codecs(reg);
        codecs::register_zoneconfig_codecs(reg);
        codecs::register_opentherm_codecs(reg);
    }
} // namespace evohome
