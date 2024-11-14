#pragma once

#include "mip_metadata.hpp"

#include <mip/mip_descriptors.hpp>

#include <set>
#include <initializer_list>


namespace mip::metadata
{

class Definitions
{
    struct Less
    {
        inline bool operator()(const FieldInfo *a, const FieldInfo *b) const
        {
            return a->descriptor < b->descriptor;
        }
        inline bool operator()(const FieldInfo *a, CompositeDescriptor desc) const
        {
            return a->descriptor < desc;
        }
        inline bool operator()(CompositeDescriptor desc, const FieldInfo *a) const
        {
            return desc < a->descriptor;
        }
        using is_transparent = void;
    };

    using Container = std::set<const FieldInfo *, Less>;

public:
    using FieldInfoSpan  = microstrain::Span<const FieldInfo* const>;
    using FieldInfoSpans = microstrain::Span<const FieldInfoSpan* const>;

    Definitions() = default;
    Definitions(const FieldInfoSpans &fields) { registerDefinitions(fields); }

    void registerField(const FieldInfo* field);
    void registerDefinitions(const FieldInfoSpan &fields);
    void registerDefinitions(const FieldInfoSpans &fields);

    const FieldInfo* findField(mip::CompositeDescriptor descriptor) const;


private:
    Container mFields;
};




} // namespace mip::metadata
