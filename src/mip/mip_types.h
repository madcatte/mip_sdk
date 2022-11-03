#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "mip_assert.h"

// Note that it is important to have the "mip" directory in this include, otherwise the correct file will not be included during local development
#include "mip/mip_version.h"

#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {
#endif

// Used like a signed version of size_t
typedef int_least16_t remaining_count;

///@brief Type used for packet timestamps and timeouts.
///
/// Timestamps must be monotonic except for overflow at the maximum value back to 0.
/// The units can be anything, but typically are milliseconds. Choose a long enough
/// unit so that consecutive calls to the parser will not exceed half of the maximum
/// value for this type. For milliseconds, the time to overflow is approximately 50
/// days, so the parser should be invoked at least every 25 days. Failure to observe
/// this requirement may result in false timeouts or delays in getting parsed packets.
///
#ifdef MIP_TIMESTAMP_TYPE
typedef MIP_TIMESTAMP_TYPE timestamp_type;
MIP_STATIC_ASSERT( sizeof(timestamp_type) >= 8 || (timestamp_type)-1 > 0, "MIP_TIMESTAMP_TYPE must be unsigned unless 64 bits.");
#else
typedef uint64_t timestamp_type;
#endif

typedef timestamp_type timeout_type;

// Verify that the type used for the timestamp at compile time is compatible with the type used at runtime
#ifdef MIP_SDK_COMPILED_WITH_MIP_TIMESTAMP_TYPE
MIP_STATIC_ASSERT(sizeof(MIP_SDK_COMPILED_WITH_MIP_TIMESTAMP_TYPE) == sizeof(timestamp_type), "Library was compiled with different timestamp type.");
#endif

#ifdef __cplusplus

} // extern "C"
} // namespace C

using RemainingCount = C::remaining_count;
using Timestamp      = C::timestamp_type;
using Timeout        = C::timeout_type;

} // namespace mip

#endif
