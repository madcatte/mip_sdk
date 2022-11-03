#pragma once

#include <assert.h>

#include "mip_logging.h"

////////////////////////////////////////////////////////////////////////////////
///@brief custom static assert macro to deal with different versions of C/C++
///
///@param expr  Logical expression to check for at compile time
///@param msg   Message to display in compiler output to notify of build failure
///
#if defined(__cplusplus)
#define MIP_STATIC_ASSERT(expr, msg) static_assert(expr, msg)
#elif __STDC_VERSION__ >= 201112L
#define MIP_STATIC_ASSERT(expr, msg) _Static_assert(expr, msg)
#else
#define MIP_STATIC_ASSERT(expr, msg) extern int (*__Static_assert_function (void))[!!sizeof (struct { int __error_if_negative: (expr) ? 2 : -1; })]
#endif

////////////////////////////////////////////////////////////////////////////////
///@brief custom assert macro that will log on failure
///
///@param expr  Logical expression to check for at runtime
///
#if MIP_ENABLE_LOGGING && (!defined(MIP_LOGGING_MAX_LEVEL) || MIP_LOGGING_MAX_LEVEL >= MIP_LOG_LEVEL_FATAL)
#define MIP_ASSERT(expr) \
    if (!(expr)) \
    { \
        MIP_LOG_FATAL("%s:%u Assertion '" #expr "' failed\n", __FILE__, __LINE__); \
        assert(expr); \
    }
#else
#define MIP_ASSERT(expr) assert(expr)
#endif