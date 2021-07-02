//
// Created by jonas on 6/30/21.
//

#ifndef BIRDEYE_VIZ_VISIBILITY_CONTROL_HPP
#define BIRDEYE_VIZ_VISIBILITY_CONTROL_HPP

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
    #ifdef __GNUC__
        #define BIRDEYE_VIZ_EXPORT __attribute__((dllexport))
        #define BIRDEYE_VIZ_IMPORT __attribute__((dllimport))
    #else
        #define BIRDEYE_VIZ_EXPORT __declspec(dllexport)
        #define BIRDEYE_VIZ_IMPORT __declspec(dllimport)
    #endif
    #ifdef BIRDEYE_VIZ_BUILDING_LIBRARY
        #define BIRDEYE_VIZ_PUBLIC BIRDEYE_VIZ_EXPORT
    #else
        #define BIRDEYE_VIZ_PUBLIC BIRDEYE_VIZ_IMPORT
    #endif
    #define BIRDEYE_VIZ_PUBLIC_TYPE BIRDEYE_VIZ_PUBLIC
    #define BIRDEYE_VIZ_LOCAL
#else
    #define BIRDEYE_VIZ_EXPORT __attribute__((visibility("default")))
    #define BIRDEYE_VIZ_IMPORT
    #if __GNUC__ >= 4
        #define BIRDEYE_VIZ_PUBLIC __attribute__((visibility("default")))
        #define BIRDEYE_VIZ_LOCAL __attribute__((visibility("hidden")))
    #else
        #define BIRDEYE_VIZ_PUBLIC
        #define BIRDEYE_VIZ_LOCAL
    #endif
    #define BIRDEYE_VIZ_PUBLIC_TYPE
#endif

#endif // BIRDEYE_VIZ_VISIBILITY_CONTROL_HPP
