//
// Created by jonas on 6/30/21.
//

#ifndef rviz_birdeye_display_VISIBILITY_CONTROL_HPP
#define rviz_birdeye_display_VISIBILITY_CONTROL_HPP

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
    #ifdef __GNUC__
        #define rviz_birdeye_display_EXPORT __attribute__((dllexport))
        #define rviz_birdeye_display_IMPORT __attribute__((dllimport))
    #else
        #define rviz_birdeye_display_EXPORT __declspec(dllexport)
        #define rviz_birdeye_display_IMPORT __declspec(dllimport)
    #endif
    #ifdef rviz_birdeye_display_BUILDING_LIBRARY
        #define rviz_birdeye_display_PUBLIC rviz_birdeye_display_EXPORT
    #else
        #define rviz_birdeye_display_PUBLIC rviz_birdeye_display_IMPORT
    #endif
    #define rviz_birdeye_display_PUBLIC_TYPE rviz_birdeye_display_PUBLIC
    #define rviz_birdeye_display_LOCAL
#else
    #define rviz_birdeye_display_EXPORT __attribute__((visibility("default")))
    #define rviz_birdeye_display_IMPORT
    #if __GNUC__ >= 4
        #define rviz_birdeye_display_PUBLIC __attribute__((visibility("default")))
        #define rviz_birdeye_display_LOCAL __attribute__((visibility("hidden")))
    #else
        #define rviz_birdeye_display_PUBLIC
        #define rviz_birdeye_display_LOCAL
    #endif
    #define rviz_birdeye_display_PUBLIC_TYPE
#endif

#endif // rviz_birdeye_display_VISIBILITY_CONTROL_HPP
