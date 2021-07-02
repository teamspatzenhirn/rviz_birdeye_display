cmake_minimum_required(VERSION 3.8)
project(birdeye_viz)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

find_package(spatz_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(OpenCV REQUIRED)


set(birdeye_viz_headers_to_moc
        include/birdeye_viz/BirdeyeDisplay.hpp
        )

foreach (header "${birdeye_viz_headers_to_moc}")
    qt5_wrap_cpp(birdeye_viz_moc_files "${header}")
endforeach ()

set(birdeye_viz_source_files
        src/birdeye_viz/BirdeyeDisplay.cpp
        )

add_library(birdeye_viz SHARED
        ${birdeye_viz_moc_files}
        ${birdeye_viz_source_files}
        )

target_include_directories(birdeye_viz PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${Qt5Widgets_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        )

target_link_libraries(birdeye_viz PUBLIC
        rviz_ogre_vendor::OgreMain
        rviz_ogre_vendor::OgreOverlay
        opencv_core
        opencv_imgproc
        opencv_highgui
        )

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(birdeye_viz PRIVATE "BIRDEYE_VIZ_BUILDING_LIBRARY")

# prevent pluginlib from using boost
target_compile_definitions(birdeye_viz PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_target_dependencies(birdeye_viz
        PUBLIC
        sensor_msgs
        spatz_interfaces
        rviz_common
        rviz_rendering
        )

ament_export_include_directories(include)
ament_export_targets(birdeye_viz HAS_LIBRARY_TARGET)
ament_export_dependencies(
        rviz_common
        sensor_msgs
        spatz_interfaces
        rviz_ogre_vendor
)

install(
        TARGETS birdeye_viz
        EXPORT birdeye_viz
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

install(
        DIRECTORY include/
        DESTINATION include
)

ament_package(
        CONFIG_EXTRAS "birdeye_viz-extras.cmake"
)
