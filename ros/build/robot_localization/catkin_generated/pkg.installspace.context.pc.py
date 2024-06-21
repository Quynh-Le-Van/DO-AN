# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include/eigen3;/usr/local/include".split(';') if "${prefix}/include;/usr/include/eigen3;/usr/local/include" != "" else []
PROJECT_CATKIN_DEPENDS = "angles;cmake_modules;diagnostic_msgs;diagnostic_updater;eigen_conversions;geographic_msgs;geometry_msgs;message_filters;message_runtime;nav_msgs;roscpp;sensor_msgs;std_msgs;std_srvs;tf2;tf2_geometry_msgs;tf2_ros".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lekf;-lekf_localization_nodelet;-lfilter_base;-lfilter_utilities;-lnavsat_transform;-lnavsat_transform_nodelet;-lros_filter;-lros_filter_utilities;-lrobot_localization_estimator;-lros_robot_localization_listener;-lukf;-lukf_localization_nodelet;/usr/local/lib/libGeographicLib.so.26.0.0;-lyaml-cpp".split(';') if "-lekf;-lekf_localization_nodelet;-lfilter_base;-lfilter_utilities;-lnavsat_transform;-lnavsat_transform_nodelet;-lros_filter;-lros_filter_utilities;-lrobot_localization_estimator;-lros_robot_localization_listener;-lukf;-lukf_localization_nodelet;/usr/local/lib/libGeographicLib.so.26.0.0;-lyaml-cpp" != "" else []
PROJECT_NAME = "robot_localization"
PROJECT_SPACE_DIR = "/home/proteenteen/Documents/DO-AN/DO-AN/ros/install"
PROJECT_VERSION = "2.7.6"
