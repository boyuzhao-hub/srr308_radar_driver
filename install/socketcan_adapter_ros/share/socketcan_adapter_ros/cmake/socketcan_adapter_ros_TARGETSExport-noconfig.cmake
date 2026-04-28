#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "socketcan_adapter_ros::socketcan_adapter_ros" for configuration ""
set_property(TARGET socketcan_adapter_ros::socketcan_adapter_ros APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(socketcan_adapter_ros::socketcan_adapter_ros PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libsocketcan_adapter_ros.so"
  IMPORTED_SONAME_NOCONFIG "libsocketcan_adapter_ros.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS socketcan_adapter_ros::socketcan_adapter_ros )
list(APPEND _IMPORT_CHECK_FILES_FOR_socketcan_adapter_ros::socketcan_adapter_ros "${_IMPORT_PREFIX}/lib/libsocketcan_adapter_ros.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
