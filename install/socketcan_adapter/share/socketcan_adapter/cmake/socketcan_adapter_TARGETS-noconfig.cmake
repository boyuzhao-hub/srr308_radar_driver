#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "socketcan_adapter::socketcan_adapter" for configuration ""
set_property(TARGET socketcan_adapter::socketcan_adapter APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(socketcan_adapter::socketcan_adapter PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libsocketcan_adapter.so"
  IMPORTED_SONAME_NOCONFIG "libsocketcan_adapter.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS socketcan_adapter::socketcan_adapter )
list(APPEND _IMPORT_CHECK_FILES_FOR_socketcan_adapter::socketcan_adapter "${_IMPORT_PREFIX}/lib/libsocketcan_adapter.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
