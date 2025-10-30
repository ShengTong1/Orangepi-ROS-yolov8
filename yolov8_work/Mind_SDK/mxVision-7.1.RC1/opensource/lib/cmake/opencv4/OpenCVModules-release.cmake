#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "opencv_world" for configuration "Release"
set_property(TARGET opencv_world APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_world PROPERTIES
  IMPORTED_LOCATION_RELEASE "/usr1/opensource/opensource/opensource/opencv/_build/../../tmp/opencv/lib/libopencv_world.so.4.11.0"
  IMPORTED_SONAME_RELEASE "libopencv_world.so.411"
  )

list(APPEND _cmake_import_check_targets opencv_world )
list(APPEND _cmake_import_check_files_for_opencv_world "/usr1/opensource/opensource/opensource/opencv/_build/../../tmp/opencv/lib/libopencv_world.so.4.11.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
