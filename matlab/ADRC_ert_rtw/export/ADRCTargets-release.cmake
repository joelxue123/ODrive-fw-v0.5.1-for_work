#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ADRC::ADRC_objects" for configuration "Release"
set_property(TARGET ADRC::ADRC_objects APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ADRC::ADRC_objects PROPERTIES
  IMPORTED_COMMON_LANGUAGE_RUNTIME_RELEASE ""
  IMPORTED_OBJECTS_RELEASE "${_IMPORT_PREFIX}/ADRC_ert_rtw/objects-Release/ADRC_objects/ADRC.c.obj"
  )

list(APPEND _IMPORT_CHECK_TARGETS ADRC::ADRC_objects )
list(APPEND _IMPORT_CHECK_FILES_FOR_ADRC::ADRC_objects "${_IMPORT_PREFIX}/ADRC_ert_rtw/objects-Release/ADRC_objects/ADRC.c.obj" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
