######################################################################
# CMakeLists.txt generated for component ADRC
# Product type: executable
######################################################################
cmake_minimum_required(VERSION 3.12)
project(ADRC)
if(DEFINED ENV{CMAKE_EXPORT_COMPILE_COMMANDS} AND NOT DEFINED CMAKE_EXPORT_COMPILE_COMMANDS)
    set(CMAKE_EXPORT_COMPILE_COMMANDS $ENV{CMAKE_EXPORT_COMPILE_COMMANDS})
endif()


######################################################################
## Path variables
######################################################################
get_filename_component(START_DIR .. ABSOLUTE)
set(MATLAB_ROOT C:/Program\ Files/MATLAB/R2022b CACHE PATH "")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
if(${CMAKE_CURRENT_BINARY_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
    set(BUILDING_IN_SOURCE_DIR TRUE)
    set(BINARY_START_DIR "${START_DIR}")
else()
    set(BUILDING_IN_SOURCE_DIR FALSE)
    set(BINARY_START_DIR "${CMAKE_BINARY_DIR}")
endif()

######################################################################
## Target definition and commands
######################################################################
add_library(ADRC_objects OBJECT ${START_DIR}/ADRC_ert_rtw/ADRC.c)
set_target_properties(ADRC_objects PROPERTIES
    PREFIX ""
    POSITION_INDEPENDENT_CODE ON)
target_compile_features(ADRC_objects PUBLIC c_std_99)
target_compile_definitions(ADRC_objects PRIVATE -DMODEL=ADRC
    -DNUMST=1
    -DNCSTATES=0
    -DHAVESTDIO
    -DMODEL_HAS_DYNAMICALLY_LOADED_SFCNS=0
    -DCLASSIC_INTERFACE=0
    -DALLOCATIONFCN=0
    -DTID01EQ=0
    -DTERMFCN=1
    -DONESTEPFCN=1
    -DMAT_FILE=0
    -DMULTI_INSTANCE_CODE=0
    -DINTEGER_CODE=0
    -DMT=0)
target_link_libraries(ADRC_objects PRIVATE -static
    -m64)
target_include_directories(ADRC_objects PUBLIC $<BUILD_INTERFACE:${START_DIR}>
    $<INSTALL_INTERFACE:$<INSTALL_PREFIX>>
    $<BUILD_INTERFACE:${START_DIR}/ADRC_ert_rtw>
    $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/ADRC_ert_rtw>
    $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/include>
    $<BUILD_INTERFACE:${MATLAB_ROOT}/simulink/include>
    $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/src>
    $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/src/ext_mode/common>
    $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/ert>)
find_library(FOUND_LIBM m)
if(FOUND_LIBM)
   target_link_libraries(ADRC_objects PUBLIC m)
endif()

add_executable(ADRC  ${START_DIR}/ADRC_ert_rtw/ert_main.c)
set_target_properties(ADRC PROPERTIES
    PREFIX ""
    POSITION_INDEPENDENT_CODE ON)
if(CMAKE_CXX_COMPILER)
    set_target_properties(ADRC PROPERTIES LINKER_LANGUAGE CXX)
endif()
target_compile_features(ADRC PUBLIC c_std_99)
target_compile_definitions(ADRC PRIVATE -DMODEL=ADRC
    -DNUMST=1
    -DNCSTATES=0
    -DHAVESTDIO
    -DMODEL_HAS_DYNAMICALLY_LOADED_SFCNS=0
    -DCLASSIC_INTERFACE=0
    -DALLOCATIONFCN=0
    -DTID01EQ=0
    -DTERMFCN=1
    -DONESTEPFCN=1
    -DMAT_FILE=0
    -DMULTI_INSTANCE_CODE=0
    -DINTEGER_CODE=0
    -DMT=0)
target_link_libraries(ADRC PRIVATE -static
    -m64)
target_include_directories(ADRC PRIVATE ${START_DIR}
    ${START_DIR}/ADRC_ert_rtw
    ${MATLAB_ROOT}/extern/include
    ${MATLAB_ROOT}/simulink/include
    ${MATLAB_ROOT}/rtw/c/src
    ${MATLAB_ROOT}/rtw/c/src/ext_mode/common
    ${MATLAB_ROOT}/rtw/c/ert)
target_link_libraries(ADRC PRIVATE ADRC_objects)
find_library(FOUND_LIBM m)
if(FOUND_LIBM)
   target_link_libraries(ADRC PRIVATE m)
endif()
if(WIN32)
   target_link_libraries(ADRC PRIVATE kernel32
       ws2_32
       mswsock
       advapi32)
endif()

######################################################################
## Extract debug symbols
######################################################################
if(APPLE AND (CMAKE_C_COMPILER_ID STREQUAL "AppleClang"))
    string(APPEND _dsymutilcmd "$<IF:$<CONFIG:DEBUG,RELWITHDEBINFO>,"
                "xcrun;dsymutil;$<TARGET_FILE:ADRC>;--flat,"
                "true"
        >)
    add_custom_command(TARGET ADRC
            POST_BUILD
            COMMAND "${_dsymutilcmd}"
            COMMAND_EXPAND_LISTS
            BYPRODUCTS ADRC.dwarf)
    install(FILES $<TARGET_FILE_DIR:ADRC>/ADRC.dwarf DESTINATION . OPTIONAL CONFIGURATIONS Debug RelWithDebInfo)
endif()
if(CMAKE_C_LINKER_SUPPORTS_PDB)
    install(FILES $<TARGET_PDB_FILE:ADRC> DESTINATION . OPTIONAL CONFIGURATIONS Debug RelWithDebInfo)
endif()


######################################################################
## Target install and export rules
######################################################################
install(TARGETS ADRC_objects EXPORT ADRCTargets OBJECTS DESTINATION ADRC_ert_rtw)
install(TARGETS ADRC RUNTIME DESTINATION . OPTIONAL)
install(EXPORT ADRCTargets NAMESPACE ADRC:: DESTINATION ADRC_ert_rtw/export)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/ADRC.cmake 
 "include(\"\${CMAKE_CURRENT_LIST_DIR}/ADRCTargets.cmake\")" \n
    "target_include_directories(ADRC::ADRC_objects INTERFACE \n"
    "    \${MATLAB_ROOT}/extern/include" \n 
    "    \${MATLAB_ROOT}/simulink/include" \n 
    "    \${MATLAB_ROOT}/rtw/c/src" \n 
    "    \${MATLAB_ROOT}/rtw/c/src/ext_mode/common" \n 
    "    \${MATLAB_ROOT}/rtw/c/ert)" \n )
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/ADRC.cmake DESTINATION ADRC_ert_rtw/export)

######################################################################
## Build success message
######################################################################
add_custom_command(TARGET ADRC POST_BUILD
     COMMAND ${CMAKE_COMMAND} -E cmake_echo_color --cyan "\\#\\#\\# Created executable: $<TARGET_FILE:ADRC>")

######################################################################
## Call toolchain hook function if defined
######################################################################
if(COMMAND toolchain_target_hook)
    toolchain_target_hook(ADRC)
endif()
