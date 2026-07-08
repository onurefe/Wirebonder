cmake_minimum_required(VERSION 3.20)

# On macOS, AppleClang may not find C++ standard library headers automatically.
# Resolve the SDK path and prepend it so both GTest and our sources compile.
if(APPLE)
    execute_process(
        COMMAND xcrun --show-sdk-path
        OUTPUT_VARIABLE APPLE_SDK_PATH
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    include_directories(BEFORE SYSTEM "${APPLE_SDK_PATH}/usr/include/c++/v1")
endif()

include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(firmware_tests
    test_pid_controller.cpp
    test_leaky_integrator_controller.cpp
    test_queue.cpp
    test_iq_demodulator.cpp
    test_transducer_analyzer.cpp
    stubs/hal_stubs.cpp
    ${CMAKE_SOURCE_DIR}/Core/Src/transducer_analyzer.cpp
    ${CMAKE_SOURCE_DIR}/Core/Src/leaky_integrator_controller.cpp
    ${CMAKE_SOURCE_DIR}/Core/Src/pid_controller.cpp
    ${CMAKE_SOURCE_DIR}/Core/Src/adc_service.cpp
    ${CMAKE_SOURCE_DIR}/Core/Src/dac_service.cpp
    ${CMAKE_SOURCE_DIR}/Core/Src/pll_module.cpp
)

target_compile_features(firmware_tests PRIVATE cxx_std_17)

target_include_directories(firmware_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/stubs
    ${CMAKE_SOURCE_DIR}/Core/Inc
)

target_compile_definitions(firmware_tests PRIVATE UNIT_TEST)

target_compile_options(firmware_tests PRIVATE
    -Wall -Wextra
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
)

target_link_libraries(firmware_tests PRIVATE GTest::gtest_main GTest::gmock m)

include(GoogleTest)
gtest_discover_tests(firmware_tests
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    PROPERTIES ENVIRONMENT "DYLD_LIBRARY_PATH=${CMAKE_CURRENT_BINARY_DIR}"
)
