cmake_minimum_required(VERSION 3.16)

# Library with core flight software components
add_library(aquila_fsw
    src/config.cpp
    src/state.cpp
    src/sensors.cpp
    src/estimator.cpp
    src/controller.cpp
    src/modes.cpp
    src/logger.cpp
)

#target_include_directories(<target> [SYSTEM] [AFTER|BEFORE] <INTERFACE|PUBLIC|PRIVATE> [items1...] [<INTERFACE|PUBLIC|PRIVATE> [items2...] ...])
target_include_directories(aquila_fsw
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(aquila_fsw PRIVATE -Wall -Wextra -Wpedantic)

# Main executable that runs the FSW loop
add_executable(aquila_fsw_main
    src/main_loop.cpp
)

target_link_libraries(aquila_fsw_main
    PRIVATE
        aquila_fsw
)

# ---- Tests ----

if(BUILD_TESTING)
    # Simple test executable for now
    add_executable(aquila_fsw_tests
        tests/test_dummy.cpp
    )

    target_link_libraries(aquila_fsw_tests
        PRIVATE
            aquila_fsw
    )

    add_test(NAME aquila_fsw_tests COMMAND aquila_fsw_tests)
endif()