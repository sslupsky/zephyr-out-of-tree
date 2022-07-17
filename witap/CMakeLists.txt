if(CONFIG_WITAP)

zephyr_interface_library_named(WITAP)

target_include_directories(WITAP INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
#target_include_directories(WITAP INTERFACE /Users/stevenslupsky/Documents/source/zephyrproject/zephyr-out-of-tree/include)

zephyr_include_directories(.)
zephyr_include_directories(../include)

zephyr_library()
zephyr_library_sources(
    witap_led.c
    witap_power.cpp
    witap_log_backend.c
    CayenneLpp.cpp
    settings_shell.c
    )
zephyr_library_link_libraries(WITAP)
target_link_libraries(WITAP INTERFACE zephyr_interface)
endif()
