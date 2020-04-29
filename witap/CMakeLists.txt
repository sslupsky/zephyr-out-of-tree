if(CONFIG_WITAP)

zephyr_interface_library_named(WITAP)

target_include_directories(WITAP INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

zephyr_include_directories(.)

zephyr_library()
zephyr_library_sources(
    witap_led.c
    witap_power.cpp
    witap_log_backend.c
    )
zephyr_library_link_libraries(WITAP)
target_link_libraries(WITAP INTERFACE zephyr_interface)
endif()
