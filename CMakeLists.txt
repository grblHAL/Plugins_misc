add_library(misc_plugins INTERFACE)

target_sources(misc_plugins INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}/bltouch.c
 ${CMAKE_CURRENT_LIST_DIR}/eventout.c
 ${CMAKE_CURRENT_LIST_DIR}/pwm_servo_m280.c
 ${CMAKE_CURRENT_LIST_DIR}/rgb_led_m150.c
 ${CMAKE_CURRENT_LIST_DIR}/rgb_led_strips.c
)

target_include_directories(misc_plugins INTERFACE ${CMAKE_CURRENT_LIST_DIR})
