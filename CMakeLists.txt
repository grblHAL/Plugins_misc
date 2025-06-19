add_library(misc_plugins INTERFACE)

target_sources(misc_plugins INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}/bltouch.c
 ${CMAKE_CURRENT_LIST_DIR}/esp_at.c
 ${CMAKE_CURRENT_LIST_DIR}/eventout.c
 ${CMAKE_CURRENT_LIST_DIR}/feed_override_m220.c
 ${CMAKE_CURRENT_LIST_DIR}/homing_pulloff.c
 ${CMAKE_CURRENT_LIST_DIR}/pca9654e.c
 ${CMAKE_CURRENT_LIST_DIR}/mcp3221.c
 ${CMAKE_CURRENT_LIST_DIR}/mcp4725.c
 ${CMAKE_CURRENT_LIST_DIR}/pwm_servo_m280.c
 ${CMAKE_CURRENT_LIST_DIR}/rgb_led_m150.c
 ${CMAKE_CURRENT_LIST_DIR}/rgb_led_strips.c
 ${CMAKE_CURRENT_LIST_DIR}/tooltable.c
)

target_include_directories(misc_plugins INTERFACE ${CMAKE_CURRENT_LIST_DIR})
