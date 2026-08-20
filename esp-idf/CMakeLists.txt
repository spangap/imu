# spangap/imu — generic motion-sensing task.
#
# Cross-straddle deps flow through ${SPANGAP_REQUIRES} only (the staging lint
# rejects a hand-written sibling repo name). The straddle owns the chip; a
# board contributes only Kconfig pin data.
include(${CMAKE_CURRENT_LIST_DIR}/spangap_requires.cmake)
idf_component_register(
    SRCS
        "imu.cpp"
        # generated services trampoline (+ any conditional/<name>/ code)
        ${SPANGAP_CONDITIONAL_SRCS}
    INCLUDE_DIRS "." ${SPANGAP_CONDITIONAL_INCLUDE_DIRS}
    REQUIRES
        ${SPANGAP_REQUIRES}
        driver
        esp_timer
)
