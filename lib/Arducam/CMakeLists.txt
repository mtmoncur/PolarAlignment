# aux_source_directory(. DIR_Arducam_SRCS)

# # 生成链接库
# add_library(Arducam ${DIR_Arducam_SRCS})

add_library(Arducam INTERFACE)


target_sources(Arducam INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/hm01b0.c
)

target_include_directories(Arducam INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(Arducam INTERFACE
    cmsis_core
    hardware_dma
    hardware_i2c
    hardware_pio
    hardware_pwm
)