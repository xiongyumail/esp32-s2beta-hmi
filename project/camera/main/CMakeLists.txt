set(COMPONENT_SRCS "main.c"
                   "gui.c"
                   "seg_font.c" 
                   "i2s_camera_test.c" 
                   "i2s_dma_debug.c" 
                   "sccb.c" 
                   "uart_res.c")
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
