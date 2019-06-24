set(COMPONENT_SRCS "app_main.cpp" 
                   "gui.c" 
                   "seg_font.c" 
                   "my_symbol.c" 
                   "QR.c"
                   "espressif.c"
                   "mouse_cursor_icon.c")
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
