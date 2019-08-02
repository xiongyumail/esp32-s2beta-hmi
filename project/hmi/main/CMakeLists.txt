set(COMPONENT_SRCS "app_main.cpp" 
                   "gui.c" 
                   "seg_font.c" 
                   "my_symbol.c" 
                   "logo.c"
                   "espressif.c"
                   "mouse_cursor_icon.c"
                   "file_server.c"
                   EMBED_FILES "favicon.ico" "upload_script.html")
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
