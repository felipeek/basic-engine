#ifndef BASIC_ENGINE_CORE_H
#define BASIC_ENGINE_CORE_H

#include "common.h"

extern int core_parse_arguments(s32 argc, char** argv);
extern int core_init();
extern void core_destroy();
extern void core_update(r32 delta_time);
extern void core_render();
extern void core_input_process(boolean* key_state, r32 delta_time);
extern void core_mouse_change_process(boolean reset, r64 x_pos, r64 y_pos);
extern void core_mouse_click_process(s32 button, s32 action, r64 x_pos, r64 y_pos);
extern void core_scroll_change_process(r64 x_offset, r64 y_offset);
extern void core_window_resize_process(s32 width, s32 height);

#endif