#ifndef BASIC_ENGINE_UI_H
#define BASIC_ENGINE_UI_H
#include "common.h"

typedef struct {
	// Shall be used to store state.
} Ui_Ctx;

Ui_Ctx ui_init();
void ui_render(Ui_Ctx* ctx, bool is_active);
void ui_destroy(Ui_Ctx* ctx);

#endif