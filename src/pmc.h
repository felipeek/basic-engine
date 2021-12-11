#ifndef BASIC_ENGINE_PMC_H
#define BASIC_ENGINE_PMC_H

#include "common.h"
#include "gm.h"
#include "graphics.h"
#include <hash_map.h>

Hash_Map pmc_map;

typedef struct {
	void* e1;
	void* e2;
} PMC_Map_Key;

typedef struct {
	Entity* e1;
	Entity* e2;
	vec3 r1_wc;		// world coordinates
	vec3 r2_wc;		// world coordinates
	vec3 r1_lc;		// local coordinates
	vec3 r2_lc;		// local coordinates
	vec3 normal;
	r32 lambda_n;
	r32 lambda_t;
} PMC_Contact;

typedef struct {
	PMC_Contact* contacts;
	PMC_Contact* reserve;
} PMC;


// @TODO IMPORTANT: The cache needs to be per BOUNDING BOX, not per ENTITY! <<<<
void pmc_init();
void pmc_destroy();
void pmc_render(Perspective_Camera* camera);
void pmc_update();
void pmc_add(PMC_Contact contact);
void pmc_clear(Entity* e1, Entity* e2);
void pmc_clear_all();
void pmc_perturb(Entity* e1, Entity* e2, vec3 normal);

#endif