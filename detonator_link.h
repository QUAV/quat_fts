#ifndef DETONATOR_LINK_H
#define DETONATOR_LINK_H

#include <kernel/list.h>
#include <kernel/dispatcher.h>

typedef void (*detonator_link_fire_func_t)();

void detonator_link_initialize(dispatcher_context_t *context, detonator_link_fire_func_t fire_func);

#endif // DETONATOR_LINK_H