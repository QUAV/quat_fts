#ifndef FTS_OUTPUT_H
#define FTS_OUTPUT_H

#include <kernel/dispatcher.h>

void output_initialize(dispatcher_context_t *context);
void output_disarm();
void output_rearm();
void output_fire();

#endif // FTS_OUTPUT_H


