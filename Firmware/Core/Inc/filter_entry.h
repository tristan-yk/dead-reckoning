#ifndef FILTER_ENTRY_H
#define FILTER_ENTRY_H

#include "filter_entry_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

void filter_entry(filter_entryStackData *SD, float x[10], float b_P[100],
                  struct0_T *mem, float dt, const struct2_T *sens_in,
                  bool is_init);

void filter_entry_initialize(filter_entryStackData *SD);

#ifdef __cplusplus
}
#endif

#endif
