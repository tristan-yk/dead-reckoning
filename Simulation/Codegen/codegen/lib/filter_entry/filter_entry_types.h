#ifndef FILTER_ENTRY_TYPES_H
#define FILTER_ENTRY_TYPES_H

#include "rtwtypes.h"

#ifndef typedef_struct1_T
#define typedef_struct1_T
typedef struct {
  double accel[3];
  double gyro[3];
  double mag[3];
  double baro;
} struct1_T;
#endif

#ifndef typedef_struct0_T
#define typedef_struct0_T
typedef struct {
  struct1_T sens_filt;
} struct0_T;
#endif

#ifndef typedef_struct3_T
#define typedef_struct3_T
typedef struct {
  double meas[3];
  bool status;
} struct3_T;
#endif

#ifndef typedef_struct4_T
#define typedef_struct4_T
typedef struct {
  double meas;
  bool status;
} struct4_T;
#endif

#ifndef typedef_struct2_T
#define typedef_struct2_T
typedef struct {
  struct3_T accel;
  struct3_T gyro;
  struct3_T mag;
  struct4_T baro;
} struct2_T;
#endif

#ifndef typedef_struct_T
#define typedef_struct_T
typedef struct {
  double P0[100];
  double R_accel[9];
  double R_baro;
  double R_mag;
  double a_fast;
  double a_slow;
  double sa;
  double sbg;
  double sg;
} struct_T;
#endif

#ifndef c_typedef_filter_entryPersisten
#define c_typedef_filter_entryPersisten
typedef struct {
  struct_T params;
  struct_T b_params;
  struct_T c_params;
  struct_T d_params;
  struct_T e_params;
} filter_entryPersistentData;
#endif

#ifndef typedef_filter_entryStackData
#define typedef_filter_entryStackData
typedef struct {
  filter_entryPersistentData *pd;
} filter_entryStackData;
#endif

#endif
