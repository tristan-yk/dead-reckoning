#include "filter_entry.h"
#include "filter_entry_types.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

static const struct_T r = {
    {0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     1.0E-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     1.0E-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     1.0E-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.01,   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.01},
    {0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.05},
    4.0,
    0.0027415567780803771,
    0.01,
    0.04,
    1.0,
    0.0001,
    0.001};

static int asr_s32(int u, unsigned int n);

static float b_atan2(float y, float x);

static float b_log2(float x);

static float b_norm(const float x[3]);

static void b_rotateRight(int n, float z[16], int iz0, const float cs[6],
                          int ic0, int is0);

static void b_xzlascl(float cfrom, float cto, int m, float A[3], int iA0);

static float c_norm(const float x[4]);

static void ekf_dynamics_init(filter_entryStackData *SD);

static void ekf_innov_accel_init(filter_entryStackData *SD);

static void ekf_innov_baro_init(filter_entryStackData *SD);

static void ekf_innov_mag_init(filter_entryStackData *SD);

static void expm(float A[16], float F[16]);

static void filter_init_init(filter_entryStackData *SD);

static int getExpmParams(const float A[16], float A2[16], float A4[16],
                         float A6[16], float *s);

static void inv(const float x[16], float y[16]);

static void mpower(const float b_a[16], float b, float c[16]);

static void mrdiv(const float A[30], const float b_B[9], float d_Y[30]);

static void recomputeBlockDiag(const float A[16], float F[16],
                               const int blockFormat[3]);

static void rotateRight(int n, float z[16], int iz0, const float cs[6], int ic0,
                        int is0);

static float rt_powf_snf(float u0, float u1);

static float xdlaev2(float b_a, float b, float c, float *rt2, float *cs1,
                     float *sn1);

static float xnrm2(int n, const float x[16], int ix0);

static int xsyheev(float A[16], float b_W[4]);

static int xzgetrf(float A[16], int ipiv[4]);

static float xzlartg(float b_f, float g, float *sn, float *b_r);

static void xzlascl(float cfrom, float cto, int m, float A[4], int iA0);

static int xzsteqr(float d[4], float e[3], float z[16]);

static void xzsyhetrd(float A[16], float b_D[4], float b_E[3], float tau[3]);

static int asr_s32(int u, unsigned int n)
{
  int y;
  if (u >= ((int)((signed char)0))) {
    y = (int)((unsigned int)(((unsigned int)u) >> n));
  } else {
    y = (-((int)((unsigned int)(((unsigned int)((int)(-1 - u))) >> n)))) - 1;
  }
  return y;
}

static float b_atan2(float y, float x)
{
  float b_r;
  if ((rtIsNaNF(y)) || (rtIsNaNF(x))) {
    b_r = rtNaNF;
  } else if ((rtIsInfF(y)) && (rtIsInfF(x))) {
    int b_i;
    int i1;
    if (y > 0.0F) {
      b_i = 1;
    } else {
      b_i = -1;
    }
    if (x > 0.0F) {
      i1 = 1;
    } else {
      i1 = -1;
    }
    b_r = atan2f((float)b_i, (float)i1);
  } else if (x == 0.0F) {
    if (y > 0.0F) {
      b_r = RT_PIF / 2.0F;
    } else if (y < 0.0F) {
      b_r = -(RT_PIF / 2.0F);
    } else {
      b_r = 0.0F;
    }
  } else {
    b_r = atan2f(y, x);
  }
  return b_r;
}

static float b_log2(float x)
{
  float b_f;
  int eint;
  if (x == 0.0F) {
    b_f = rtMinusInfF;
  } else if (x < 0.0F) {
    b_f = rtNaNF;
  } else if ((!rtIsInfF(x)) && (!rtIsNaNF(x))) {
    b_f = frexpf(x, &eint);
    if (b_f == 0.5F) {
      b_f = ((float)eint) - 1.0F;
    } else if ((eint == ((int)((signed char)1))) && (b_f < 0.75F)) {
      b_f = logf(2.0F * b_f) / 0.693147182F;
    } else {
      b_f = (logf(b_f) / 0.693147182F) + ((float)eint);
    }
  } else {
    b_f = x;
  }
  return b_f;
}

static float b_norm(const float x[3])
{
  float absxk;
  float scale;
  float t;
  float y;
  scale = 1.29246971E-26F;
  absxk = fabsf(x[0]);
  if (absxk > 1.29246971E-26F) {
    y = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    y = t * t;
  }
  absxk = fabsf(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = ((y * t) * t) + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = fabsf(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = ((y * t) * t) + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  return scale * sqrtf(y);
}

static void b_rotateRight(int n, float z[16], int iz0, const float cs[6],
                          int ic0, int is0)
{
  int j;
  for (j = 0; j <= (n - 2); j++) {
    float ctemp;
    float stemp;
    int offsetj;
    int offsetjp1;
    ctemp = cs[(ic0 + j) - 1];
    stemp = cs[(is0 + j) - 1];
    offsetj = ((j * 4) + iz0) - 2;
    offsetjp1 = (((j + 1) * 4) + iz0) - 2;
    if ((ctemp != 1.0F) || (stemp != 0.0F)) {
      float b_f;
      float temp;
      temp = z[offsetjp1 + 1];
      b_f = z[offsetj + 1];
      z[offsetjp1 + 1] = (ctemp * temp) - (stemp * b_f);
      b_f = (stemp * temp) + (ctemp * b_f);
      z[offsetj + 1] = b_f;
      temp = z[offsetjp1 + 2];
      b_f = z[offsetj + 2];
      z[offsetjp1 + 2] = (ctemp * temp) - (stemp * b_f);
      b_f = (stemp * temp) + (ctemp * b_f);
      z[offsetj + 2] = b_f;
      temp = z[offsetjp1 + 3];
      b_f = z[offsetj + 3];
      z[offsetjp1 + 3] = (ctemp * temp) - (stemp * b_f);
      b_f = (stemp * temp) + (ctemp * b_f);
      z[offsetj + 3] = b_f;
      temp = z[offsetjp1 + 4];
      b_f = z[offsetj + 4];
      z[offsetjp1 + 4] = (ctemp * temp) - (stemp * b_f);
      b_f = (stemp * temp) + (ctemp * b_f);
      z[offsetj + 4] = b_f;
    }
  }
}

static void b_xzlascl(float cfrom, float cto, int m, float A[3], int iA0)
{
  float cfromc;
  float ctoc;
  int b_i;
  bool notdone;
  cfromc = cfrom;
  ctoc = cto;
  notdone = true;
  while (notdone) {
    float cfrom1;
    float cto1;
    float mul;
    cfrom1 = cfromc * 1.97215226E-31F;
    cto1 = ctoc / 5.0706024E+30F;
    if ((fabsf(cfrom1) > fabsf(ctoc)) && (ctoc != 0.0F)) {
      mul = 1.97215226E-31F;
      cfromc = cfrom1;
    } else if (fabsf(cto1) > fabsf(cfromc)) {
      mul = 5.0706024E+30F;
      ctoc = cto1;
    } else {
      mul = ctoc / cfromc;
      notdone = false;
    }
    for (b_i = 0; b_i < m; b_i++) {
      int c_i;
      c_i = (iA0 + b_i) - 1;
      A[c_i] *= mul;
    }
  }
}

static float c_norm(const float x[4])
{
  float absxk;
  float scale;
  float t;
  float y;
  scale = 1.29246971E-26F;
  absxk = fabsf(x[0]);
  if (absxk > 1.29246971E-26F) {
    y = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    y = t * t;
  }
  absxk = fabsf(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = ((y * t) * t) + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = fabsf(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = ((y * t) * t) + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = fabsf(x[3]);
  if (absxk > scale) {
    t = scale / absxk;
    y = ((y * t) * t) + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  return scale * sqrtf(y);
}

static void ekf_dynamics_init(filter_entryStackData *SD)
{
  SD->pd->b_params = r;
}

static void ekf_innov_accel_init(filter_entryStackData *SD)
{
  SD->pd->c_params = r;
}

static void ekf_innov_baro_init(filter_entryStackData *SD)
{
  SD->pd->e_params = r;
}

static void ekf_innov_mag_init(filter_entryStackData *SD)
{
  SD->pd->d_params = r;
}

static void expm(float A[16], float F[16])
{
  float A2[16];
  float A6[16];
  float V[16];
  float exptj;
  float s;
  int b_k;
  int c_i;
  int k;
  bool recomputeDiags;
  recomputeDiags = true;
  for (k = 0; k < 16; k++) {
    if (recomputeDiags) {
      exptj = A[k];
      if ((rtIsInfF(exptj)) || (rtIsNaNF(exptj))) {
        recomputeDiags = false;
      }
    } else {
      recomputeDiags = false;
    }
  }
  if (!recomputeDiags) {
    for (k = 0; k < 16; k++) {
      F[k] = rtNaNF;
    }
  } else {
    int b_i;
    int exitg1;
    int j;
    bool exitg2;
    recomputeDiags = true;
    j = 0;
    exitg2 = false;
    while ((!exitg2) && (j < ((int)((signed char)4)))) {
      b_i = 0;
      do {
        exitg1 = 0;
        if (b_i < ((int)((signed char)4))) {
          if ((b_i != j) && (!(A[b_i + (4 * j)] == 0.0F))) {
            recomputeDiags = false;
            exitg1 = 1;
          } else {
            b_i++;
          }
        } else {
          j++;
          exitg1 = 2;
        }
      } while (exitg1 == ((int)((signed char)0)));
      if (exitg1 == ((int)((signed char)1))) {
        exitg2 = true;
      }
    }
    if (recomputeDiags) {
      (void)memset(&F[0], 0, 16U * (sizeof(float)));
      F[0] = expf(A[0]);
      F[5] = expf(A[5]);
      F[10] = expf(A[10]);
      F[15] = expf(A[15]);
    } else {
      recomputeDiags = true;
      j = 0;
      exitg2 = false;
      while ((!exitg2) && (j < ((int)((signed char)4)))) {
        b_i = 0;
        do {
          exitg1 = 0;
          if (b_i <= j) {
            if (!(A[b_i + (4 * j)] == A[j + (4 * b_i)])) {
              recomputeDiags = false;
              exitg1 = 1;
            } else {
              b_i++;
            }
          } else {
            j++;
            exitg1 = 2;
          }
        } while (exitg1 == ((int)((signed char)0)));
        if (exitg1 == ((int)((signed char)1))) {
          exitg2 = true;
        }
      }
      if (recomputeDiags) {
        float w[4];
        (void)memcpy(&A2[0], &A[0], 16U * (sizeof(float)));
        (void)xsyheev(A2, w);
        for (k = 0; k < 4; k++) {
          exptj = expf(w[k]);
          F[4 * k] = A2[4 * k] * exptj;
          j = (4 * k) + 1;
          F[j] = A2[j] * exptj;
          j = (4 * k) + 2;
          F[j] = A2[j] * exptj;
          j = (4 * k) + 3;
          F[j] = A2[j] * exptj;
        }
        (void)memset(&A6[0], 0,
                     (sizeof(float)) << ((unsigned int)((unsigned char)4)));
        for (k = 0; k < 4; k++) {
          int A6_tmp;
          exptj = A6[4 * k];
          j = (4 * k) + 1;
          b_i = (4 * k) + 2;
          A6_tmp = (4 * k) + 3;
          for (b_k = 0; b_k < 4; b_k++) {
            float y;
            y = A2[k + (4 * b_k)];
            exptj += F[4 * b_k] * y;
            A6[j] += F[(4 * b_k) + 1] * y;
            A6[b_i] += F[(4 * b_k) + 2] * y;
            A6[A6_tmp] += F[(4 * b_k) + 3] * y;
          }
          A6[4 * k] = exptj;
        }
        (void)memcpy(&F[0], &A6[0], 16U * (sizeof(float)));
        for (k = 0; k < 4; k++) {
          A6[4 * k] = (F[4 * k] + F[k]) / 2.0F;
          j = (4 * k) + 1;
          A6[j] = (F[j] + F[k + 4]) / 2.0F;
          j = (4 * k) + 2;
          A6[j] = (F[j] + F[k + 8]) / 2.0F;
          j = (4 * k) + 3;
          A6[j] = (F[j] + F[k + 12]) / 2.0F;
        }
        (void)memcpy(&F[0], &A6[0], 16U * (sizeof(float)));
      } else {
        float A4[16];
        float b_A6[16];
        float b_y;
        float c_y;
        float y;
        int ipiv[4];
        int blockFormat[3];
        int A6_tmp;
        int F_tmp;
        recomputeDiags = true;
        j = 3;
        while (recomputeDiags && (j <= ((int)((signed char)4)))) {
          b_i = j;
          while (recomputeDiags && (b_i <= ((int)((signed char)4)))) {
            recomputeDiags = (A[(b_i + (4 * (j - 3))) - 1] == 0.0F);
            b_i++;
          }
          j++;
        }
        if (recomputeDiags) {
          j = 0;
          exitg2 = false;
          while ((!exitg2) && (j < ((int)((signed char)3)))) {
            b_i = j + (4 * j);
            exptj = A[b_i + 1];
            if (exptj != 0.0F) {
              if (((j + 1) != ((int)((signed char)3))) &&
                  (A[(j + (4 * (j + 1))) + 2] != 0.0F)) {
                recomputeDiags = false;
                exitg2 = true;
              } else {
                A6_tmp = j + (4 * (j + 1));
                if (A[b_i] != A[A6_tmp + 1]) {
                  recomputeDiags = false;
                  exitg2 = true;
                } else {
                  y = A[A6_tmp];
                  if (rtIsNaNF(exptj)) {
                    b_y = rtNaNF;
                  } else if (exptj < 0.0F) {
                    b_y = -1.0F;
                  } else {
                    b_y = (float)((exptj > 0.0F) ? 1.0F : 0.0F);
                  }
                  if (rtIsNaNF(y)) {
                    exptj = rtNaNF;
                  } else if (y < 0.0F) {
                    exptj = -1.0F;
                  } else {
                    exptj = (float)((y > 0.0F) ? 1.0F : 0.0F);
                  }
                  if ((b_y * exptj) != -1.0F) {
                    recomputeDiags = false;
                    exitg2 = true;
                  } else {
                    j++;
                  }
                }
              }
            } else {
              j++;
            }
          }
        }
        b_i = getExpmParams(A, A2, A4, b_A6, &s);
        if (s != 0.0F) {
          exptj = rt_powf_snf(2.0F, s);
          y = rt_powf_snf(2.0F, 2.0F * s);
          b_y = rt_powf_snf(2.0F, 4.0F * s);
          c_y = rt_powf_snf(2.0F, 6.0F * s);
          for (k = 0; k < 16; k++) {
            A[k] /= exptj;
            A2[k] /= y;
            A4[k] /= b_y;
            b_A6[k] /= c_y;
          }
        }
        if (recomputeDiags) {
          blockFormat[0] = 0;
          blockFormat[1] = 0;
          blockFormat[2] = 0;
          j = 0;
          while ((j + 1) < ((int)((signed char)3))) {
            if (A[(j + (4 * j)) + 1] != 0.0F) {
              blockFormat[j] = 2;
              blockFormat[j + 1] = 0;
              j += 2;
            } else if (A[(j + (4 * (j + 1))) + 2] == 0.0F) {
              blockFormat[j] = 1;
              j++;
            } else {
              blockFormat[j] = 0;
              j++;
            }
          }
          if (A[11] != 0.0F) {
            blockFormat[2] = 2;
          } else if ((blockFormat[1] == ((int)((signed char)0))) ||
                     (blockFormat[1] == ((int)((signed char)1)))) {
            blockFormat[2] = 1;
          } else {
            /* no actions */
          }
        }
        if (b_i == ((int)((signed char)3))) {
          (void)memcpy(&F[0], &A2[0], 16U * (sizeof(float)));
          F[0] = A2[0] + 60.0F;
          F[5] += 60.0F;
          F[10] += 60.0F;
          F[15] += 60.0F;
          (void)memset(&A6[0], 0,
                       (sizeof(float)) << ((unsigned int)((unsigned char)4)));
          for (k = 0; k < 4; k++) {
            exptj = A6[4 * k];
            j = (4 * k) + 1;
            b_i = (4 * k) + 2;
            A6_tmp = (4 * k) + 3;
            for (b_k = 0; b_k < 4; b_k++) {
              y = F[b_k + (4 * k)];
              exptj += A[4 * b_k] * y;
              A6[j] += A[(4 * b_k) + 1] * y;
              A6[b_i] += A[(4 * b_k) + 2] * y;
              A6[A6_tmp] += A[(4 * b_k) + 3] * y;
            }
            A6[4 * k] = exptj;
          }
          for (k = 0; k < 16; k++) {
            F[k] = A6[k];
            V[k] = 12.0F * A2[k];
          }
          exptj = 120.0F;
        } else if (b_i == ((int)((signed char)5))) {
          for (k = 0; k < 16; k++) {
            F[k] = A4[k] + (420.0F * A2[k]);
          }
          F[0] += 15120.0F;
          F[5] += 15120.0F;
          F[10] += 15120.0F;
          F[15] += 15120.0F;
          (void)memset(&A6[0], 0,
                       (sizeof(float)) << ((unsigned int)((unsigned char)4)));
          for (k = 0; k < 4; k++) {
            exptj = A6[4 * k];
            j = (4 * k) + 1;
            b_i = (4 * k) + 2;
            A6_tmp = (4 * k) + 3;
            for (b_k = 0; b_k < 4; b_k++) {
              y = F[b_k + (4 * k)];
              exptj += A[4 * b_k] * y;
              A6[j] += A[(4 * b_k) + 1] * y;
              A6[b_i] += A[(4 * b_k) + 2] * y;
              A6[A6_tmp] += A[(4 * b_k) + 3] * y;
            }
            A6[4 * k] = exptj;
          }
          for (k = 0; k < 16; k++) {
            F[k] = A6[k];
            V[k] = (30.0F * A4[k]) + (3360.0F * A2[k]);
          }
          exptj = 30240.0F;
        } else if (b_i == ((int)((signed char)7))) {
          for (k = 0; k < 16; k++) {
            F[k] = (b_A6[k] + (1512.0F * A4[k])) + (277200.0F * A2[k]);
          }
          F[0] += 8.64864E+6F;
          F[5] += 8.64864E+6F;
          F[10] += 8.64864E+6F;
          F[15] += 8.64864E+6F;
          (void)memset(&A6[0], 0,
                       (sizeof(float)) << ((unsigned int)((unsigned char)4)));
          for (k = 0; k < 4; k++) {
            exptj = A6[4 * k];
            j = (4 * k) + 1;
            b_i = (4 * k) + 2;
            A6_tmp = (4 * k) + 3;
            for (b_k = 0; b_k < 4; b_k++) {
              y = F[b_k + (4 * k)];
              exptj += A[4 * b_k] * y;
              A6[j] += A[(4 * b_k) + 1] * y;
              A6[b_i] += A[(4 * b_k) + 2] * y;
              A6[A6_tmp] += A[(4 * b_k) + 3] * y;
            }
            A6[4 * k] = exptj;
          }
          for (k = 0; k < 16; k++) {
            F[k] = A6[k];
            V[k] = ((56.0F * b_A6[k]) + (25200.0F * A4[k])) +
                   (1.99584E+6F * A2[k]);
          }
          exptj = 1.729728E+7F;
        } else if (b_i == ((int)((signed char)9))) {
          (void)memset(&V[0], 0,
                       (sizeof(float)) << ((unsigned int)((unsigned char)4)));
          for (k = 0; k < 4; k++) {
            exptj = V[4 * k];
            j = (4 * k) + 1;
            b_i = (4 * k) + 2;
            A6_tmp = (4 * k) + 3;
            for (b_k = 0; b_k < 4; b_k++) {
              y = A2[b_k + (4 * k)];
              exptj += b_A6[4 * b_k] * y;
              V[j] += b_A6[(4 * b_k) + 1] * y;
              V[b_i] += b_A6[(4 * b_k) + 2] * y;
              V[A6_tmp] += b_A6[(4 * b_k) + 3] * y;
            }
            V[4 * k] = exptj;
          }
          for (k = 0; k < 16; k++) {
            F[k] = ((V[k] + (3960.0F * b_A6[k])) + (2.16216E+6F * A4[k])) +
                   (3.027024E+8F * A2[k]);
          }
          F[0] += 8.82161254E+9F;
          F[5] += 8.82161254E+9F;
          F[10] += 8.82161254E+9F;
          F[15] += 8.82161254E+9F;
          (void)memset(&A6[0], 0,
                       (sizeof(float)) << ((unsigned int)((unsigned char)4)));
          for (k = 0; k < 4; k++) {
            exptj = A6[4 * k];
            j = (4 * k) + 1;
            b_i = (4 * k) + 2;
            A6_tmp = (4 * k) + 3;
            for (b_k = 0; b_k < 4; b_k++) {
              y = F[b_k + (4 * k)];
              exptj += A[4 * b_k] * y;
              A6[j] += A[(4 * b_k) + 1] * y;
              A6[b_i] += A[(4 * b_k) + 2] * y;
              A6[A6_tmp] += A[(4 * b_k) + 3] * y;
            }
            A6[4 * k] = exptj;
          }
          for (k = 0; k < 16; k++) {
            F[k] = A6[k];
            V[k] = (((90.0F * V[k]) + (110880.0F * b_A6[k])) +
                    (3.027024E+7F * A4[k])) +
                   (2.0756736E+9F * A2[k]);
          }
          exptj = 1.76432251E+10F;
        } else {
          for (k = 0; k < 16; k++) {
            exptj = b_A6[k];
            y = A4[k];
            b_y = A2[k];
            F[k] = ((3.35221289E+10F * exptj) + (1.05594707E+13F * y)) +
                   (1.18735378E+15F * b_y);
            A6[k] = (exptj + (16380.0F * y)) + (4.08408E+7F * b_y);
          }
          F[0] += 3.23823762E+16F;
          F[5] += 3.23823762E+16F;
          F[10] += 3.23823762E+16F;
          F[15] += 3.23823762E+16F;
          for (k = 0; k < 4; k++) {
            exptj = b_A6[k];
            y = b_A6[k + 4];
            b_y = b_A6[k + 8];
            c_y = b_A6[k + 12];
            for (b_k = 0; b_k < 4; b_k++) {
              j = k + (4 * b_k);
              V[j] = ((((exptj * A6[4 * b_k]) + (y * A6[(4 * b_k) + 1])) +
                       (b_y * A6[(4 * b_k) + 2])) +
                      (c_y * A6[(4 * b_k) + 3])) +
                     F[j];
            }
          }
          (void)memset(&F[0], 0,
                       (sizeof(float)) << ((unsigned int)((unsigned char)4)));
          for (k = 0; k < 4; k++) {
            exptj = F[4 * k];
            j = (4 * k) + 1;
            b_i = (4 * k) + 2;
            A6_tmp = (4 * k) + 3;
            for (b_k = 0; b_k < 4; b_k++) {
              y = V[b_k + (4 * k)];
              exptj += A[4 * b_k] * y;
              F[j] += A[(4 * b_k) + 1] * y;
              F[b_i] += A[(4 * b_k) + 2] * y;
              F[A6_tmp] += A[(4 * b_k) + 3] * y;
            }
            F[4 * k] = exptj;
          }
          for (k = 0; k < 16; k++) {
            A6[k] = ((182.0F * b_A6[k]) + (960960.0F * A4[k])) +
                    (1.32324198E+9F * A2[k]);
          }
          for (k = 0; k < 4; k++) {
            for (b_k = 0; b_k < 4; b_k++) {
              j = k + (4 * b_k);
              V[j] = ((((((b_A6[k] * A6[4 * b_k]) +
                          (b_A6[k + 4] * A6[(4 * b_k) + 1])) +
                         (b_A6[k + 8] * A6[(4 * b_k) + 2])) +
                        (b_A6[k + 12] * A6[(4 * b_k) + 3])) +
                       (6.70442586E+11F * b_A6[j])) +
                      (1.29060194E+14F * A4[j])) +
                     (7.77177E+15F * A2[j]);
            }
          }
          exptj = 6.47647525E+16F;
        }
        V[0] += exptj;
        V[5] += exptj;
        V[10] += exptj;
        V[15] += exptj;
        for (k = 0; k < 16; k++) {
          exptj = F[k];
          V[k] -= exptj;
          exptj *= 2.0F;
          F[k] = exptj;
        }
        (void)xzgetrf(V, ipiv);
        for (k = 0; k < 3; k++) {
          j = ipiv[k];
          if (j != (k + 1)) {
            exptj = F[k];
            F[k] = F[j - 1];
            F[j - 1] = exptj;
            exptj = F[k + 4];
            F[k + 4] = F[j + 3];
            F[j + 3] = exptj;
            exptj = F[k + 8];
            F[k + 8] = F[j + 7];
            F[j + 7] = exptj;
            exptj = F[k + 12];
            F[k + 12] = F[j + 11];
            F[j + 11] = exptj;
          }
        }
        for (k = 0; k < 4; k++) {
          j = 4 * k;
          for (b_k = 0; b_k < 4; b_k++) {
            b_i = 4 * b_k;
            A6_tmp = b_k + j;
            if (F[A6_tmp] != 0.0F) {
              F_tmp = b_k + 2;
              for (c_i = F_tmp; c_i < 5; c_i++) {
                int b_F_tmp;
                b_F_tmp = (c_i + j) - 1;
                F[b_F_tmp] -= F[A6_tmp] * V[(c_i + b_i) - 1];
              }
            }
          }
        }
        for (k = 0; k < 4; k++) {
          j = 4 * k;
          for (b_k = 3; b_k >= 0; b_k--) {
            b_i = 4 * b_k;
            A6_tmp = b_k + j;
            exptj = F[A6_tmp];
            if (exptj != 0.0F) {
              F[A6_tmp] = exptj / V[b_k + b_i];
              for (c_i = 0; c_i < b_k; c_i++) {
                F_tmp = c_i + j;
                F[F_tmp] -= F[A6_tmp] * V[c_i + b_i];
              }
            }
          }
        }
        F[0]++;
        F[5]++;
        F[10]++;
        F[15]++;
        if (recomputeDiags) {
          recomputeBlockDiag(A, F, blockFormat);
        }
        F_tmp = (int)s;
        for (c_i = 0; c_i < F_tmp; c_i++) {
          (void)memset(&A6[0], 0,
                       (sizeof(float)) << ((unsigned int)((unsigned char)4)));
          for (k = 0; k < 4; k++) {
            exptj = A6[4 * k];
            j = (4 * k) + 1;
            b_i = (4 * k) + 2;
            A6_tmp = (4 * k) + 3;
            for (b_k = 0; b_k < 4; b_k++) {
              y = F[b_k + (4 * k)];
              exptj += F[4 * b_k] * y;
              A6[j] += F[(4 * b_k) + 1] * y;
              A6[b_i] += F[(4 * b_k) + 2] * y;
              A6[A6_tmp] += F[(4 * b_k) + 3] * y;
            }
            A6[4 * k] = exptj;
          }
          (void)memcpy(&F[0], &A6[0], 16U * (sizeof(float)));
          if (recomputeDiags) {
            for (k = 0; k < 16; k++) {
              A[k] *= 2.0F;
            }
            recomputeBlockDiag(A, F, blockFormat);
          }
        }
      }
    }
  }
}

static void filter_init_init(filter_entryStackData *SD)
{
  SD->pd->params = r;
}

static int getExpmParams(const float A[16], float A2[16], float A4[16],
                         float A6[16], float *s)
{
  static const float theta[3] = {0.425873F, 1.8801527F, 3.92572474F};
  float fv[16];
  float y[16];
  float b_s;
  float d6;
  float d8;
  float d_s;
  float e_s;
  float eta1;
  float eta3;
  int A2_tmp;
  int b_A2_tmp;
  int b_eint;
  int b_i;
  int c_A2_tmp;
  int eint;
  int k;
  int m;
  bool exitg1;
  bool guard1;
  bool guard2;
  bool guard3;
  bool guard4;
  b_s = 0.0F;
  (void)memset(&A2[0], 0,
               (sizeof(float)) << ((unsigned int)((unsigned char)4)));
  for (k = 0; k < 4; k++) {
    d_s = A2[4 * k];
    A2_tmp = (4 * k) + 1;
    b_A2_tmp = (4 * k) + 2;
    c_A2_tmp = (4 * k) + 3;
    for (b_i = 0; b_i < 4; b_i++) {
      e_s = A[b_i + (4 * k)];
      d_s += A[4 * b_i] * e_s;
      A2[A2_tmp] += A[(4 * b_i) + 1] * e_s;
      A2[b_A2_tmp] += A[(4 * b_i) + 2] * e_s;
      A2[c_A2_tmp] += A[(4 * b_i) + 3] * e_s;
    }
    A2[4 * k] = d_s;
  }
  (void)memset(&A4[0], 0,
               (sizeof(float)) << ((unsigned int)((unsigned char)4)));
  for (k = 0; k < 4; k++) {
    d_s = A4[4 * k];
    m = (4 * k) + 1;
    A2_tmp = (4 * k) + 2;
    b_A2_tmp = (4 * k) + 3;
    for (b_i = 0; b_i < 4; b_i++) {
      e_s = A2[b_i + (4 * k)];
      d_s += A2[4 * b_i] * e_s;
      A4[m] += A2[(4 * b_i) + 1] * e_s;
      A4[A2_tmp] += A2[(4 * b_i) + 2] * e_s;
      A4[b_A2_tmp] += A2[(4 * b_i) + 3] * e_s;
    }
    A4[4 * k] = d_s;
  }
  (void)memset(&A6[0], 0,
               (sizeof(float)) << ((unsigned int)((unsigned char)4)));
  for (k = 0; k < 4; k++) {
    d_s = A6[4 * k];
    m = (4 * k) + 1;
    A2_tmp = (4 * k) + 2;
    b_A2_tmp = (4 * k) + 3;
    for (b_i = 0; b_i < 4; b_i++) {
      e_s = A2[b_i + (4 * k)];
      d_s += A4[4 * b_i] * e_s;
      A6[m] += A4[(4 * b_i) + 1] * e_s;
      A6[A2_tmp] += A4[(4 * b_i) + 2] * e_s;
      A6[b_A2_tmp] += A4[(4 * b_i) + 3] * e_s;
    }
    A6[4 * k] = d_s;
  }
  d_s = 0.0F;
  A2_tmp = 0;
  exitg1 = false;
  while ((!exitg1) && (A2_tmp < ((int)((signed char)4)))) {
    e_s = ((fabsf(A6[4 * A2_tmp]) + fabsf(A6[(4 * A2_tmp) + 1])) +
           fabsf(A6[(4 * A2_tmp) + 2])) +
          fabsf(A6[(4 * A2_tmp) + 3]);
    if (rtIsNaNF(e_s)) {
      d_s = rtNaNF;
      exitg1 = true;
    } else {
      if (e_s > d_s) {
        d_s = e_s;
      }
      A2_tmp++;
    }
  }
  d6 = rt_powf_snf(d_s, 0.166666672F);
  e_s = 0.0F;
  A2_tmp = 0;
  exitg1 = false;
  while ((!exitg1) && (A2_tmp < ((int)((signed char)4)))) {
    d_s = ((fabsf(A4[4 * A2_tmp]) + fabsf(A4[(4 * A2_tmp) + 1])) +
           fabsf(A4[(4 * A2_tmp) + 2])) +
          fabsf(A4[(4 * A2_tmp) + 3]);
    if (rtIsNaNF(d_s)) {
      e_s = rtNaNF;
      exitg1 = true;
    } else {
      if (d_s > e_s) {
        e_s = d_s;
      }
      A2_tmp++;
    }
  }
  eta1 = fmaxf(rt_powf_snf(e_s, 0.25F), d6);
  guard1 = false;
  guard2 = false;
  guard3 = false;
  guard4 = false;
  if (eta1 <= 0.0149558522F) {
    for (k = 0; k < 16; k++) {
      fv[k] = 0.192850113F * fabsf(A[k]);
    }
    mpower(fv, 7.0F, y);
    eta3 = 0.0F;
    A2_tmp = 0;
    exitg1 = false;
    while ((!exitg1) && (A2_tmp < ((int)((signed char)4)))) {
      e_s = ((fabsf(y[4 * A2_tmp]) + fabsf(y[(4 * A2_tmp) + 1])) +
             fabsf(y[(4 * A2_tmp) + 2])) +
            fabsf(y[(4 * A2_tmp) + 3]);
      if (rtIsNaNF(e_s)) {
        eta3 = rtNaNF;
        exitg1 = true;
      } else {
        if (e_s > eta3) {
          eta3 = e_s;
        }
        A2_tmp++;
      }
    }
    e_s = 0.0F;
    A2_tmp = 0;
    exitg1 = false;
    while ((!exitg1) && (A2_tmp < ((int)((signed char)4)))) {
      d_s = ((fabsf(A[4 * A2_tmp]) + fabsf(A[(4 * A2_tmp) + 1])) +
             fabsf(A[(4 * A2_tmp) + 2])) +
            fabsf(A[(4 * A2_tmp) + 3]);
      if (rtIsNaNF(d_s)) {
        e_s = rtNaNF;
        exitg1 = true;
      } else {
        if (d_s > e_s) {
          e_s = d_s;
        }
        A2_tmp++;
      }
    }
    if (fmaxf(ceilf(b_log2((2.0F * (eta3 / e_s)) / 1.1920929E-7F) / 6.0F),
              0.0F) == 0.0F) {
      m = 3;
    } else {
      guard4 = true;
    }
  } else {
    guard4 = true;
  }
  if (guard4) {
    if (eta1 <= 0.253939837F) {
      for (k = 0; k < 16; k++) {
        fv[k] = 0.123218715F * fabsf(A[k]);
      }
      mpower(fv, 11.0F, y);
      eta1 = 0.0F;
      A2_tmp = 0;
      exitg1 = false;
      while ((!exitg1) && (A2_tmp < ((int)((signed char)4)))) {
        e_s = ((fabsf(y[4 * A2_tmp]) + fabsf(y[(4 * A2_tmp) + 1])) +
               fabsf(y[(4 * A2_tmp) + 2])) +
              fabsf(y[(4 * A2_tmp) + 3]);
        if (rtIsNaNF(e_s)) {
          eta1 = rtNaNF;
          exitg1 = true;
        } else {
          if (e_s > eta1) {
            eta1 = e_s;
          }
          A2_tmp++;
        }
      }
      e_s = 0.0F;
      A2_tmp = 0;
      exitg1 = false;
      while ((!exitg1) && (A2_tmp < ((int)((signed char)4)))) {
        d_s = ((fabsf(A[4 * A2_tmp]) + fabsf(A[(4 * A2_tmp) + 1])) +
               fabsf(A[(4 * A2_tmp) + 2])) +
              fabsf(A[(4 * A2_tmp) + 3]);
        if (rtIsNaNF(d_s)) {
          e_s = rtNaNF;
          exitg1 = true;
        } else {
          if (d_s > e_s) {
            e_s = d_s;
          }
          A2_tmp++;
        }
      }
      if (fmaxf(ceilf(b_log2((2.0F * (eta1 / e_s)) / 1.1920929E-7F) / 10.0F),
                0.0F) == 0.0F) {
        m = 5;
      } else {
        guard3 = true;
      }
    } else {
      guard3 = true;
    }
  }
  if (guard3) {
    mpower(A4, 2.0F, y);
    e_s = 0.0F;
    A2_tmp = 0;
    exitg1 = false;
    while ((!exitg1) && (A2_tmp < ((int)((signed char)4)))) {
      d_s = ((fabsf(y[4 * A2_tmp]) + fabsf(y[(4 * A2_tmp) + 1])) +
             fabsf(y[(4 * A2_tmp) + 2])) +
            fabsf(y[(4 * A2_tmp) + 3]);
      if (rtIsNaNF(d_s)) {
        e_s = rtNaNF;
        exitg1 = true;
      } else {
        if (d_s > e_s) {
          e_s = d_s;
        }
        A2_tmp++;
      }
    }
    d8 = rt_powf_snf(e_s, 0.125F);
    eta3 = fmaxf(d6, d8);
    if (eta3 <= 0.950417876F) {
      for (k = 0; k < 16; k++) {
        fv[k] = 0.0904753208F * fabsf(A[k]);
      }
      mpower(fv, 15.0F, y);
      eta1 = 0.0F;
      A2_tmp = 0;
      exitg1 = false;
      while ((!exitg1) && (A2_tmp < ((int)((signed char)4)))) {
        e_s = ((fabsf(y[4 * A2_tmp]) + fabsf(y[(4 * A2_tmp) + 1])) +
               fabsf(y[(4 * A2_tmp) + 2])) +
              fabsf(y[(4 * A2_tmp) + 3]);
        if (rtIsNaNF(e_s)) {
          eta1 = rtNaNF;
          exitg1 = true;
        } else {
          if (e_s > eta1) {
            eta1 = e_s;
          }
          A2_tmp++;
        }
      }
      e_s = 0.0F;
      A2_tmp = 0;
      exitg1 = false;
      while ((!exitg1) && (A2_tmp < ((int)((signed char)4)))) {
        d_s = ((fabsf(A[4 * A2_tmp]) + fabsf(A[(4 * A2_tmp) + 1])) +
               fabsf(A[(4 * A2_tmp) + 2])) +
              fabsf(A[(4 * A2_tmp) + 3]);
        if (rtIsNaNF(d_s)) {
          e_s = rtNaNF;
          exitg1 = true;
        } else {
          if (d_s > e_s) {
            e_s = d_s;
          }
          A2_tmp++;
        }
      }
      if (fmaxf(ceilf(b_log2((2.0F * (eta1 / e_s)) / 1.1920929E-7F) / 14.0F),
                0.0F) == 0.0F) {
        m = 7;
      } else {
        guard2 = true;
      }
    } else {
      guard2 = true;
    }
  }
  if (guard2) {
    if (eta3 <= 2.09784794F) {
      for (k = 0; k < 16; k++) {
        fv[k] = 0.0714677349F * fabsf(A[k]);
      }
      mpower(fv, 19.0F, y);
      eta1 = 0.0F;
      A2_tmp = 0;
      exitg1 = false;
      while ((!exitg1) && (A2_tmp < ((int)((signed char)4)))) {
        e_s = ((fabsf(y[4 * A2_tmp]) + fabsf(y[(4 * A2_tmp) + 1])) +
               fabsf(y[(4 * A2_tmp) + 2])) +
              fabsf(y[(4 * A2_tmp) + 3]);
        if (rtIsNaNF(e_s)) {
          eta1 = rtNaNF;
          exitg1 = true;
        } else {
          if (e_s > eta1) {
            eta1 = e_s;
          }
          A2_tmp++;
        }
      }
      e_s = 0.0F;
      A2_tmp = 0;
      exitg1 = false;
      while ((!exitg1) && (A2_tmp < ((int)((signed char)4)))) {
        d_s = ((fabsf(A[4 * A2_tmp]) + fabsf(A[(4 * A2_tmp) + 1])) +
               fabsf(A[(4 * A2_tmp) + 2])) +
              fabsf(A[(4 * A2_tmp) + 3]);
        if (rtIsNaNF(d_s)) {
          e_s = rtNaNF;
          exitg1 = true;
        } else {
          if (d_s > e_s) {
            e_s = d_s;
          }
          A2_tmp++;
        }
      }
      if (fmaxf(ceilf(b_log2((2.0F * (eta1 / e_s)) / 1.1920929E-7F) / 18.0F),
                0.0F) == 0.0F) {
        m = 9;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
  }
  if (guard1) {
    float z[16];
    (void)memset(&y[0], 0,
                 (sizeof(float)) << ((unsigned int)((unsigned char)4)));
    for (k = 0; k < 4; k++) {
      e_s = y[4 * k];
      m = (4 * k) + 1;
      A2_tmp = (4 * k) + 2;
      b_A2_tmp = (4 * k) + 3;
      for (b_i = 0; b_i < 4; b_i++) {
        d_s = A6[b_i + (4 * k)];
        e_s += A4[4 * b_i] * d_s;
        y[m] += A4[(4 * b_i) + 1] * d_s;
        y[A2_tmp] += A4[(4 * b_i) + 2] * d_s;
        y[b_A2_tmp] += A4[(4 * b_i) + 3] * d_s;
      }
      y[4 * k] = e_s;
    }
    e_s = 0.0F;
    A2_tmp = 0;
    exitg1 = false;
    while ((!exitg1) && (A2_tmp < ((int)((signed char)4)))) {
      d_s = ((fabsf(y[4 * A2_tmp]) + fabsf(y[(4 * A2_tmp) + 1])) +
             fabsf(y[(4 * A2_tmp) + 2])) +
            fabsf(y[(4 * A2_tmp) + 3]);
      if (rtIsNaNF(d_s)) {
        e_s = rtNaNF;
        exitg1 = true;
      } else {
        if (d_s > e_s) {
          e_s = d_s;
        }
        A2_tmp++;
      }
    }
    b_s = fmaxf(ceilf(b_log2(fminf(eta3, fmaxf(d8, rt_powf_snf(e_s, 0.1F))) /
                             5.37192059F)),
                0.0F);
    e_s = rt_powf_snf(2.0F, b_s);
    for (k = 0; k < 16; k++) {
      d_s = A[k] / e_s;
      z[k] = d_s;
      fv[k] = 0.050315544F * fabsf(d_s);
    }
    mpower(fv, 27.0F, y);
    eta1 = 0.0F;
    A2_tmp = 0;
    exitg1 = false;
    while ((!exitg1) && (A2_tmp < ((int)((signed char)4)))) {
      e_s = ((fabsf(y[4 * A2_tmp]) + fabsf(y[(4 * A2_tmp) + 1])) +
             fabsf(y[(4 * A2_tmp) + 2])) +
            fabsf(y[(4 * A2_tmp) + 3]);
      if (rtIsNaNF(e_s)) {
        eta1 = rtNaNF;
        exitg1 = true;
      } else {
        if (e_s > eta1) {
          eta1 = e_s;
        }
        A2_tmp++;
      }
    }
    e_s = 0.0F;
    A2_tmp = 0;
    exitg1 = false;
    while ((!exitg1) && (A2_tmp < ((int)((signed char)4)))) {
      d_s = ((fabsf(z[4 * A2_tmp]) + fabsf(z[(4 * A2_tmp) + 1])) +
             fabsf(z[(4 * A2_tmp) + 2])) +
            fabsf(z[(4 * A2_tmp) + 3]);
      if (rtIsNaNF(d_s)) {
        e_s = rtNaNF;
        exitg1 = true;
      } else {
        if (d_s > e_s) {
          e_s = d_s;
        }
        A2_tmp++;
      }
    }
    b_s += fmaxf(ceilf(b_log2((2.0F * (eta1 / e_s)) / 1.1920929E-7F) / 26.0F),
                 0.0F);
    if (rtIsInfF(b_s)) {
      d_s = 0.0F;
      A2_tmp = 0;
      exitg1 = false;
      while ((!exitg1) && (A2_tmp < ((int)((signed char)4)))) {
        e_s = ((fabsf(A[4 * A2_tmp]) + fabsf(A[(4 * A2_tmp) + 1])) +
               fabsf(A[(4 * A2_tmp) + 2])) +
              fabsf(A[(4 * A2_tmp) + 3]);
        if (rtIsNaNF(e_s)) {
          d_s = rtNaNF;
          exitg1 = true;
        } else {
          if (e_s > d_s) {
            d_s = e_s;
          }
          A2_tmp++;
        }
      }
      d_s /= 5.37192059F;
      if ((!rtIsInfF(d_s)) && (!rtIsNaNF(d_s))) {
        d_s = frexpf(d_s, &b_eint);
      } else {
        b_eint = 0;
      }
      b_s = (float)b_eint;
      if (d_s == 0.5F) {
        b_s = ((float)b_eint) - 1.0F;
      }
    }
    m = 13;
  }
  *s = b_s;
  d_s = 0.0F;
  A2_tmp = 0;
  exitg1 = false;
  while ((!exitg1) && (A2_tmp < ((int)((signed char)4)))) {
    e_s = ((fabsf(A[4 * A2_tmp]) + fabsf(A[(4 * A2_tmp) + 1])) +
           fabsf(A[(4 * A2_tmp) + 2])) +
          fabsf(A[(4 * A2_tmp) + 3]);
    if (rtIsNaNF(e_s)) {
      d_s = rtNaNF;
      exitg1 = true;
    } else {
      if (e_s > d_s) {
        d_s = e_s;
      }
      A2_tmp++;
    }
  }
  b_A2_tmp = 0;
  c_A2_tmp = 7;
  if (d_s <= 3.92572474F) {
    A2_tmp = 0;
    exitg1 = false;
    while ((!exitg1) && (A2_tmp < ((int)((signed char)3)))) {
      if (d_s <= theta[A2_tmp]) {
        c_A2_tmp = (2 * A2_tmp) + 3;
        exitg1 = true;
      } else {
        A2_tmp++;
      }
    }
  } else {
    d_s /= 3.92572474F;
    if ((!rtIsInfF(d_s)) && (!rtIsNaNF(d_s))) {
      d_s = frexpf(d_s, &eint);
    } else {
      eint = 0;
    }
    b_A2_tmp = eint;
    if (d_s == 0.5F) {
      b_A2_tmp = eint - 1;
    }
  }
  if (((float)b_A2_tmp) <= (b_s + 2.0F)) {
    *s = (float)b_A2_tmp;
    m = c_A2_tmp;
  }
  return m;
}

static void inv(const float x[16], float y[16])
{
  float b_x[16];
  int ipiv[4];
  int b_i;
  int c_i;
  int j;
  int k;
  int kAcol;
  int pipk;
  int y_tmp;
  signed char p[4];
  for (k = 0; k < 16; k++) {
    y[k] = 0.0F;
    b_x[k] = x[k];
  }
  (void)xzgetrf(b_x, ipiv);
  p[0] = 1;
  p[1] = 2;
  p[2] = 3;
  p[3] = 4;
  if (ipiv[0] > ((int)((signed char)1))) {
    pipk = (int)p[ipiv[0] - 1];
    p[ipiv[0] - 1] = 1;
    p[0] = (signed char)pipk;
  }
  if (ipiv[1] > ((int)((signed char)2))) {
    pipk = (int)p[ipiv[1] - 1];
    p[ipiv[1] - 1] = p[1];
    p[1] = (signed char)pipk;
  }
  if (ipiv[2] > ((int)((signed char)3))) {
    pipk = (int)p[ipiv[2] - 1];
    p[ipiv[2] - 1] = p[2];
    p[2] = (signed char)pipk;
  }
  for (k = 0; k < 4; k++) {
    pipk = 4 * (((int)p[k]) - 1);
    y[k + pipk] = 1.0F;
    for (j = k + 1; j < 5; j++) {
      kAcol = (j + pipk) - 1;
      if (y[kAcol] != 0.0F) {
        b_i = j + 1;
        for (c_i = b_i; c_i < 5; c_i++) {
          y_tmp = (c_i + pipk) - 1;
          y[y_tmp] -= y[kAcol] * b_x[(c_i + (4 * (j - 1))) - 1];
        }
      }
    }
  }
  for (k = 0; k < 4; k++) {
    pipk = 4 * k;
    for (j = 3; j >= 0; j--) {
      float b_f;
      kAcol = 4 * j;
      b_i = j + pipk;
      b_f = y[b_i];
      if (b_f != 0.0F) {
        y[b_i] = b_f / b_x[j + kAcol];
        for (c_i = 0; c_i < j; c_i++) {
          y_tmp = c_i + pipk;
          y[y_tmp] -= y[b_i] * b_x[c_i + kAcol];
        }
      }
    }
  }
}

static void mpower(const float b_a[16], float b, float c[16])
{
  float aBuffer[16];
  float cBuffer[16];
  float c_a[16];
  int b_i;
  int b_k;
  int k;
  if (floorf(b) == b) {
    float e;
    e = fabsf(b);
    if (e < 2.14748365E+9F) {
      int b_n;
      int n;
      int nb;
      int nbitson;
      (void)memcpy(&c_a[0], &b_a[0], 16U * (sizeof(float)));
      n = (int)e;
      b_n = (int)e;
      nbitson = 0;
      nb = -1;
      while (b_n > ((int)((signed char)0))) {
        nb++;
        if ((((unsigned int)b_n) & ((unsigned int)1U)) != ((unsigned int)0U)) {
          nbitson++;
        }
        b_n = asr_s32(b_n, 1U);
      }
      if (((int)e) <= ((int)((signed char)2))) {
        if (b == 2.0F) {
          (void)memset(&c[0], 0,
                       (sizeof(float)) << ((unsigned int)((unsigned char)4)));
          for (k = 0; k < 4; k++) {
            float b_f;
            int c_tmp;
            b_f = c[4 * k];
            b_n = (4 * k) + 1;
            nbitson = (4 * k) + 2;
            c_tmp = (4 * k) + 3;
            for (b_i = 0; b_i < 4; b_i++) {
              float f1;
              f1 = b_a[b_i + (4 * k)];
              b_f += b_a[4 * b_i] * f1;
              c[b_n] += b_a[(4 * b_i) + 1] * f1;
              c[nbitson] += b_a[(4 * b_i) + 2] * f1;
              c[c_tmp] += b_a[(4 * b_i) + 3] * f1;
            }
            c[4 * k] = b_f;
          }
        } else if (b == 1.0F) {
          (void)memcpy(&c[0], &b_a[0], 16U * (sizeof(float)));
        } else if (b == -1.0F) {
          inv(b_a, c);
        } else if (b == -2.0F) {
          (void)memset(&c_a[0], 0,
                       (sizeof(float)) << ((unsigned int)((unsigned char)4)));
          for (k = 0; k < 4; k++) {
            float b_f;
            int c_tmp;
            b_f = c_a[4 * k];
            b_n = (4 * k) + 1;
            nbitson = (4 * k) + 2;
            c_tmp = (4 * k) + 3;
            for (b_i = 0; b_i < 4; b_i++) {
              float f1;
              f1 = b_a[b_i + (4 * k)];
              b_f += b_a[4 * b_i] * f1;
              c_a[b_n] += b_a[(4 * b_i) + 1] * f1;
              c_a[nbitson] += b_a[(4 * b_i) + 2] * f1;
              c_a[c_tmp] += b_a[(4 * b_i) + 3] * f1;
            }
            c_a[4 * k] = b_f;
          }
          inv(c_a, c);
        } else {
          bool lsb;
          lsb = false;
          for (k = 0; k < 16; k++) {
            if (lsb || (rtIsNaNF(b_a[k]))) {
              lsb = true;
            }
          }
          if (lsb) {
            for (k = 0; k < 16; k++) {
              c[k] = rtNaNF;
            }
          } else {
            (void)memset(&c[0], 0, 16U * (sizeof(float)));
            c[0] = 1.0F;
            c[5] = 1.0F;
            c[10] = 1.0F;
            c[15] = 1.0F;
          }
        }
      } else {
        float b_f;
        float f1;
        int c_tmp;
        bool aBufferInUse;
        bool first;
        bool lsb;
        first = true;
        aBufferInUse = false;
        lsb = ((((unsigned int)nbitson) & ((unsigned int)1U)) !=
               ((unsigned int)0U));
        if ((lsb && (b < 0.0F)) || ((!lsb) && (b >= 0.0F))) {
          lsb = true;
        } else {
          lsb = false;
        }
        for (b_k = 0; b_k < nb; b_k++) {
          if ((((unsigned int)n) & ((unsigned int)1U)) != ((unsigned int)0U)) {
            if (first) {
              first = false;
              if (lsb) {
                if (aBufferInUse) {
                  (void)memcpy(&cBuffer[0], &aBuffer[0], 16U * (sizeof(float)));
                } else {
                  (void)memcpy(&cBuffer[0], &c_a[0], 16U * (sizeof(float)));
                }
              } else if (aBufferInUse) {
                (void)memcpy(&c[0], &aBuffer[0], 16U * (sizeof(float)));
              } else {
                (void)memcpy(&c[0], &c_a[0], 16U * (sizeof(float)));
              }
            } else {
              if (aBufferInUse) {
                if (lsb) {
                  (void)memset(&c[0], 0,
                               (sizeof(float))
                                   << ((unsigned int)((unsigned char)4)));
                  for (k = 0; k < 4; k++) {
                    b_f = c[4 * k];
                    b_n = (4 * k) + 1;
                    nbitson = (4 * k) + 2;
                    c_tmp = (4 * k) + 3;
                    for (b_i = 0; b_i < 4; b_i++) {
                      f1 = aBuffer[b_i + (4 * k)];
                      b_f += cBuffer[4 * b_i] * f1;
                      c[b_n] += cBuffer[(4 * b_i) + 1] * f1;
                      c[nbitson] += cBuffer[(4 * b_i) + 2] * f1;
                      c[c_tmp] += cBuffer[(4 * b_i) + 3] * f1;
                    }
                    c[4 * k] = b_f;
                  }
                } else {
                  (void)memset(&cBuffer[0], 0,
                               (sizeof(float))
                                   << ((unsigned int)((unsigned char)4)));
                  for (k = 0; k < 4; k++) {
                    b_f = cBuffer[4 * k];
                    b_n = (4 * k) + 1;
                    nbitson = (4 * k) + 2;
                    c_tmp = (4 * k) + 3;
                    for (b_i = 0; b_i < 4; b_i++) {
                      f1 = aBuffer[b_i + (4 * k)];
                      b_f += c[4 * b_i] * f1;
                      cBuffer[b_n] += c[(4 * b_i) + 1] * f1;
                      cBuffer[nbitson] += c[(4 * b_i) + 2] * f1;
                      cBuffer[c_tmp] += c[(4 * b_i) + 3] * f1;
                    }
                    cBuffer[4 * k] = b_f;
                  }
                }
              } else if (lsb) {
                (void)memset(&c[0], 0,
                             (sizeof(float))
                                 << ((unsigned int)((unsigned char)4)));
                for (k = 0; k < 4; k++) {
                  b_f = c[4 * k];
                  b_n = (4 * k) + 1;
                  nbitson = (4 * k) + 2;
                  c_tmp = (4 * k) + 3;
                  for (b_i = 0; b_i < 4; b_i++) {
                    f1 = c_a[b_i + (4 * k)];
                    b_f += cBuffer[4 * b_i] * f1;
                    c[b_n] += cBuffer[(4 * b_i) + 1] * f1;
                    c[nbitson] += cBuffer[(4 * b_i) + 2] * f1;
                    c[c_tmp] += cBuffer[(4 * b_i) + 3] * f1;
                  }
                  c[4 * k] = b_f;
                }
              } else {
                (void)memset(&cBuffer[0], 0,
                             (sizeof(float))
                                 << ((unsigned int)((unsigned char)4)));
                for (k = 0; k < 4; k++) {
                  b_f = cBuffer[4 * k];
                  b_n = (4 * k) + 1;
                  nbitson = (4 * k) + 2;
                  c_tmp = (4 * k) + 3;
                  for (b_i = 0; b_i < 4; b_i++) {
                    f1 = c_a[b_i + (4 * k)];
                    b_f += c[4 * b_i] * f1;
                    cBuffer[b_n] += c[(4 * b_i) + 1] * f1;
                    cBuffer[nbitson] += c[(4 * b_i) + 2] * f1;
                    cBuffer[c_tmp] += c[(4 * b_i) + 3] * f1;
                  }
                  cBuffer[4 * k] = b_f;
                }
              }
              lsb = !lsb;
            }
          }
          n = asr_s32(n, 1U);
          if (aBufferInUse) {
            (void)memset(&c_a[0], 0,
                         (sizeof(float)) << ((unsigned int)((unsigned char)4)));
            for (k = 0; k < 4; k++) {
              b_f = c_a[4 * k];
              b_n = (4 * k) + 1;
              nbitson = (4 * k) + 2;
              c_tmp = (4 * k) + 3;
              for (b_i = 0; b_i < 4; b_i++) {
                f1 = aBuffer[b_i + (4 * k)];
                b_f += aBuffer[4 * b_i] * f1;
                c_a[b_n] += aBuffer[(4 * b_i) + 1] * f1;
                c_a[nbitson] += aBuffer[(4 * b_i) + 2] * f1;
                c_a[c_tmp] += aBuffer[(4 * b_i) + 3] * f1;
              }
              c_a[4 * k] = b_f;
            }
          } else {
            (void)memset(&aBuffer[0], 0,
                         (sizeof(float)) << ((unsigned int)((unsigned char)4)));
            for (k = 0; k < 4; k++) {
              b_f = aBuffer[4 * k];
              b_n = (4 * k) + 1;
              nbitson = (4 * k) + 2;
              c_tmp = (4 * k) + 3;
              for (b_i = 0; b_i < 4; b_i++) {
                f1 = c_a[b_i + (4 * k)];
                b_f += c_a[4 * b_i] * f1;
                aBuffer[b_n] += c_a[(4 * b_i) + 1] * f1;
                aBuffer[nbitson] += c_a[(4 * b_i) + 2] * f1;
                aBuffer[c_tmp] += c_a[(4 * b_i) + 3] * f1;
              }
              aBuffer[4 * k] = b_f;
            }
          }
          aBufferInUse = !aBufferInUse;
        }
        if (first) {
          if (b < 0.0F) {
            if (aBufferInUse) {
              inv(aBuffer, c);
            } else {
              inv(c_a, c);
            }
          } else if (aBufferInUse) {
            (void)memcpy(&c[0], &aBuffer[0], 16U * (sizeof(float)));
          } else {
            (void)memcpy(&c[0], &c_a[0], 16U * (sizeof(float)));
          }
        } else if (b < 0.0F) {
          if (aBufferInUse) {
            (void)memset(&cBuffer[0], 0,
                         (sizeof(float)) << ((unsigned int)((unsigned char)4)));
            for (k = 0; k < 4; k++) {
              b_f = cBuffer[4 * k];
              b_n = (4 * k) + 1;
              nbitson = (4 * k) + 2;
              c_tmp = (4 * k) + 3;
              for (b_i = 0; b_i < 4; b_i++) {
                f1 = aBuffer[b_i + (4 * k)];
                b_f += c[4 * b_i] * f1;
                cBuffer[b_n] += c[(4 * b_i) + 1] * f1;
                cBuffer[nbitson] += c[(4 * b_i) + 2] * f1;
                cBuffer[c_tmp] += c[(4 * b_i) + 3] * f1;
              }
              cBuffer[4 * k] = b_f;
            }
          } else {
            (void)memset(&cBuffer[0], 0,
                         (sizeof(float)) << ((unsigned int)((unsigned char)4)));
            for (k = 0; k < 4; k++) {
              b_f = cBuffer[4 * k];
              b_n = (4 * k) + 1;
              nbitson = (4 * k) + 2;
              c_tmp = (4 * k) + 3;
              for (b_i = 0; b_i < 4; b_i++) {
                f1 = c_a[b_i + (4 * k)];
                b_f += c[4 * b_i] * f1;
                cBuffer[b_n] += c[(4 * b_i) + 1] * f1;
                cBuffer[nbitson] += c[(4 * b_i) + 2] * f1;
                cBuffer[c_tmp] += c[(4 * b_i) + 3] * f1;
              }
              cBuffer[4 * k] = b_f;
            }
          }
          inv(cBuffer, c);
        } else if (aBufferInUse) {
          (void)memset(&c[0], 0,
                       (sizeof(float)) << ((unsigned int)((unsigned char)4)));
          for (k = 0; k < 4; k++) {
            b_f = c[4 * k];
            b_n = (4 * k) + 1;
            nbitson = (4 * k) + 2;
            c_tmp = (4 * k) + 3;
            for (b_i = 0; b_i < 4; b_i++) {
              f1 = aBuffer[b_i + (4 * k)];
              b_f += cBuffer[4 * b_i] * f1;
              c[b_n] += cBuffer[(4 * b_i) + 1] * f1;
              c[nbitson] += cBuffer[(4 * b_i) + 2] * f1;
              c[c_tmp] += cBuffer[(4 * b_i) + 3] * f1;
            }
            c[4 * k] = b_f;
          }
        } else {
          (void)memset(&c[0], 0,
                       (sizeof(float)) << ((unsigned int)((unsigned char)4)));
          for (k = 0; k < 4; k++) {
            b_f = c[4 * k];
            b_n = (4 * k) + 1;
            nbitson = (4 * k) + 2;
            c_tmp = (4 * k) + 3;
            for (b_i = 0; b_i < 4; b_i++) {
              f1 = c_a[b_i + (4 * k)];
              b_f += cBuffer[4 * b_i] * f1;
              c[b_n] += cBuffer[(4 * b_i) + 1] * f1;
              c[nbitson] += cBuffer[(4 * b_i) + 2] * f1;
              c[c_tmp] += cBuffer[(4 * b_i) + 3] * f1;
            }
            c[4 * k] = b_f;
          }
        }
      }
    } else {
      (void)memcpy(&c_a[0], &b_a[0], 16U * (sizeof(float)));
      if (!rtIsInfF(b)) {
        bool lsb;
        lsb = true;
        float ed2;
        int exitg1;
        do {
          float b_f;
          float f1;
          int b_n;
          int c_tmp;
          int nbitson;
          exitg1 = 0;
          ed2 = floorf(e / 2.0F);
          if ((2.0F * ed2) != e) {
            if (lsb) {
              (void)memcpy(&c[0], &c_a[0], 16U * (sizeof(float)));
              lsb = false;
            } else {
              (void)memset(&cBuffer[0], 0,
                           (sizeof(float))
                               << ((unsigned int)((unsigned char)4)));
              for (k = 0; k < 4; k++) {
                b_f = cBuffer[4 * k];
                b_n = (4 * k) + 1;
                nbitson = (4 * k) + 2;
                c_tmp = (4 * k) + 3;
                for (b_i = 0; b_i < 4; b_i++) {
                  f1 = c_a[b_i + (4 * k)];
                  b_f += c[4 * b_i] * f1;
                  cBuffer[b_n] += c[(4 * b_i) + 1] * f1;
                  cBuffer[nbitson] += c[(4 * b_i) + 2] * f1;
                  cBuffer[c_tmp] += c[(4 * b_i) + 3] * f1;
                }
                cBuffer[4 * k] = b_f;
              }
              (void)memcpy(&c[0], &cBuffer[0], 16U * (sizeof(float)));
            }
          }
          if (ed2 == 0.0F) {
            exitg1 = 1;
          } else {
            e = ed2;
            (void)memset(&cBuffer[0], 0,
                         (sizeof(float)) << ((unsigned int)((unsigned char)4)));
            for (k = 0; k < 4; k++) {
              b_f = cBuffer[4 * k];
              b_n = (4 * k) + 1;
              nbitson = (4 * k) + 2;
              c_tmp = (4 * k) + 3;
              for (b_i = 0; b_i < 4; b_i++) {
                f1 = c_a[b_i + (4 * k)];
                b_f += c_a[4 * b_i] * f1;
                cBuffer[b_n] += c_a[(4 * b_i) + 1] * f1;
                cBuffer[nbitson] += c_a[(4 * b_i) + 2] * f1;
                cBuffer[c_tmp] += c_a[(4 * b_i) + 3] * f1;
              }
              cBuffer[4 * k] = b_f;
            }
            (void)memcpy(&c_a[0], &cBuffer[0], 16U * (sizeof(float)));
          }
        } while (exitg1 == ((int)((signed char)0)));
        if (b < 0.0F) {
          (void)memcpy(&c_a[0], &c[0], 16U * (sizeof(float)));
          inv(c_a, c);
        }
      } else {
        for (k = 0; k < 16; k++) {
          c[k] = rtNaNF;
        }
      }
    }
  }
}

static void mrdiv(const float A[30], const float b_B[9], float d_Y[30])
{
  float d_A[9];
  float a21;
  float maxval;
  int k;
  int r1;
  int r2;
  int r3;
  int rtemp;
  for (k = 0; k < 9; k++) {
    d_A[k] = b_B[k];
  }
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = fabsf(b_B[0]);
  a21 = fabsf(b_B[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }
  if (fabsf(b_B[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }
  d_A[r2] = b_B[r2] / b_B[r1];
  d_A[r3] /= d_A[r1];
  d_A[r2 + 3] -= d_A[r2] * d_A[r1 + 3];
  d_A[r3 + 3] -= d_A[r3] * d_A[r1 + 3];
  d_A[r2 + 6] -= d_A[r2] * d_A[r1 + 6];
  d_A[r3 + 6] -= d_A[r3] * d_A[r1 + 6];
  if (fabsf(d_A[r3 + 3]) > fabsf(d_A[r2 + 3])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }
  d_A[r3 + 3] /= d_A[r2 + 3];
  d_A[r3 + 6] -= d_A[r3 + 3] * d_A[r2 + 6];
  for (k = 0; k < 10; k++) {
    int b_Y_tmp;
    int c_Y_tmp;
    rtemp = k + (10 * r1);
    d_Y[rtemp] = A[k] / d_A[r1];
    b_Y_tmp = k + (10 * r2);
    d_Y[b_Y_tmp] = A[k + 10] - (d_Y[rtemp] * d_A[r1 + 3]);
    c_Y_tmp = k + (10 * r3);
    d_Y[c_Y_tmp] = A[k + 20] - (d_Y[rtemp] * d_A[r1 + 6]);
    d_Y[b_Y_tmp] /= d_A[r2 + 3];
    d_Y[c_Y_tmp] -= d_Y[b_Y_tmp] * d_A[r2 + 6];
    d_Y[c_Y_tmp] /= d_A[r3 + 6];
    d_Y[b_Y_tmp] -= d_Y[c_Y_tmp] * d_A[r3 + 3];
    d_Y[rtemp] -= d_Y[c_Y_tmp] * d_A[r3];
    d_Y[rtemp] -= d_Y[b_Y_tmp] * d_A[r2];
  }
}

static void recomputeBlockDiag(const float A[16], float F[16],
                               const int blockFormat[3])
{
  float avg;
  float expa11;
  float expa22;
  float x12;
  if (blockFormat[0] != ((int)((signed char)0))) {
    if (blockFormat[0] == ((int)((signed char)1))) {
      expa11 = expf(A[0]);
      expa22 = expf(A[5]);
      avg = (A[0] + A[5]) / 2.0F;
      if (fmaxf(avg, fabsf(A[0] - A[5]) / 2.0F) < 88.7228394F) {
        x12 = (A[5] - A[0]) / 2.0F;
        if (x12 == 0.0F) {
          x12 = 1.0F;
        } else {
          x12 = sinhf(x12) / x12;
        }
        x12 *= A[4] * expf(avg);
      } else {
        x12 = (A[4] * (expa22 - expa11)) / (A[5] - A[0]);
      }
      F[0] = expa11;
      F[4] = x12;
      F[5] = expa22;
    } else if (blockFormat[0] == ((int)((signed char)2))) {
      x12 = sqrtf(fabsf(A[1] * A[4]));
      avg = expf(A[0]);
      if (x12 == 0.0F) {
        expa11 = 1.0F;
      } else {
        expa11 = sinf(x12) / x12;
      }
      F[0] = avg * cosf(x12);
      F[1] = (avg * A[1]) * expa11;
      F[4] = (avg * A[4]) * expa11;
      F[5] = F[0];
    } else {
      /* no actions */
    }
  }
  if (blockFormat[1] != ((int)((signed char)0))) {
    if (blockFormat[1] == ((int)((signed char)1))) {
      expa11 = expf(A[5]);
      expa22 = expf(A[10]);
      avg = (A[5] + A[10]) / 2.0F;
      if (fmaxf(avg, fabsf(A[5] - A[10]) / 2.0F) < 88.7228394F) {
        x12 = (A[10] - A[5]) / 2.0F;
        if (x12 == 0.0F) {
          x12 = 1.0F;
        } else {
          x12 = sinhf(x12) / x12;
        }
        x12 *= A[9] * expf(avg);
      } else {
        x12 = (A[9] * (expa22 - expa11)) / (A[10] - A[5]);
      }
      F[5] = expa11;
      F[9] = x12;
      F[10] = expa22;
    } else if (blockFormat[1] == ((int)((signed char)2))) {
      x12 = sqrtf(fabsf(A[6] * A[9]));
      avg = expf(A[5]);
      if (x12 == 0.0F) {
        expa11 = 1.0F;
      } else {
        expa11 = sinf(x12) / x12;
      }
      F[5] = avg * cosf(x12);
      F[6] = (avg * A[6]) * expa11;
      F[9] = (avg * A[9]) * expa11;
      F[10] = F[5];
    } else {
      /* no actions */
    }
  }
  if (blockFormat[2] != ((int)((signed char)0))) {
    if (blockFormat[2] == ((int)((signed char)1))) {
      expa11 = expf(A[10]);
      expa22 = expf(A[15]);
      avg = (A[10] + A[15]) / 2.0F;
      if (fmaxf(avg, fabsf(A[10] - A[15]) / 2.0F) < 88.7228394F) {
        x12 = (A[15] - A[10]) / 2.0F;
        if (x12 == 0.0F) {
          x12 = 1.0F;
        } else {
          x12 = sinhf(x12) / x12;
        }
        x12 *= A[14] * expf(avg);
      } else {
        x12 = (A[14] * (expa22 - expa11)) / (A[15] - A[10]);
      }
      F[10] = expa11;
      F[14] = x12;
      F[15] = expa22;
    } else if (blockFormat[2] == ((int)((signed char)2))) {
      x12 = sqrtf(fabsf(A[11] * A[14]));
      avg = expf(A[10]);
      if (x12 == 0.0F) {
        expa11 = 1.0F;
      } else {
        expa11 = sinf(x12) / x12;
      }
      F[10] = avg * cosf(x12);
      F[11] = (avg * A[11]) * expa11;
      F[14] = (avg * A[14]) * expa11;
      F[15] = F[10];
    } else {
      /* no actions */
    }
  }
  if (blockFormat[2] == ((int)((signed char)0))) {
    F[15] = expf(A[15]);
  }
}

static void rotateRight(int n, float z[16], int iz0, const float cs[6], int ic0,
                        int is0)
{
  int b_i;
  int j;
  b_i = n - 1;
  for (j = b_i; j >= 1; j--) {
    float ctemp;
    float stemp;
    int offsetj;
    int offsetjp1;
    ctemp = cs[(ic0 + j) - 2];
    stemp = cs[(is0 + j) - 2];
    offsetj = (((j - 1) * 4) + iz0) - 2;
    offsetjp1 = ((j * 4) + iz0) - 2;
    if ((ctemp != 1.0F) || (stemp != 0.0F)) {
      float b_f;
      float temp;
      temp = z[offsetjp1 + 1];
      b_f = z[offsetj + 1];
      z[offsetjp1 + 1] = (ctemp * temp) - (stemp * b_f);
      b_f = (stemp * temp) + (ctemp * b_f);
      z[offsetj + 1] = b_f;
      temp = z[offsetjp1 + 2];
      b_f = z[offsetj + 2];
      z[offsetjp1 + 2] = (ctemp * temp) - (stemp * b_f);
      b_f = (stemp * temp) + (ctemp * b_f);
      z[offsetj + 2] = b_f;
      temp = z[offsetjp1 + 3];
      b_f = z[offsetj + 3];
      z[offsetjp1 + 3] = (ctemp * temp) - (stemp * b_f);
      b_f = (stemp * temp) + (ctemp * b_f);
      z[offsetj + 3] = b_f;
      temp = z[offsetjp1 + 4];
      b_f = z[offsetj + 4];
      z[offsetjp1 + 4] = (ctemp * temp) - (stemp * b_f);
      b_f = (stemp * temp) + (ctemp * b_f);
      z[offsetj + 4] = b_f;
    }
  }
}

static float rt_powf_snf(float u0, float u1)
{
  float y;
  if ((rtIsNaNF(u0)) || (rtIsNaNF(u1))) {
    y = rtNaNF;
  } else {
    float b_f;
    y = fabsf(u0);
    b_f = fabsf(u1);
    if (rtIsInfF(u1)) {
      if (y == 1.0F) {
        y = 1.0F;
      } else if (y > 1.0F) {
        if (u1 > 0.0F) {
          y = rtInfF;
        } else {
          y = 0.0F;
        }
      } else if (u1 > 0.0F) {
        y = 0.0F;
      } else {
        y = rtInfF;
      }
    } else if (b_f == 0.0F) {
      y = 1.0F;
    } else if (b_f == 1.0F) {
      if (u1 > 0.0F) {
        y = u0;
      } else {
        y = 1.0F / u0;
      }
    } else if (u1 == 2.0F) {
      y = u0 * u0;
    } else if ((u1 == 0.5F) && (u0 >= 0.0F)) {
      y = sqrtf(u0);
    } else if ((u0 < 0.0F) && (u1 > floorf(u1))) {
      y = rtNaNF;
    } else {
      y = powf(u0, u1);
    }
  }
  return y;
}

static float xdlaev2(float b_a, float b, float c, float *rt2, float *cs1,
                     float *sn1)
{
  float ab;
  float acmn;
  float acmx;
  float adf;
  float df;
  float rt1;
  float sm;
  float tb;
  int sgn1;
  int sgn2;
  sm = b_a + c;
  df = b_a - c;
  adf = fabsf(df);
  tb = b + b;
  ab = fabsf(tb);
  if (fabsf(b_a) > fabsf(c)) {
    acmx = b_a;
    acmn = c;
  } else {
    acmx = c;
    acmn = b_a;
  }
  if (adf > ab) {
    rt1 = ab / adf;
    adf *= sqrtf((rt1 * rt1) + 1.0F);
  } else if (adf < ab) {
    adf /= ab;
    adf = ab * sqrtf((adf * adf) + 1.0F);
  } else {
    adf = ab * 1.41421354F;
  }
  if (sm < 0.0F) {
    rt1 = 0.5F * (sm - adf);
    sgn1 = -1;
    *rt2 = ((acmx / rt1) * acmn) - ((b / rt1) * b);
  } else if (sm > 0.0F) {
    rt1 = 0.5F * (sm + adf);
    sgn1 = 1;
    *rt2 = ((acmx / rt1) * acmn) - ((b / rt1) * b);
  } else {
    rt1 = 0.5F * adf;
    *rt2 = -0.5F * adf;
    sgn1 = 1;
  }
  if (df >= 0.0F) {
    adf += df;
    sgn2 = 1;
  } else {
    adf = df - adf;
    sgn2 = -1;
  }
  if (fabsf(adf) > ab) {
    adf = (-tb) / adf;
    *sn1 = 1.0F / sqrtf((adf * adf) + 1.0F);
    *cs1 = adf * (*sn1);
  } else if (ab == 0.0F) {
    *cs1 = 1.0F;
    *sn1 = 0.0F;
  } else {
    adf = (-adf) / tb;
    *cs1 = 1.0F / sqrtf((adf * adf) + 1.0F);
    *sn1 = adf * (*cs1);
  }
  if (sgn1 == sgn2) {
    adf = *cs1;
    *cs1 = -(*sn1);
    *sn1 = adf;
  }
  return rt1;
}

static float xnrm2(int n, const float x[16], int ix0)
{
  float y;
  int k;
  y = 0.0F;
  if (n >= ((int)((signed char)1))) {
    if (n == ((int)((signed char)1))) {
      y = fabsf(x[ix0 - 1]);
    } else {
      float scale;
      int kend;
      scale = 1.29246971E-26F;
      kend = ix0 + n;
      for (k = ix0; k < kend; k++) {
        float absxk;
        absxk = fabsf(x[k - 1]);
        if (absxk > scale) {
          float t;
          t = scale / absxk;
          y = ((y * t) * t) + 1.0F;
          scale = absxk;
        } else {
          float t;
          t = absxk / scale;
          y += t * t;
        }
      }
      y = scale * sqrtf(y);
    }
  }
  return y;
}

static int xsyheev(float A[16], float b_W[4])
{
  float work[4];
  float absx;
  float anrm;
  int b_i;
  int c_i;
  int exitg1;
  int h_i;
  int info;
  int j;
  int offset;
  bool exitg2;
  info = 0;
  anrm = 0.0F;
  offset = 0;
  exitg2 = false;
  while ((!exitg2) && (offset < ((int)((signed char)4)))) {
    b_i = 0;
    do {
      exitg1 = 0;
      if (b_i <= offset) {
        absx = fabsf(A[b_i + (4 * offset)]);
        if (rtIsNaNF(absx)) {
          anrm = rtNaNF;
          exitg1 = 1;
        } else {
          if (absx > anrm) {
            anrm = absx;
          }
          b_i++;
        }
      } else {
        offset++;
        exitg1 = 2;
      }
    } while (exitg1 == ((int)((signed char)0)));
    if (exitg1 == ((int)((signed char)1))) {
      exitg2 = true;
    }
  }
  if ((rtIsInfF(anrm)) || (rtIsNaNF(anrm))) {
    b_W[0] = rtNaNF;
    b_W[1] = rtNaNF;
    b_W[2] = rtNaNF;
    b_W[3] = rtNaNF;
    for (j = 0; j < 16; j++) {
      A[j] = rtNaNF;
    }
  } else {
    float e[3];
    float tau[3];
    bool guard1;
    bool iscale;
    iscale = false;
    guard1 = false;
    if ((anrm > 0.0F) && (anrm < 3.14018486E-16F)) {
      iscale = true;
      anrm = 3.14018486E-16F / anrm;
      guard1 = true;
    } else if (anrm > 3.18452578E+15F) {
      iscale = true;
      anrm = 3.18452578E+15F / anrm;
      guard1 = true;
    } else {
      /* no actions */
    }
    if (guard1) {
      float cfromc;
      bool notdone;
      absx = anrm;
      cfromc = 1.0F;
      notdone = true;
      while (notdone) {
        float cfrom1;
        float cto1;
        float mul;
        cfrom1 = cfromc * 1.97215226E-31F;
        cto1 = absx / 5.0706024E+30F;
        if ((fabsf(cfrom1) > absx) && (absx != 0.0F)) {
          mul = 1.97215226E-31F;
          cfromc = cfrom1;
        } else if (cto1 > fabsf(cfromc)) {
          mul = 5.0706024E+30F;
          absx = cto1;
        } else {
          mul = absx / cfromc;
          notdone = false;
        }
        for (j = 0; j < 4; j++) {
          offset = (j * 4) - 1;
          A[offset + 1] *= mul;
          A[offset + 2] *= mul;
          A[offset + 3] *= mul;
          A[offset + 4] *= mul;
        }
      }
    }
    xzsyhetrd(A, b_W, e, tau);
    for (j = 2; j >= 0; j--) {
      offset = 4 * (j + 1);
      A[offset] = 0.0F;
      b_i = j + 3;
      for (c_i = b_i; c_i < 5; c_i++) {
        A[(c_i + offset) - 1] = A[(c_i + (4 * j)) - 1];
      }
    }
    A[0] = 1.0F;
    A[1] = 0.0F;
    A[2] = 0.0F;
    A[3] = 0.0F;
    work[0] = 0.0F;
    work[1] = 0.0F;
    work[2] = 0.0F;
    work[3] = 0.0F;
    for (h_i = 2; h_i >= 0; h_i--) {
      int iaii;
      iaii = (h_i + (h_i * 4)) + 10;
      if ((h_i + 1) < ((int)((signed char)3))) {
        int lastv;
        A[iaii - 5] = 1.0F;
        if (tau[h_i] != 0.0F) {
          lastv = 3 - h_i;
          offset = iaii - h_i;
          while ((lastv > ((int)((signed char)0))) && (A[offset - 3] == 0.0F)) {
            lastv--;
            offset--;
          }
          info = 1 - h_i;
          exitg2 = false;
          while ((!exitg2) && ((info + 1) > ((int)((signed char)0)))) {
            offset = iaii + (info * 4);
            b_i = offset;
            do {
              exitg1 = 0;
              if (b_i <= ((offset + lastv) - 1)) {
                if (A[b_i - 1] != 0.0F) {
                  exitg1 = 1;
                } else {
                  b_i++;
                }
              } else {
                info--;
                exitg1 = 2;
              }
            } while (exitg1 == ((int)((signed char)0)));
            if (exitg1 == ((int)((signed char)1))) {
              exitg2 = true;
            }
          }
        } else {
          lastv = 0;
          info = -1;
        }
        if (lastv > ((int)((signed char)0))) {
          if ((info + 1) != ((int)((signed char)0))) {
            (void)memset(&work[0], 0,
                         ((unsigned int)((int)(info + 1))) * (sizeof(float)));
            b_i = iaii + (4 * info);
            for (c_i = iaii; c_i <= b_i; c_i += 4) {
              absx = 0.0F;
              offset = c_i + lastv;
              for (j = c_i; j < offset; j++) {
                absx += A[j - 1] * A[((iaii + j) - c_i) - 5];
              }
              offset = asr_s32(c_i - iaii, 2U);
              work[offset] += absx;
            }
          }
          if (!((-tau[h_i]) == 0.0F)) {
            offset = iaii;
            for (j = 0; j <= info; j++) {
              absx = work[j];
              if (absx != 0.0F) {
                absx *= -tau[h_i];
                b_i = lastv + offset;
                for (c_i = offset; c_i < b_i; c_i++) {
                  A[c_i - 1] += A[((iaii + c_i) - offset) - 5] * absx;
                }
              }
              offset += 4;
            }
          }
        }
        offset = iaii - 3;
        b_i = (iaii - h_i) - 2;
        for (j = offset; j <= b_i; j++) {
          A[j - 1] *= -tau[h_i];
        }
      }
      A[iaii - 5] = 1.0F - tau[h_i];
      for (j = 0; j < h_i; j++) {
        A[(iaii - j) - 6] = 0.0F;
      }
    }
    info = xzsteqr(b_W, e, A);
    if (info != ((int)((signed char)0))) {
      b_W[0] = rtNaNF;
      b_W[1] = rtNaNF;
      b_W[2] = rtNaNF;
      b_W[3] = rtNaNF;
      for (j = 0; j < 16; j++) {
        A[j] = rtNaNF;
      }
    } else if (iscale) {
      absx = 1.0F / anrm;
      b_W[0] *= absx;
      b_W[1] *= absx;
      b_W[2] *= absx;
      b_W[3] *= absx;
    } else {
      /* no actions */
    }
  }
  return info;
}

static int xzgetrf(float A[16], int ipiv[4])
{
  int ijA;
  int info;
  int j;
  int k;
  ipiv[0] = 1;
  ipiv[1] = 2;
  ipiv[2] = 3;
  ipiv[3] = 4;
  info = 0;
  for (j = 0; j < 3; j++) {
    float smax;
    int b;
    int b_a;
    int jA;
    int jj;
    int jp1j;
    int mmj;
    mmj = 2 - j;
    b = j * 5;
    jj = j * 5;
    jp1j = b + 2;
    jA = 5 - j;
    b_a = 0;
    smax = fabsf(A[jj]);
    for (k = 2; k < jA; k++) {
      float s;
      s = fabsf(A[(b + k) - 1]);
      if (s > smax) {
        b_a = k - 1;
        smax = s;
      }
    }
    if (A[jj + b_a] != 0.0F) {
      if (b_a != ((int)((signed char)0))) {
        jA = j + b_a;
        ipiv[j] = jA + 1;
        smax = A[j];
        A[j] = A[jA];
        A[jA] = smax;
        smax = A[j + 4];
        A[j + 4] = A[jA + 4];
        A[jA + 4] = smax;
        smax = A[j + 8];
        A[j + 8] = A[jA + 8];
        A[jA + 8] = smax;
        smax = A[j + 12];
        A[j + 12] = A[jA + 12];
        A[jA + 12] = smax;
      }
      jA = (jj - j) + 4;
      for (k = jp1j; k <= jA; k++) {
        A[k - 1] /= A[jj];
      }
    } else {
      info = j + 1;
    }
    jA = jj;
    for (k = 0; k <= mmj; k++) {
      smax = A[(b + (k * 4)) + 4];
      if (smax != 0.0F) {
        b_a = jA + 6;
        jp1j = (jA - j) + 8;
        for (ijA = b_a; ijA <= jp1j; ijA++) {
          A[ijA - 1] += A[((jj + ijA) - jA) - 5] * (-smax);
        }
      }
      jA += 4;
    }
  }
  if ((info == ((int)((signed char)0))) && (!(A[15] != 0.0F))) {
    info = 4;
  }
  return info;
}

static float xzlartg(float b_f, float g, float *sn, float *b_r)
{
  float cs;
  float g1;
  cs = fabsf(b_f);
  g1 = fabsf(g);
  if (g == 0.0F) {
    cs = 1.0F;
    *sn = 0.0F;
    *b_r = b_f;
  } else if (b_f == 0.0F) {
    cs = 0.0F;
    if (g >= 0.0F) {
      *sn = 1.0F;
    } else {
      *sn = -1.0F;
    }
    *b_r = g1;
  } else if ((((cs > 1.08420217E-19F) && (cs < 6.5219088E+18F)) &&
              (g1 > 1.08420217E-19F)) &&
             (g1 < 6.5219088E+18F)) {
    float d;
    d = sqrtf((b_f * b_f) + (g * g));
    cs /= d;
    *b_r = d;
    if (!(b_f >= 0.0F)) {
      *b_r = -d;
    }
    *sn = g / (*b_r);
  } else {
    float d;
    float gs;
    g1 = fminf(8.50705917E+37F, fmaxf(1.17549435E-38F, fmaxf(cs, g1)));
    cs = b_f / g1;
    gs = g / g1;
    d = sqrtf((cs * cs) + (gs * gs));
    cs = fabsf(cs) / d;
    *b_r = d;
    if (!(b_f >= 0.0F)) {
      *b_r = -d;
    }
    *sn = gs / (*b_r);
    *b_r *= g1;
  }
  return cs;
}

static void xzlascl(float cfrom, float cto, int m, float A[4], int iA0)
{
  float cfromc;
  float ctoc;
  int b_i;
  bool notdone;
  cfromc = cfrom;
  ctoc = cto;
  notdone = true;
  while (notdone) {
    float cfrom1;
    float cto1;
    float mul;
    cfrom1 = cfromc * 1.97215226E-31F;
    cto1 = ctoc / 5.0706024E+30F;
    if ((fabsf(cfrom1) > fabsf(ctoc)) && (ctoc != 0.0F)) {
      mul = 1.97215226E-31F;
      cfromc = cfrom1;
    } else if (fabsf(cto1) > fabsf(cfromc)) {
      mul = 5.0706024E+30F;
      ctoc = cto1;
    } else {
      mul = ctoc / cfromc;
      notdone = false;
    }
    for (b_i = 0; b_i < m; b_i++) {
      int c_i;
      c_i = (iA0 + b_i) - 1;
      A[c_i] *= mul;
    }
  }
}

static int xzsteqr(float d[4], float e[3], float z[16])
{
  float work[6];
  float b_r;
  float g_tmp;
  float s;
  float temp;
  int b_i;
  int b_ii;
  int c_l1;
  int info;
  int jtot;
  info = 0;
  for (b_i = 0; b_i < 6; b_i++) {
    work[b_i] = 0.0F;
  }
  jtot = 0;
  c_l1 = 1;
  int exitg1;
  do {
    exitg1 = 0;
    if (c_l1 > ((int)((signed char)4))) {
      for (b_ii = 0; b_ii < 3; b_ii++) {
        float p;
        int k;
        k = b_ii;
        p = d[b_ii];
        for (b_i = b_ii + 2; b_i < 5; b_i++) {
          temp = d[b_i - 1];
          if (temp < p) {
            k = b_i - 1;
            p = temp;
          }
        }
        if (k != b_ii) {
          int ix;
          d[k] = d[b_ii];
          d[b_ii] = p;
          ix = b_ii * 4;
          k *= 4;
          temp = z[ix];
          z[ix] = z[k];
          z[k] = temp;
          temp = z[ix + 1];
          z[ix + 1] = z[k + 1];
          z[k + 1] = temp;
          temp = z[ix + 2];
          z[ix + 2] = z[k + 2];
          z[k + 2] = temp;
          temp = z[ix + 3];
          z[ix + 3] = z[k + 3];
          z[k + 3] = temp;
        }
      }
      exitg1 = 1;
    } else {
      int k_l;
      int lend;
      int lendsv;
      int lsv;
      int m;
      bool exitg2;
      if (c_l1 > ((int)((signed char)1))) {
        e[c_l1 - 2] = 0.0F;
      }
      m = c_l1;
      exitg2 = false;
      while ((!exitg2) && (m < ((int)((signed char)4)))) {
        temp = fabsf(e[m - 1]);
        if (temp == 0.0F) {
          exitg2 = true;
        } else if (temp <= ((sqrtf(fabsf(d[m - 1])) * sqrtf(fabsf(d[m]))) *
                            1.1920929E-7F)) {
          e[m - 1] = 0.0F;
          exitg2 = true;
        } else {
          m++;
        }
      }
      k_l = c_l1 - 1;
      lsv = c_l1;
      lend = m;
      lendsv = m;
      c_l1 = m + 1;
      if (m != (k_l + 1)) {
        float anorm;
        int c_n_tmp;
        int ix;
        int k;
        c_n_tmp = m - k_l;
        if (c_n_tmp <= ((int)((signed char)0))) {
          anorm = 0.0F;
        } else {
          anorm = fabsf(d[(k_l + c_n_tmp) - 1]);
          k = 0;
          exitg2 = false;
          while ((!exitg2) && (k <= (c_n_tmp - 2))) {
            ix = k_l + k;
            temp = fabsf(d[ix]);
            if (rtIsNaNF(temp)) {
              anorm = rtNaNF;
              exitg2 = true;
            } else {
              if (temp > anorm) {
                anorm = temp;
              }
              temp = fabsf(e[ix]);
              if (rtIsNaNF(temp)) {
                anorm = rtNaNF;
                exitg2 = true;
              } else {
                if (temp > anorm) {
                  anorm = temp;
                }
                k++;
              }
            }
          }
        }
        ix = 0;
        if (!(anorm == 0.0F)) {
          if ((rtIsInfF(anorm)) || (rtIsNaNF(anorm))) {
            d[0] = rtNaNF;
            d[1] = rtNaNF;
            d[2] = rtNaNF;
            d[3] = rtNaNF;
            for (b_i = 0; b_i < 16; b_i++) {
              z[b_i] = rtNaNF;
            }
            exitg1 = 1;
          } else {
            if (anorm > 3.07445744E+18F) {
              ix = 1;
              xzlascl(anorm, 3.07445744E+18F, c_n_tmp, d, k_l + 1);
              b_xzlascl(anorm, 3.07445744E+18F, c_n_tmp - 1, e, k_l + 1);
            } else if (anorm < 7.62939453E-6F) {
              ix = 2;
              xzlascl(anorm, 7.62939453E-6F, c_n_tmp, d, k_l + 1);
              b_xzlascl(anorm, 7.62939453E-6F, c_n_tmp - 1, e, k_l + 1);
            } else {
              /* no actions */
            }
            if (fabsf(d[m - 1]) < fabsf(d[k_l])) {
              lend = lsv;
              k_l = m - 1;
            }
            if (lend > (k_l + 1)) {
              int exitg4;
              do {
                exitg4 = 0;
                if ((k_l + 1) != lend) {
                  m = k_l + 1;
                  exitg2 = false;
                  while ((!exitg2) && (m < lend)) {
                    temp = fabsf(e[m - 1]);
                    if ((temp * temp) <=
                        (((1.42108547E-14F * fabsf(d[m - 1])) * fabsf(d[m])) +
                         1.17549435E-38F)) {
                      exitg2 = true;
                    } else {
                      m++;
                    }
                  }
                } else {
                  m = lend;
                }
                if (m < lend) {
                  e[m - 1] = 0.0F;
                }
                if (m == (k_l + 1)) {
                  k_l++;
                  if ((k_l + 1) > lend) {
                    exitg4 = 1;
                  }
                } else if (m == (k_l + 2)) {
                  d[k_l] = xdlaev2(d[k_l], e[k_l], d[k_l + 1], &temp,
                                   &work[k_l], &b_r);
                  d[k_l + 1] = temp;
                  work[k_l + 3] = b_r;
                  rotateRight(2, z, (k_l * 4) + 1, work, k_l + 1, k_l + 4);
                  e[k_l] = 0.0F;
                  k_l += 2;
                  if ((k_l + 1) > lend) {
                    exitg4 = 1;
                  }
                } else if (jtot == ((int)((signed char)120))) {
                  exitg4 = 1;
                } else {
                  float c;
                  float g;
                  float p;
                  jtot++;
                  g = (d[k_l + 1] - d[k_l]) / (2.0F * e[k_l]);
                  temp = fabsf(g);
                  if (temp < 1.0F) {
                    temp = sqrtf((temp * temp) + 1.0F);
                  } else if (temp > 1.0F) {
                    b_r = 1.0F / temp;
                    temp *= sqrtf((b_r * b_r) + 1.0F);
                  } else {
                    temp *= 1.41421354F;
                  }
                  if (!(g >= 0.0F)) {
                    temp = -temp;
                  }
                  g = (d[m - 1] - d[k_l]) + (e[k_l] / (g + temp));
                  s = 1.0F;
                  c = 1.0F;
                  p = 0.0F;
                  k = m - 1;
                  for (b_i = k; b_i >= (k_l + 1); b_i--) {
                    float b;
                    temp = e[b_i - 1];
                    b = c * temp;
                    c = xzlartg(g, s * temp, &s, &b_r);
                    if (b_i != (m - 1)) {
                      e[b_i] = b_r;
                    }
                    g = d[b_i] - p;
                    temp = ((d[b_i - 1] - g) * s) + ((2.0F * c) * b);
                    p = s * temp;
                    d[b_i] = g + p;
                    g = (c * temp) - b;
                    work[b_i - 1] = c;
                    work[b_i + 2] = -s;
                  }
                  rotateRight(m - k_l, z, (k_l * 4) + 1, work, k_l + 1,
                              k_l + 4);
                  d[k_l] -= p;
                  e[k_l] = g;
                }
              } while (exitg4 == ((int)((signed char)0)));
            } else {
              int exitg3;
              do {
                exitg3 = 0;
                if ((k_l + 1) != lend) {
                  m = k_l + 1;
                  exitg2 = false;
                  while ((!exitg2) && (m > lend)) {
                    temp = fabsf(e[m - 2]);
                    if ((temp * temp) <= (((1.42108547E-14F * fabsf(d[m - 1])) *
                                           fabsf(d[m - 2])) +
                                          1.17549435E-38F)) {
                      exitg2 = true;
                    } else {
                      m--;
                    }
                  }
                } else {
                  m = lend;
                }
                if (m > lend) {
                  e[m - 2] = 0.0F;
                }
                if (m == (k_l + 1)) {
                  k_l--;
                  if ((k_l + 1) < lend) {
                    exitg3 = 1;
                  }
                } else if (m == k_l) {
                  d[k_l - 1] = xdlaev2(d[k_l - 1], e[k_l - 1], d[k_l], &temp,
                                       &work[m - 1], &b_r);
                  d[k_l] = temp;
                  work[m + 2] = b_r;
                  b_rotateRight(2, z, ((k_l - 1) * 4) + 1, work, m, m + 3);
                  e[k_l - 1] = 0.0F;
                  k_l -= 2;
                  if ((k_l + 1) < lend) {
                    exitg3 = 1;
                  }
                } else if (jtot == ((int)((signed char)120))) {
                  exitg3 = 1;
                } else {
                  float c;
                  float g;
                  float p;
                  jtot++;
                  g_tmp = e[k_l - 1];
                  g = (d[k_l - 1] - d[k_l]) / (2.0F * g_tmp);
                  temp = fabsf(g);
                  if (temp < 1.0F) {
                    temp = sqrtf((temp * temp) + 1.0F);
                  } else if (temp > 1.0F) {
                    b_r = 1.0F / temp;
                    temp *= sqrtf((b_r * b_r) + 1.0F);
                  } else {
                    temp *= 1.41421354F;
                  }
                  if (!(g >= 0.0F)) {
                    temp = -temp;
                  }
                  g = (d[m - 1] - d[k_l]) + (g_tmp / (g + temp));
                  s = 1.0F;
                  c = 1.0F;
                  p = 0.0F;
                  for (b_i = m; b_i <= k_l; b_i++) {
                    float b;
                    temp = e[b_i - 1];
                    b = c * temp;
                    c = xzlartg(g, s * temp, &s, &g_tmp);
                    if (b_i != m) {
                      e[b_i - 2] = g_tmp;
                    }
                    g = d[b_i - 1] - p;
                    temp = ((d[b_i] - g) * s) + ((2.0F * c) * b);
                    p = s * temp;
                    d[b_i - 1] = g + p;
                    g = (c * temp) - b;
                    work[b_i - 1] = c;
                    work[b_i + 2] = s;
                  }
                  b_rotateRight((k_l - m) + 2, z, ((m - 1) * 4) + 1, work, m,
                                m + 3);
                  d[k_l] -= p;
                  e[k_l - 1] = g;
                }
              } while (exitg3 == ((int)((signed char)0)));
            }
            if (ix == ((int)((signed char)1))) {
              k = lendsv - lsv;
              xzlascl(3.07445744E+18F, anorm, k + 1, d, lsv);
              b_xzlascl(3.07445744E+18F, anorm, k, e, lsv);
            } else if (ix == ((int)((signed char)2))) {
              k = lendsv - lsv;
              xzlascl(7.62939453E-6F, anorm, k + 1, d, lsv);
              b_xzlascl(7.62939453E-6F, anorm, k, e, lsv);
            } else {
              /* no actions */
            }
            if (jtot >= ((int)((signed char)120))) {
              if (e[0] != 0.0F) {
                info = 1;
              }
              if (e[1] != 0.0F) {
                info++;
              }
              if (e[2] != 0.0F) {
                info++;
              }
              exitg1 = 1;
            }
          }
        }
      }
    }
  } while (exitg1 == ((int)((signed char)0)));
  return info;
}

static void xzsyhetrd(float A[16], float b_D[4], float b_E[3], float tau[3])
{
  int b_i;
  int b_ii;
  int k;
  for (b_i = 0; b_i < 3; b_i++) {
    float beta1;
    float taui;
    float temp2;
    float xnorm;
    int alpha_tmp_tmp;
    int ix0;
    int knt;
    int u0;
    alpha_tmp_tmp = b_i + (4 * b_i);
    temp2 = A[alpha_tmp_tmp + 1];
    u0 = b_i + 3;
    if (u0 > ((int)((signed char)4))) {
      u0 = 4;
    }
    ix0 = (b_i * 4) + u0;
    taui = 0.0F;
    xnorm = xnrm2(2 - b_i, A, ix0);
    if (xnorm != 0.0F) {
      beta1 = fabsf(A[alpha_tmp_tmp + 1]);
      xnorm = fabsf(xnorm);
      if (beta1 < xnorm) {
        beta1 /= xnorm;
        beta1 = xnorm * sqrtf((beta1 * beta1) + 1.0F);
      } else if (beta1 > xnorm) {
        xnorm /= beta1;
        beta1 *= sqrtf((xnorm * xnorm) + 1.0F);
      } else if (rtIsNaNF(xnorm)) {
        beta1 = rtNaNF;
      } else {
        beta1 *= 1.41421354F;
      }
      if (temp2 >= 0.0F) {
        beta1 = -beta1;
      }
      if (fabsf(beta1) < 9.86076132E-32F) {
        knt = 0;
        u0 = (ix0 - b_i) + 1;
        do {
          knt++;
          for (k = ix0; k <= u0; k++) {
            A[k - 1] *= 1.01412048E+31F;
          }
          beta1 *= 1.01412048E+31F;
          temp2 *= 1.01412048E+31F;
        } while ((fabsf(beta1) < 9.86076132E-32F) &&
                 (knt < ((int)((signed char)20))));
        xnorm = fabsf(temp2);
        beta1 = fabsf(xnrm2(2 - b_i, A, ix0));
        if (xnorm < beta1) {
          xnorm /= beta1;
          beta1 *= sqrtf((xnorm * xnorm) + 1.0F);
        } else if (xnorm > beta1) {
          beta1 /= xnorm;
          beta1 = xnorm * sqrtf((beta1 * beta1) + 1.0F);
        } else if (rtIsNaNF(beta1)) {
          beta1 = rtNaNF;
        } else {
          beta1 = xnorm * 1.41421354F;
        }
        if (temp2 >= 0.0F) {
          beta1 = -beta1;
        }
        taui = (beta1 - temp2) / beta1;
        xnorm = 1.0F / (temp2 - beta1);
        for (k = ix0; k <= u0; k++) {
          A[k - 1] *= xnorm;
        }
        for (k = 0; k < knt; k++) {
          beta1 *= 9.86076132E-32F;
        }
        temp2 = beta1;
      } else {
        taui = (beta1 - temp2) / beta1;
        xnorm = 1.0F / (temp2 - beta1);
        u0 = (ix0 - b_i) + 1;
        for (k = ix0; k <= u0; k++) {
          A[k - 1] *= xnorm;
        }
        temp2 = beta1;
      }
    }
    b_E[b_i] = temp2;
    if (taui != 0.0F) {
      int b_tau_tmp;
      int c_i;
      int tau_tmp;
      A[alpha_tmp_tmp + 1] = 1.0F;
      for (k = b_i + 1; k < 4; k++) {
        tau[k - 1] = 0.0F;
      }
      u0 = 2 - b_i;
      knt = 4 - b_i;
      for (k = 0; k <= u0; k++) {
        ix0 = b_i + k;
        beta1 = taui * A[(ix0 + (4 * b_i)) + 1];
        temp2 = 0.0F;
        tau_tmp = 4 * (ix0 + 1);
        tau[ix0] += beta1 * A[(ix0 + tau_tmp) + 1];
        c_i = k + 2;
        for (b_ii = c_i; b_ii < knt; b_ii++) {
          b_tau_tmp = b_i + b_ii;
          xnorm = A[b_tau_tmp + tau_tmp];
          tau[b_tau_tmp - 1] += beta1 * xnorm;
          temp2 += xnorm * A[b_tau_tmp + (4 * b_i)];
        }
        tau[ix0] += taui * temp2;
      }
      u0 = 2 - b_i;
      xnorm = 0.0F;
      for (k = 0; k <= u0; k++) {
        xnorm += tau[b_i + k] * A[(alpha_tmp_tmp + k) + 1];
      }
      xnorm *= -0.5F * taui;
      if (!(xnorm == 0.0F)) {
        u0 = 3 - b_i;
        for (k = 0; k < u0; k++) {
          knt = b_i + k;
          tau[knt] += xnorm * A[(alpha_tmp_tmp + k) + 1];
        }
      }
      tau_tmp = 2 - b_i;
      c_i = 4 - b_i;
      for (b_ii = 0; b_ii <= tau_tmp; b_ii++) {
        u0 = b_i + b_ii;
        beta1 = A[(u0 + (4 * b_i)) + 1];
        temp2 = tau[u0];
        xnorm = temp2 * beta1;
        b_tau_tmp = 4 * (u0 + 1);
        u0 = (u0 + b_tau_tmp) + 1;
        A[u0] = (A[u0] - xnorm) - xnorm;
        u0 = b_ii + 2;
        for (k = u0; k < c_i; k++) {
          knt = b_i + k;
          ix0 = knt + b_tau_tmp;
          A[ix0] =
              (A[ix0] - (tau[knt - 1] * beta1)) - (A[knt + (4 * b_i)] * temp2);
        }
      }
    }
    A[alpha_tmp_tmp + 1] = b_E[b_i];
    b_D[b_i] = A[alpha_tmp_tmp];
    tau[b_i] = taui;
  }
  b_D[3] = A[15];
}

void filter_entry(filter_entryStackData *SD, float x[10], float b_P[100],
                  struct0_T *mem, float dt, const struct2_T *sens_in,
                  bool is_init)
{
  static const float fv2[16] = {
      1.0E-12F, 0.0F, 0.0F,     0.0F, 0.0F, 1.0E-12F, 0.0F, 0.0F,
      0.0F,     0.0F, 1.0E-12F, 0.0F, 0.0F, 0.0F,     0.0F, 1.0E-12F};
  static const float fv[9] = {0.5F, 0.0F, 0.0F, 0.0F, 0.5F,
                              0.0F, 0.0F, 0.0F, 0.5F};
  static const signed char iv[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  double Qc[49];
  float F[100];
  float b_Q[100];
  float c_F[100];
  float b_G[70];
  float c_G[70];
  float d_H[30];
  float d_P[30];
  float e_H[30];
  float c_K[10];
  float dq[4];
  float w[3];
  int b_i;
  int c_i;
  int i1;
  signed char d_I[100];
  if (!is_init) {
    float R_bn[9];
    float q[4];
    float cp;
    float sp;
    float tr;
    tr = mem->sens_filt.baro;
    if (sens_in->accel.status) {
      mem->sens_filt.accel[0] =
          (((float)((double)(1.0 - SD->pd->params.a_fast))) *
           mem->sens_filt.accel[0]) +
          (((float)SD->pd->params.a_fast) * sens_in->accel.meas[0]);
      mem->sens_filt.accel[1] =
          (((float)((double)(1.0 - SD->pd->params.a_fast))) *
           mem->sens_filt.accel[1]) +
          (((float)SD->pd->params.a_fast) * sens_in->accel.meas[1]);
      mem->sens_filt.accel[2] =
          (((float)((double)(1.0 - SD->pd->params.a_fast))) *
           mem->sens_filt.accel[2]) +
          (((float)SD->pd->params.a_fast) * sens_in->accel.meas[2]);
    }
    if (sens_in->gyro.status) {
      mem->sens_filt.gyro[0] =
          (((float)((double)(1.0 - SD->pd->params.a_fast))) *
           mem->sens_filt.gyro[0]) +
          (((float)SD->pd->params.a_fast) * sens_in->gyro.meas[0]);
      mem->sens_filt.gyro[1] =
          (((float)((double)(1.0 - SD->pd->params.a_fast))) *
           mem->sens_filt.gyro[1]) +
          (((float)SD->pd->params.a_fast) * sens_in->gyro.meas[1]);
      mem->sens_filt.gyro[2] =
          (((float)((double)(1.0 - SD->pd->params.a_fast))) *
           mem->sens_filt.gyro[2]) +
          (((float)SD->pd->params.a_fast) * sens_in->gyro.meas[2]);
    }
    if (sens_in->mag.status) {
      mem->sens_filt.mag[0] =
          (((float)((double)(1.0 - SD->pd->params.a_slow))) *
           mem->sens_filt.mag[0]) +
          (((float)SD->pd->params.a_slow) * sens_in->mag.meas[0]);
      mem->sens_filt.mag[1] =
          (((float)((double)(1.0 - SD->pd->params.a_slow))) *
           mem->sens_filt.mag[1]) +
          (((float)SD->pd->params.a_slow) * sens_in->mag.meas[1]);
      mem->sens_filt.mag[2] =
          (((float)((double)(1.0 - SD->pd->params.a_slow))) *
           mem->sens_filt.mag[2]) +
          (((float)SD->pd->params.a_slow) * sens_in->mag.meas[2]);
    }
    if (sens_in->baro.status) {
      tr = (((float)((double)(1.0 - SD->pd->params.a_slow))) *
            mem->sens_filt.baro) +
           (((float)SD->pd->params.a_slow) * sens_in->baro.meas);
    }
    mem->sens_filt.baro = tr;
    for (b_i = 0; b_i < 10; b_i++) {
      x[b_i] = 0.0F;
    }
    float theta[3];
    tr = b_norm(mem->sens_filt.accel);
    sp = (-mem->sens_filt.accel[0]) / tr;
    w[0] = sp;
    cp = sp * mem->sens_filt.mag[0];
    sp = (-mem->sens_filt.accel[1]) / tr;
    w[1] = sp;
    cp += sp * mem->sens_filt.mag[1];
    sp = (-mem->sens_filt.accel[2]) / tr;
    cp += sp * mem->sens_filt.mag[2];
    theta[0] = mem->sens_filt.mag[0] - (cp * w[0]);
    theta[1] = mem->sens_filt.mag[1] - (cp * w[1]);
    theta[2] = mem->sens_filt.mag[2] - (cp * sp);
    tr = b_norm(theta);
    cp = theta[0] / tr;
    theta[0] = cp;
    R_bn[0] = cp;
    cp = theta[1] / tr;
    theta[1] = cp;
    R_bn[3] = cp;
    cp = theta[2] / tr;
    R_bn[1] = (w[1] * cp) - (theta[1] * sp);
    R_bn[4] = (theta[0] * sp) - (w[0] * cp);
    R_bn[7] = (w[0] * theta[1]) - (theta[0] * w[1]);
    tr = (R_bn[0] + R_bn[4]) + sp;
    if (tr > 0.0F) {
      tr = sqrtf(tr + 1.0F) * 2.0F;
      q[0] = 0.25F * tr;
      q[1] = (w[1] - R_bn[7]) / tr;
      q[2] = (cp - w[0]) / tr;
      q[3] = (R_bn[1] - R_bn[3]) / tr;
    } else if ((R_bn[0] > R_bn[4]) && (R_bn[0] > sp)) {
      tr = sqrtf(((R_bn[0] + 1.0F) - R_bn[4]) - sp) * 2.0F;
      q[0] = (w[1] - R_bn[7]) / tr;
      q[1] = 0.25F * tr;
      q[2] = (R_bn[1] + R_bn[3]) / tr;
      q[3] = (w[0] + cp) / tr;
    } else if (R_bn[4] > sp) {
      tr = sqrtf(((R_bn[4] + 1.0F) - R_bn[0]) - sp) * 2.0F;
      q[0] = (cp - w[0]) / tr;
      q[1] = (R_bn[1] + R_bn[3]) / tr;
      q[2] = 0.25F * tr;
      q[3] = (w[1] + R_bn[7]) / tr;
    } else {
      tr = sqrtf(((sp + 1.0F) - R_bn[0]) - R_bn[4]) * 2.0F;
      q[0] = (R_bn[1] - R_bn[3]) / tr;
      q[1] = (w[0] + cp) / tr;
      q[2] = (w[1] + R_bn[7]) / tr;
      q[3] = 0.25F * tr;
    }
    tr = c_norm(q);
    q[0] /= tr;
    q[1] /= tr;
    q[2] /= tr;
    q[3] /= tr;
    if (q[0] < 0.0F) {
      q[0] = -q[0];
      q[1] = -q[1];
      q[2] = -q[2];
      q[3] = -q[3];
    }
    x[0] = q[0];
    x[1] = q[1];
    x[2] = q[2];
    x[3] = q[3];
    x[4] = mem->sens_filt.gyro[0];
    x[5] = mem->sens_filt.gyro[1];
    x[6] = mem->sens_filt.gyro[2];
    for (b_i = 0; b_i < 100; b_i++) {
      b_P[b_i] = (float)SD->pd->params.P0[b_i];
    }
  } else {
    float Psi[12];
    float c_H[10];
    float R_bn[9];
    float b_a[9];
    float c_R[9];
    float q[4];
    float theta[3];
    float a_tmp;
    float b_a_tmp;
    float b_f;
    float c_a_tmp;
    float cp;
    float d;
    float d_R;
    float sp;
    float tr;
    int F_tmp;
    int H_tmp;
    if (sens_in->gyro.status) {
      double b_v[7];
      double e_a;
      double f_a;
      float c_a[16];
      float fv1[16];
      float K_tmp;
      signed char e_I[9];
      q[0] = x[0];
      q[1] = x[1];
      q[2] = x[2];
      q[3] = x[3];
      b_f = sens_in->gyro.meas[0] - x[4];
      w[0] = b_f;
      theta[0] = b_f * dt;
      b_f = sens_in->gyro.meas[1] - x[5];
      w[1] = b_f;
      theta[1] = b_f * dt;
      b_f = sens_in->gyro.meas[2] - x[6];
      theta[2] = b_f * dt;
      tr = b_norm(theta);
      if (((double)tr) < 1.0E-8) {
        dq[0] = 1.0F;
        for (b_i = 0; b_i < 3; b_i++) {
          tr = theta[b_i];
          dq[b_i + 1] = tr / 2.0F;
          Psi[4 * b_i] = (-tr) / 4.0F;
          Psi[(4 * b_i) + 1] = fv[3 * b_i];
          Psi[(4 * b_i) + 2] = fv[(3 * b_i) + 1];
          Psi[(4 * b_i) + 3] = fv[(3 * b_i) + 2];
        }
      } else {
        cp = tr / 2.0F;
        sp = sinf(cp);
        cp = cosf(cp);
        dq[0] = cp;
        d_R = theta[0] / tr;
        theta[0] = d_R;
        dq[1] = d_R * sp;
        d_R = theta[1] / tr;
        theta[1] = d_R;
        dq[2] = d_R * sp;
        d_R = theta[2] / tr;
        theta[2] = d_R;
        dq[3] = d_R * sp;
        for (b_i = 0; b_i < 3; b_i++) {
          R_bn[3 * b_i] = theta[0] * theta[b_i];
          R_bn[(3 * b_i) + 1] = theta[1] * theta[b_i];
          R_bn[(3 * b_i) + 2] = d_R * theta[b_i];
        }
        b_a_tmp = -0.5F * sp;
        c_a_tmp = sp / tr;
        for (b_i = 0; b_i < 9; b_i++) {
          e_I[b_i] = 0;
        }
        tr = 0.5F * cp;
        e_I[0] = 1;
        Psi[0] = b_a_tmp * theta[0];
        e_I[4] = 1;
        Psi[4] = b_a_tmp * theta[1];
        e_I[8] = 1;
        Psi[8] = b_a_tmp * d_R;
        for (b_i = 0; b_i < 3; b_i++) {
          cp = R_bn[3 * b_i];
          Psi[(4 * b_i) + 1] =
              (c_a_tmp * (((float)e_I[3 * b_i]) - cp)) + (tr * cp);
          F_tmp = (3 * b_i) + 1;
          cp = R_bn[F_tmp];
          Psi[(4 * b_i) + 2] =
              (c_a_tmp * (((float)e_I[F_tmp]) - cp)) + (tr * cp);
          F_tmp = (3 * b_i) + 2;
          cp = R_bn[F_tmp];
          Psi[(4 * b_i) + 3] =
              (c_a_tmp * (((float)e_I[F_tmp]) - cp)) + (tr * cp);
        }
      }
      tr = x[0];
      c_K[1] =
          ((tr * dq[1]) + (dq[0] * x[1])) + ((x[2] * dq[3]) - (dq[2] * x[3]));
      c_K[4] = x[4];
      c_K[2] =
          ((tr * dq[2]) + (dq[0] * x[2])) + ((dq[1] * x[3]) - (x[1] * dq[3]));
      c_K[5] = x[5];
      c_K[3] =
          ((tr * dq[3]) + (dq[0] * x[3])) + ((x[1] * dq[2]) - (dq[1] * x[2]));
      c_K[6] = x[6];
      c_K[0] =
          (x[0] * dq[0]) - (((x[1] * dq[1]) + (x[2] * dq[2])) + (x[3] * dq[3]));
      c_K[7] = x[7];
      c_K[8] = x[8] + (x[7] * dt);
      K_tmp = dt * dt;
      c_K[9] = (x[9] + (x[8] * dt)) + ((0.5F * x[7]) * K_tmp);
      for (b_i = 0; b_i < 10; b_i++) {
        x[b_i] = c_K[b_i];
      }
      (void)memset(&F[0], 0, 100U * (sizeof(float)));
      tr = 0.5F * dt;
      c_a_tmp = tr * 0.0F;
      c_a[0] = c_a_tmp;
      b_a_tmp = tr * (-w[0]);
      c_a[4] = b_a_tmp;
      sp = tr * (-w[1]);
      c_a[8] = sp;
      d = tr * (-b_f);
      c_a[12] = d;
      a_tmp = tr * w[0];
      c_a[1] = a_tmp;
      c_a[5] = c_a_tmp;
      d_R = tr * b_f;
      c_a[9] = d_R;
      c_a[13] = sp;
      cp = tr * w[1];
      c_a[2] = cp;
      c_a[6] = d;
      c_a[10] = c_a_tmp;
      c_a[14] = a_tmp;
      c_a[3] = d_R;
      c_a[7] = cp;
      c_a[11] = b_a_tmp;
      c_a[15] = c_a_tmp;
      expm(c_a, fv1);
      for (b_i = 0; b_i < 4; b_i++) {
        F[10 * b_i] = fv1[4 * b_i];
        F[(10 * b_i) + 1] = fv1[(4 * b_i) + 1];
        F[(10 * b_i) + 2] = fv1[(4 * b_i) + 2];
        F[(10 * b_i) + 3] = fv1[(4 * b_i) + 3];
      }
      tr = (-dt) * q[0];
      c_a[0] = tr;
      d_R = (-dt) * (-q[1]);
      c_a[4] = d_R;
      sp = (-dt) * (-q[2]);
      c_a[8] = sp;
      cp = (-dt) * (-q[3]);
      c_a[12] = cp;
      c_a_tmp = (-dt) * q[1];
      c_a[1] = c_a_tmp;
      c_a[5] = tr;
      c_a[9] = cp;
      cp = (-dt) * q[2];
      c_a[13] = cp;
      c_a[2] = cp;
      cp = (-dt) * q[3];
      c_a[6] = cp;
      c_a[10] = tr;
      c_a[14] = d_R;
      c_a[3] = cp;
      c_a[7] = sp;
      c_a[11] = c_a_tmp;
      c_a[15] = tr;
      for (b_i = 0; b_i < 3; b_i++) {
        F_tmp = 10 * (b_i + 4);
        cp = 0.0F;
        tr = 0.0F;
        sp = 0.0F;
        c_a_tmp = 0.0F;
        for (c_i = 0; c_i < 4; c_i++) {
          d_R = Psi[c_i + (4 * b_i)];
          cp += c_a[4 * c_i] * d_R;
          tr += c_a[(4 * c_i) + 1] * d_R;
          sp += c_a[(4 * c_i) + 2] * d_R;
          c_a_tmp += c_a[(4 * c_i) + 3] * d_R;
        }
        F[F_tmp + 3] = c_a_tmp;
        F[F_tmp + 2] = sp;
        F[F_tmp + 1] = tr;
        F[F_tmp] = cp;
      }
      for (b_i = 0; b_i < 9; b_i++) {
        e_I[b_i] = 0;
      }
      e_I[0] = 1;
      e_I[4] = 1;
      e_I[8] = 1;
      for (b_i = 0; b_i < 9; b_i++) {
        R_bn[b_i] = (float)e_I[b_i];
      }
      for (b_i = 0; b_i < 3; b_i++) {
        F_tmp = 10 * (b_i + 4);
        F[F_tmp + 4] = R_bn[3 * b_i];
        F[F_tmp + 5] = R_bn[(3 * b_i) + 1];
        F[F_tmp + 6] = R_bn[(3 * b_i) + 2];
      }
      F[77] = 1.0F;
      F[78] = dt;
      F[88] = 1.0F;
      F[79] = 0.5F * K_tmp;
      F[89] = dt;
      F[99] = 1.0F;
      (void)memset(&b_G[0], 0, 70U * (sizeof(float)));
      b_G[67] = 1.0F;
      e_a = SD->pd->b_params.sg * SD->pd->b_params.sg;
      f_a = SD->pd->b_params.sbg * SD->pd->b_params.sbg;
      for (b_i = 0; b_i < 3; b_i++) {
        F_tmp = 10 * (b_i + 4);
        b_G[10 * b_i] = F[F_tmp];
        b_G[(10 * b_i) + 1] = F[F_tmp + 1];
        b_G[(10 * b_i) + 2] = F[F_tmp + 2];
        b_G[(10 * b_i) + 3] = F[F_tmp + 3];
        F_tmp = 10 * (b_i + 3);
        b_G[F_tmp + 4] = R_bn[3 * b_i];
        b_G[F_tmp + 5] = R_bn[(3 * b_i) + 1];
        b_G[F_tmp + 6] = R_bn[(3 * b_i) + 2];
        b_v[b_i] = e_a;
        b_v[b_i + 3] = f_a;
      }
      e_a = SD->pd->b_params.sa * SD->pd->b_params.sa;
      b_v[6] = e_a;
      (void)memset(&Qc[0], 0, 49U * (sizeof(double)));
      for (b_i = 0; b_i < 7; b_i++) {
        Qc[b_i + (7 * b_i)] = b_v[b_i];
      }
      (void)memset(&c_G[0], 0, 70U * (sizeof(float)));
      for (b_i = 0; b_i < 7; b_i++) {
        for (c_i = 0; c_i < 7; c_i++) {
          tr = (float)Qc[c_i + (7 * b_i)];
          for (i1 = 0; i1 < 10; i1++) {
            F_tmp = i1 + (10 * b_i);
            c_G[F_tmp] += b_G[i1 + (10 * c_i)] * tr;
          }
        }
      }
      (void)memset(&b_Q[0], 0, 100U * (sizeof(float)));
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 7; c_i++) {
          tr = b_G[b_i + (10 * c_i)];
          for (i1 = 0; i1 < 10; i1++) {
            F_tmp = i1 + (10 * b_i);
            b_Q[F_tmp] += c_G[i1 + (10 * c_i)] * tr;
          }
        }
      }
      for (b_i = 0; b_i < 100; b_i++) {
        b_Q[b_i] *= dt;
      }
      tr = rt_powf_snf(dt, 3.0F);
      b_Q[77] = ((float)e_a) * dt;
      cp = ((float)e_a) * (K_tmp / 2.0F);
      b_Q[87] = cp;
      sp = ((float)e_a) * (tr / 6.0F);
      b_Q[97] = sp;
      b_Q[78] = cp;
      b_Q[88] = ((float)e_a) * (tr / 3.0F);
      tr = ((float)e_a) * (rt_powf_snf(dt, 4.0F) / 8.0F);
      b_Q[98] = tr;
      b_Q[79] = sp;
      b_Q[89] = tr;
      b_Q[99] = ((float)e_a) * (rt_powf_snf(dt, 5.0F) / 20.0F);
      for (b_i = 0; b_i < 4; b_i++) {
        b_Q[10 * b_i] += fv2[4 * b_i];
        F_tmp = (10 * b_i) + 1;
        b_Q[F_tmp] += fv2[(4 * b_i) + 1];
        F_tmp = (10 * b_i) + 2;
        b_Q[F_tmp] += fv2[(4 * b_i) + 2];
        F_tmp = (10 * b_i) + 3;
        b_Q[F_tmp] += fv2[(4 * b_i) + 3];
      }
      (void)memset(&c_F[0], 0, 100U * (sizeof(float)));
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 10; c_i++) {
          tr = b_P[c_i + (10 * b_i)];
          for (i1 = 0; i1 < 10; i1++) {
            F_tmp = i1 + (10 * b_i);
            c_F[F_tmp] += F[i1 + (10 * c_i)] * tr;
          }
        }
      }
      for (c_i = 0; c_i < 10; c_i++) {
        for (i1 = 0; i1 < 10; i1++) {
          tr = 0.0F;
          for (b_i = 0; b_i < 10; b_i++) {
            tr += c_F[c_i + (10 * b_i)] * F[i1 + (10 * b_i)];
          }
          F_tmp = c_i + (10 * i1);
          b_P[F_tmp] = tr + b_Q[F_tmp];
        }
      }
      tr = c_norm(&x[0]);
      x[0] /= tr;
      x[1] /= tr;
      x[2] /= tr;
      x[3] /= tr;
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 10; c_i++) {
          F_tmp = c_i + (10 * b_i);
          F[F_tmp] = (b_P[F_tmp] + b_P[b_i + (10 * c_i)]) / 2.0F;
        }
      }
      (void)memcpy(&b_P[0], &F[0], 100U * (sizeof(float)));
    }
    if (sens_in->accel.status) {
      float d_K[30];
      a_tmp = b_norm(mem->sens_filt.accel);
      w[0] = 0.0F;
      w[1] = 0.0F;
      w[2] = (-x[7]) - a_tmp;
      tr = c_norm(&x[0]);
      q[0] = x[0] / tr;
      q[1] = x[1] / tr;
      q[2] = x[2] / tr;
      q[3] = x[3] / tr;
      cp = 0.0F;
      sp = 2.0F * q[0];
      for (b_i = 0; b_i < 3; b_i++) {
        tr = q[b_i + 1];
        cp += tr * tr;
        c_R[3 * b_i] = tr * q[1];
        c_R[(3 * b_i) + 1] = tr * q[2];
        c_R[(3 * b_i) + 2] = tr * q[3];
      }
      cp = (q[0] * q[0]) - cp;
      tr = sp * 0.0F;
      b_a[0] = tr;
      b_a[1] = sp * (-q[3]);
      b_a[2] = sp * q[2];
      b_a[3] = sp * q[3];
      b_a[4] = tr;
      b_a[5] = sp * (-q[1]);
      b_a[6] = sp * (-q[2]);
      b_a[7] = sp * q[1];
      b_a[8] = tr;
      for (b_i = 0; b_i < 3; b_i++) {
        R_bn[3 * b_i] =
            ((cp * ((float)iv[b_i])) + (2.0F * c_R[3 * b_i])) + b_a[3 * b_i];
        F_tmp = (3 * b_i) + 1;
        R_bn[F_tmp] =
            ((cp * ((float)iv[b_i + 3])) + (2.0F * c_R[F_tmp])) + b_a[F_tmp];
        F_tmp = (3 * b_i) + 2;
        R_bn[F_tmp] =
            ((cp * ((float)iv[b_i + 6])) + (2.0F * c_R[F_tmp])) + b_a[F_tmp];
      }
      (void)memset(&d_H[0], 0, 30U * (sizeof(float)));
      cp = 2.0F * x[0];
      tr = cp * 0.0F;
      d_H[0] = tr - (2.0F * ((x[2] * w[2]) - (x[3] * 0.0F)));
      d_H[1] = tr - (2.0F * ((x[3] * 0.0F) - (x[1] * w[2])));
      sp = cp * w[2];
      d_H[2] = sp - (2.0F * ((x[1] * 0.0F) - (x[2] * 0.0F)));
      b_a_tmp = 2.0F * (((x[1] * 0.0F) + (x[2] * 0.0F)) + (w[2] * x[3]));
      b_a[0] = tr;
      b_a[3] = cp * (-w[2]);
      b_a[6] = tr;
      b_a[1] = sp;
      b_a[4] = tr;
      cp *= -0.0F;
      b_a[7] = cp;
      for (b_i = 0; b_i < 3; b_i++) {
        int b_H_tmp;
        F_tmp = (3 * b_i) + 2;
        b_a[F_tmp] = cp;
        tr = x[b_i + 1];
        sp = -0.0F * tr;
        H_tmp = 3 * (b_i + 1);
        c_a_tmp = w[b_i];
        d_H[H_tmp] = ((sp + (b_a_tmp * ((float)iv[3 * b_i]))) +
                      ((2.0F * x[1]) * c_a_tmp)) +
                     b_a[3 * b_i];
        b_H_tmp = (3 * b_i) + 1;
        d_H[H_tmp + 1] = ((sp + (b_a_tmp * ((float)iv[b_H_tmp]))) +
                          ((2.0F * x[2]) * c_a_tmp)) +
                         b_a[b_H_tmp];
        d_H[H_tmp + 2] =
            ((((-2.0F * w[2]) * tr) + (b_a_tmp * ((float)iv[F_tmp]))) +
             ((2.0F * x[3]) * w[b_i])) +
            cp;
        d_H[b_i + 21] = -R_bn[b_i + 6];
      }
      tr = fabsf(b_norm(sens_in->accel.meas) - a_tmp) / 0.5F;
      tr *= tr;
      for (b_i = 0; b_i < 9; b_i++) {
        c_R[b_i] = ((float)SD->pd->c_params.R_accel[b_i]) * (tr + 1.0F);
      }
      for (b_i = 0; b_i < 3; b_i++) {
        for (c_i = 0; c_i < 10; c_i++) {
          d_K[c_i + (10 * b_i)] = d_H[b_i + (3 * c_i)];
        }
      }
      (void)memset(&e_H[0], 0, 30U * (sizeof(float)));
      for (b_i = 0; b_i < 10; b_i++) {
        tr = e_H[3 * b_i];
        F_tmp = (3 * b_i) + 1;
        H_tmp = (3 * b_i) + 2;
        for (c_i = 0; c_i < 10; c_i++) {
          cp = b_P[c_i + (10 * b_i)];
          tr += d_H[3 * c_i] * cp;
          e_H[F_tmp] += d_H[(3 * c_i) + 1] * cp;
          e_H[H_tmp] += d_H[(3 * c_i) + 2] * cp;
        }
        e_H[3 * b_i] = tr;
      }
      (void)memset(&d_P[0], 0, 30U * (sizeof(float)));
      for (i1 = 0; i1 < 3; i1++) {
        for (b_i = 0; b_i < 10; b_i++) {
          tr = d_K[b_i + (10 * i1)];
          for (c_i = 0; c_i < 10; c_i++) {
            F_tmp = c_i + (10 * i1);
            d_P[F_tmp] += b_P[c_i + (10 * b_i)] * tr;
          }
        }
        for (c_i = 0; c_i < 3; c_i++) {
          tr = 0.0F;
          for (b_i = 0; b_i < 10; b_i++) {
            tr += e_H[i1 + (3 * b_i)] * d_K[b_i + (10 * c_i)];
          }
          F_tmp = i1 + (3 * c_i);
          b_a[F_tmp] = tr + c_R[F_tmp];
        }
      }
      mrdiv(d_P, b_a, d_K);
      for (b_i = 0; b_i < 3; b_i++) {
        theta[b_i] = sens_in->accel.meas[b_i] -
                     (((R_bn[b_i] * 0.0F) + (R_bn[b_i + 3] * 0.0F)) +
                      (R_bn[b_i + 6] * w[2]));
      }
      cp = theta[0];
      tr = theta[1];
      sp = theta[2];
      for (b_i = 0; b_i < 10; b_i++) {
        x[b_i] +=
            ((d_K[b_i] * cp) + (d_K[b_i + 10] * tr)) + (d_K[b_i + 20] * sp);
      }
      (void)memset(&d_I[0], 0, 100U * (sizeof(signed char)));
      for (b_i = 0; b_i < 10; b_i++) {
        d_I[b_i + (10 * b_i)] = 1;
      }
      for (b_i = 0; b_i < 10; b_i++) {
        cp = d_K[b_i];
        tr = d_K[b_i + 10];
        sp = d_K[b_i + 20];
        for (c_i = 0; c_i < 10; c_i++) {
          F_tmp = b_i + (10 * c_i);
          F[F_tmp] = ((float)d_I[F_tmp]) -
                     (((cp * d_H[3 * c_i]) + (tr * d_H[(3 * c_i) + 1])) +
                      (sp * d_H[(3 * c_i) + 2]));
        }
      }
      (void)memset(&c_F[0], 0, 100U * (sizeof(float)));
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 10; c_i++) {
          tr = b_P[c_i + (10 * b_i)];
          for (i1 = 0; i1 < 10; i1++) {
            F_tmp = i1 + (10 * b_i);
            c_F[F_tmp] += F[i1 + (10 * c_i)] * tr;
          }
        }
      }
      (void)memset(&d_P[0], 0, 30U * (sizeof(float)));
      for (b_i = 0; b_i < 3; b_i++) {
        for (c_i = 0; c_i < 3; c_i++) {
          tr = c_R[c_i + (3 * b_i)];
          for (i1 = 0; i1 < 10; i1++) {
            F_tmp = i1 + (10 * b_i);
            d_P[F_tmp] += d_K[i1 + (10 * c_i)] * tr;
          }
        }
      }
      (void)memset(&b_P[0], 0, 100U * (sizeof(float)));
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 10; c_i++) {
          tr = F[b_i + (10 * c_i)];
          for (i1 = 0; i1 < 10; i1++) {
            F_tmp = i1 + (10 * b_i);
            b_P[F_tmp] += c_F[i1 + (10 * c_i)] * tr;
          }
        }
      }
      (void)memset(&F[0], 0, 100U * (sizeof(float)));
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 3; c_i++) {
          tr = d_K[b_i + (10 * c_i)];
          for (i1 = 0; i1 < 10; i1++) {
            F_tmp = i1 + (10 * b_i);
            F[F_tmp] += d_P[i1 + (10 * c_i)] * tr;
          }
        }
      }
      for (b_i = 0; b_i < 100; b_i++) {
        b_P[b_i] += F[b_i];
      }
      tr = c_norm(&x[0]);
      x[0] /= tr;
      x[1] /= tr;
      x[2] /= tr;
      x[3] /= tr;
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 10; c_i++) {
          F_tmp = c_i + (10 * b_i);
          F[F_tmp] = (b_P[F_tmp] + b_P[b_i + (10 * c_i)]) / 2.0F;
        }
      }
      (void)memcpy(&b_P[0], &F[0], 100U * (sizeof(float)));
    }
    if (sens_in->mag.status) {
      tr = c_norm(&x[0]);
      q[0] = x[0] / tr;
      q[1] = x[1] / tr;
      q[2] = x[2] / tr;
      q[3] = x[3] / tr;
      cp = 0.0F;
      sp = 2.0F * q[0];
      for (b_i = 0; b_i < 3; b_i++) {
        tr = q[b_i + 1];
        cp += tr * tr;
        c_R[3 * b_i] = q[1] * tr;
        c_R[(3 * b_i) + 1] = q[2] * tr;
        c_R[(3 * b_i) + 2] = q[3] * tr;
      }
      cp = (q[0] * q[0]) - cp;
      tr = sp * 0.0F;
      b_a[0] = tr;
      b_a[3] = sp * (-q[3]);
      b_a[6] = sp * q[2];
      b_a[1] = sp * q[3];
      b_a[4] = tr;
      b_a[7] = sp * (-q[1]);
      b_a[2] = sp * (-q[2]);
      b_a[5] = sp * q[1];
      b_a[8] = tr;
      for (b_i = 0; b_i < 9; b_i++) {
        c_R[b_i] = ((cp * ((float)iv[b_i])) + (2.0F * c_R[b_i])) + b_a[b_i];
      }
      (void)memset(&w[0], 0, 3U * (sizeof(float)));
      a_tmp = w[0];
      b_f = w[1];
      for (b_i = 0; b_i < 3; b_i++) {
        cp = sens_in->mag.meas[b_i];
        a_tmp += c_R[3 * b_i] * cp;
        b_f += c_R[(3 * b_i) + 1] * cp;
      }
      d = (a_tmp * a_tmp) + (b_f * b_f);
      for (b_i = 0; b_i < 12; b_i++) {
        Psi[b_i] = 0.0F;
      }
      tr = 2.0F * x[0];
      sp = tr * sens_in->mag.meas[0];
      Psi[0] = sp + (2.0F * ((x[2] * sens_in->mag.meas[2]) -
                             (sens_in->mag.meas[1] * x[3])));
      cp = tr * sens_in->mag.meas[1];
      Psi[1] = cp + (2.0F * ((sens_in->mag.meas[0] * x[3]) -
                             (x[1] * sens_in->mag.meas[2])));
      c_a_tmp = tr * sens_in->mag.meas[2];
      Psi[2] = c_a_tmp + (2.0F * ((x[1] * sens_in->mag.meas[1]) -
                                  (sens_in->mag.meas[0] * x[2])));
      b_a_tmp =
          2.0F *
          (((sens_in->mag.meas[0] * x[1]) + (sens_in->mag.meas[1] * x[2])) +
           (sens_in->mag.meas[2] * x[3]));
      d_R = tr * 0.0F;
      b_a[0] = d_R;
      b_a[3] = tr * (-sens_in->mag.meas[2]);
      b_a[6] = cp;
      b_a[1] = c_a_tmp;
      b_a[4] = d_R;
      b_a[7] = tr * (-sens_in->mag.meas[0]);
      b_a[2] = tr * (-sens_in->mag.meas[1]);
      b_a[5] = sp;
      b_a[8] = d_R;
      for (b_i = 0; b_i < 3; b_i++) {
        cp = x[b_i + 1];
        H_tmp = 3 * (b_i + 1);
        Psi[H_tmp] = ((((-2.0F * sens_in->mag.meas[0]) * cp) +
                       (b_a_tmp * ((float)iv[3 * b_i]))) +
                      ((2.0F * x[1]) * sens_in->mag.meas[b_i])) -
                     b_a[3 * b_i];
        F_tmp = (3 * b_i) + 1;
        Psi[H_tmp + 1] = ((((-2.0F * sens_in->mag.meas[1]) * cp) +
                           (b_a_tmp * ((float)iv[F_tmp]))) +
                          ((2.0F * x[2]) * sens_in->mag.meas[b_i])) -
                         b_a[F_tmp];
        F_tmp = (3 * b_i) + 2;
        Psi[H_tmp + 2] = ((((-2.0F * sens_in->mag.meas[2]) * cp) +
                           (b_a_tmp * ((float)iv[F_tmp]))) +
                          ((2.0F * x[3]) * sens_in->mag.meas[b_i])) -
                         b_a[F_tmp];
      }
      for (b_i = 0; b_i < 10; b_i++) {
        c_H[b_i] = 0.0F;
      }
      cp = (-b_f) / d;
      tr = a_tmp / d;
      for (b_i = 0; b_i < 4; b_i++) {
        c_H[b_i] = ((cp * Psi[3 * b_i]) + (tr * Psi[(3 * b_i) + 1])) +
                   (0.0F * Psi[(3 * b_i) + 2]);
      }
      d_R = (((float)SD->pd->d_params.R_mag) *
             (((sens_in->mag.meas[0] * sens_in->mag.meas[0]) +
               (sens_in->mag.meas[1] * sens_in->mag.meas[1])) +
              (sens_in->mag.meas[2] * sens_in->mag.meas[2]))) /
            d;
      tr = b_atan2(b_f, a_tmp);
      sp = b_atan2(sinf(-tr), cosf(-tr));
      (void)memset(&c_K[0], 0, 10U * (sizeof(float)));
      cp = 0.0F;
      for (b_i = 0; b_i < 10; b_i++) {
        tr = c_K[b_i];
        for (c_i = 0; c_i < 10; c_i++) {
          tr += c_H[c_i] * b_P[c_i + (10 * b_i)];
        }
        c_K[b_i] = tr;
        cp += tr * c_H[b_i];
      }
      tr = cp + d_R;
      for (b_i = 0; b_i < 10; b_i++) {
        cp = 0.0F;
        for (c_i = 0; c_i < 10; c_i++) {
          cp += b_P[b_i + (10 * c_i)] * c_H[c_i];
        }
        cp /= tr;
        c_K[b_i] = cp;
        x[b_i] += cp * sp;
      }
      (void)memset(&d_I[0], 0, 100U * (sizeof(signed char)));
      for (b_i = 0; b_i < 10; b_i++) {
        d_I[b_i + (10 * b_i)] = 1;
      }
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 10; c_i++) {
          F_tmp = c_i + (10 * b_i);
          F[F_tmp] = ((float)d_I[F_tmp]) - (c_K[c_i] * c_H[b_i]);
        }
      }
      (void)memset(&c_F[0], 0, 100U * (sizeof(float)));
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 10; c_i++) {
          tr = b_P[c_i + (10 * b_i)];
          for (i1 = 0; i1 < 10; i1++) {
            F_tmp = i1 + (10 * b_i);
            c_F[F_tmp] += F[i1 + (10 * c_i)] * tr;
          }
        }
      }
      (void)memset(&b_P[0], 0, 100U * (sizeof(float)));
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 10; c_i++) {
          tr = F[b_i + (10 * c_i)];
          for (i1 = 0; i1 < 10; i1++) {
            F_tmp = i1 + (10 * b_i);
            b_P[F_tmp] += c_F[i1 + (10 * c_i)] * tr;
          }
        }
      }
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 10; c_i++) {
          F[c_i + (10 * b_i)] = (c_K[c_i] * d_R) * c_K[b_i];
        }
      }
      for (b_i = 0; b_i < 100; b_i++) {
        b_P[b_i] += F[b_i];
      }
      tr = c_norm(&x[0]);
      x[0] /= tr;
      x[1] /= tr;
      x[2] /= tr;
      x[3] /= tr;
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 10; c_i++) {
          F_tmp = c_i + (10 * b_i);
          F[F_tmp] = (b_P[F_tmp] + b_P[b_i + (10 * c_i)]) / 2.0F;
        }
      }
      (void)memcpy(&b_P[0], &F[0], 100U * (sizeof(float)));
    }
    if (sens_in->baro.status) {
      for (b_i = 0; b_i < 10; b_i++) {
        c_H[b_i] = 0.0F;
      }
      c_H[9] = ((((-mem->sens_filt.baro) * 5.2559F) * 0.0065F) / 288.15F) *
               rt_powf_snf(1.0F - ((0.0065F * x[9]) / 288.15F), 4.2559F);
      sp = sens_in->baro.meas -
           (mem->sens_filt.baro *
            rt_powf_snf(1.0F - ((0.0065F * x[9]) / 288.15F), 5.2559F));
      (void)memset(&c_K[0], 0, 10U * (sizeof(float)));
      cp = 0.0F;
      for (b_i = 0; b_i < 10; b_i++) {
        tr = c_K[b_i];
        for (c_i = 0; c_i < 10; c_i++) {
          tr += c_H[c_i] * b_P[c_i + (10 * b_i)];
        }
        c_K[b_i] = tr;
        cp += tr * c_H[b_i];
      }
      tr = cp + ((float)SD->pd->e_params.R_baro);
      for (b_i = 0; b_i < 10; b_i++) {
        cp = 0.0F;
        for (c_i = 0; c_i < 10; c_i++) {
          cp += b_P[b_i + (10 * c_i)] * c_H[c_i];
        }
        cp /= tr;
        c_K[b_i] = cp;
        x[b_i] += cp * sp;
      }
      (void)memset(&d_I[0], 0, 100U * (sizeof(signed char)));
      for (b_i = 0; b_i < 10; b_i++) {
        d_I[b_i + (10 * b_i)] = 1;
      }
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 10; c_i++) {
          F_tmp = c_i + (10 * b_i);
          F[F_tmp] = ((float)d_I[F_tmp]) - (c_K[c_i] * c_H[b_i]);
        }
      }
      (void)memset(&c_F[0], 0, 100U * (sizeof(float)));
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 10; c_i++) {
          tr = b_P[c_i + (10 * b_i)];
          for (i1 = 0; i1 < 10; i1++) {
            F_tmp = i1 + (10 * b_i);
            c_F[F_tmp] += F[i1 + (10 * c_i)] * tr;
          }
        }
      }
      (void)memset(&b_P[0], 0, 100U * (sizeof(float)));
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 10; c_i++) {
          tr = F[b_i + (10 * c_i)];
          for (i1 = 0; i1 < 10; i1++) {
            F_tmp = i1 + (10 * b_i);
            b_P[F_tmp] += c_F[i1 + (10 * c_i)] * tr;
          }
        }
      }
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 10; c_i++) {
          F[c_i + (10 * b_i)] =
              (c_K[c_i] * ((float)SD->pd->e_params.R_baro)) * c_K[b_i];
        }
      }
      for (b_i = 0; b_i < 100; b_i++) {
        b_P[b_i] += F[b_i];
      }
      tr = c_norm(&x[0]);
      x[0] /= tr;
      x[1] /= tr;
      x[2] /= tr;
      x[3] /= tr;
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 10; c_i++) {
          F_tmp = c_i + (10 * b_i);
          F[F_tmp] = (b_P[F_tmp] + b_P[b_i + (10 * c_i)]) / 2.0F;
        }
      }
      (void)memcpy(&b_P[0], &F[0], 100U * (sizeof(float)));
    }
  }
}

void filter_entry_initialize(filter_entryStackData *SD)
{
  filter_init_init(SD);
  ekf_dynamics_init(SD);
  ekf_innov_accel_init(SD);
  ekf_innov_mag_init(SD);
  ekf_innov_baro_init(SD);
}
