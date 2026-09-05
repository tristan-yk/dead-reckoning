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

static double b_atan2(double y, double x);

static double b_log2(double x);

static double b_norm(const double x[4]);

static void b_rotateRight(int n, double z[16], int iz0, const double cs[6],
                          int ic0, int is0);

static void b_xzlascl(double cfrom, double cto, int m, double A[3], int iA0);

static void ekf_dynamics_init(filter_entryStackData *SD);

static void ekf_innov_accel_init(filter_entryStackData *SD);

static void ekf_innov_baro_init(filter_entryStackData *SD);

static void ekf_innov_mag_init(filter_entryStackData *SD);

static void expm(double A[16], double F[16]);

static void filter_init_init(filter_entryStackData *SD);

static int getExpmParams(const double A[16], double A2[16], double A4[16],
                         double A6[16], double *s);

static void inv(const double x[16], double y[16]);

static void mpower(const double b_a[16], double b, double c[16]);

static void mrdiv(const double A[30], const double b_B[9], double d_Y[30]);

static void recomputeBlockDiag(const double A[16], double F[16],
                               const int blockFormat[3]);

static void rotateRight(int n, double z[16], int iz0, const double cs[6],
                        int ic0, int is0);

static double rt_powd_snf(double u0, double u1);

static double xdlaev2(double b_a, double b, double c, double *rt2, double *cs1,
                      double *sn1);

static double xnrm2(int n, const double x[16], int ix0);

static int xsyheev(double A[16], double b_W[4]);

static int xzgetrf(double A[16], int ipiv[4]);

static double xzlartg(double b_f, double g, double *sn, double *b_r);

static void xzlascl(double cfrom, double cto, int m, double A[4], int iA0);

static int xzsteqr(double d[4], double e[3], double z[16]);

static void xzsyhetrd(double A[16], double b_D[4], double b_E[3],
                      double tau[3]);

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

static double b_atan2(double y, double x)
{
  double b_r;
  if ((rtIsNaN(y)) || (rtIsNaN(x))) {
    b_r = rtNaN;
  } else if ((rtIsInf(y)) && (rtIsInf(x))) {
    int b_i;
    int i1;
    if (y > 0.0) {
      b_i = 1;
    } else {
      b_i = -1;
    }
    if (x > 0.0) {
      i1 = 1;
    } else {
      i1 = -1;
    }
    b_r = atan2((double)b_i, (double)i1);
  } else if (x == 0.0) {
    if (y > 0.0) {
      b_r = RT_PI / 2.0;
    } else if (y < 0.0) {
      b_r = -(RT_PI / 2.0);
    } else {
      b_r = 0.0;
    }
  } else {
    b_r = atan2(y, x);
  }
  return b_r;
}

static double b_log2(double x)
{
  double b_f;
  int eint;
  if (x == 0.0) {
    b_f = rtMinusInf;
  } else if (x < 0.0) {
    b_f = rtNaN;
  } else if ((!rtIsInf(x)) && (!rtIsNaN(x))) {
    b_f = frexp(x, &eint);
    if (b_f == 0.5) {
      b_f = ((double)eint) - 1.0;
    } else if ((eint == ((int)((signed char)1))) && (b_f < 0.75)) {
      b_f = log(2.0 * b_f) / 0.69314718055994529;
    } else {
      b_f = (log(b_f) / 0.69314718055994529) + ((double)eint);
    }
  } else {
    b_f = x;
  }
  return b_f;
}

static double b_norm(const double x[4])
{
  double absxk;
  double scale;
  double t;
  double y;
  scale = 3.3121686421112381E-170;
  absxk = fabs(x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }
  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = ((y * t) * t) + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = fabs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = ((y * t) * t) + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = fabs(x[3]);
  if (absxk > scale) {
    t = scale / absxk;
    y = ((y * t) * t) + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  return scale * sqrt(y);
}

static void b_rotateRight(int n, double z[16], int iz0, const double cs[6],
                          int ic0, int is0)
{
  int j;
  for (j = 0; j <= (n - 2); j++) {
    double ctemp;
    double stemp;
    int offsetj;
    int offsetjp1;
    ctemp = cs[(ic0 + j) - 1];
    stemp = cs[(is0 + j) - 1];
    offsetj = ((j * 4) + iz0) - 2;
    offsetjp1 = (((j + 1) * 4) + iz0) - 2;
    if ((ctemp != 1.0) || (stemp != 0.0)) {
      double d;
      double temp;
      temp = z[offsetjp1 + 1];
      d = z[offsetj + 1];
      z[offsetjp1 + 1] = (ctemp * temp) - (stemp * d);
      d = (stemp * temp) + (ctemp * d);
      z[offsetj + 1] = d;
      temp = z[offsetjp1 + 2];
      d = z[offsetj + 2];
      z[offsetjp1 + 2] = (ctemp * temp) - (stemp * d);
      d = (stemp * temp) + (ctemp * d);
      z[offsetj + 2] = d;
      temp = z[offsetjp1 + 3];
      d = z[offsetj + 3];
      z[offsetjp1 + 3] = (ctemp * temp) - (stemp * d);
      d = (stemp * temp) + (ctemp * d);
      z[offsetj + 3] = d;
      temp = z[offsetjp1 + 4];
      d = z[offsetj + 4];
      z[offsetjp1 + 4] = (ctemp * temp) - (stemp * d);
      d = (stemp * temp) + (ctemp * d);
      z[offsetj + 4] = d;
    }
  }
}

static void b_xzlascl(double cfrom, double cto, int m, double A[3], int iA0)
{
  double cfromc;
  double ctoc;
  int b_i;
  bool notdone;
  cfromc = cfrom;
  ctoc = cto;
  notdone = true;
  while (notdone) {
    double cfrom1;
    double cto1;
    double mul;
    cfrom1 = cfromc * 2.0041683600089728E-292;
    cto1 = ctoc / 4.9896007738368E+291;
    if ((fabs(cfrom1) > fabs(ctoc)) && (ctoc != 0.0)) {
      mul = 2.0041683600089728E-292;
      cfromc = cfrom1;
    } else if (fabs(cto1) > fabs(cfromc)) {
      mul = 4.9896007738368E+291;
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

static void expm(double A[16], double F[16])
{
  double A2[16];
  double A6[16];
  double V[16];
  double exptj;
  double s;
  int b_k;
  int c_i;
  int k;
  bool recomputeDiags;
  recomputeDiags = true;
  for (k = 0; k < 16; k++) {
    if (recomputeDiags) {
      exptj = A[k];
      if ((rtIsInf(exptj)) || (rtIsNaN(exptj))) {
        recomputeDiags = false;
      }
    } else {
      recomputeDiags = false;
    }
  }
  if (!recomputeDiags) {
    for (k = 0; k < 16; k++) {
      F[k] = rtNaN;
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
          if ((b_i != j) && (!(A[b_i + (4 * j)] == 0.0))) {
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
      (void)memset(&F[0], 0, 16U * (sizeof(double)));
      F[0] = exp(A[0]);
      F[5] = exp(A[5]);
      F[10] = exp(A[10]);
      F[15] = exp(A[15]);
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
        double w[4];
        (void)memcpy(&A2[0], &A[0], 16U * (sizeof(double)));
        (void)xsyheev(A2, w);
        for (k = 0; k < 4; k++) {
          exptj = exp(w[k]);
          F[4 * k] = A2[4 * k] * exptj;
          j = (4 * k) + 1;
          F[j] = A2[j] * exptj;
          j = (4 * k) + 2;
          F[j] = A2[j] * exptj;
          j = (4 * k) + 3;
          F[j] = A2[j] * exptj;
        }
        (void)memset(&A6[0], 0,
                     (sizeof(double)) << ((unsigned int)((unsigned char)4)));
        for (k = 0; k < 4; k++) {
          int A6_tmp;
          exptj = A6[4 * k];
          j = (4 * k) + 1;
          b_i = (4 * k) + 2;
          A6_tmp = (4 * k) + 3;
          for (b_k = 0; b_k < 4; b_k++) {
            double y;
            y = A2[k + (4 * b_k)];
            exptj += F[4 * b_k] * y;
            A6[j] += F[(4 * b_k) + 1] * y;
            A6[b_i] += F[(4 * b_k) + 2] * y;
            A6[A6_tmp] += F[(4 * b_k) + 3] * y;
          }
          A6[4 * k] = exptj;
        }
        (void)memcpy(&F[0], &A6[0], 16U * (sizeof(double)));
        for (k = 0; k < 4; k++) {
          A6[4 * k] = (F[4 * k] + F[k]) / 2.0;
          j = (4 * k) + 1;
          A6[j] = (F[j] + F[k + 4]) / 2.0;
          j = (4 * k) + 2;
          A6[j] = (F[j] + F[k + 8]) / 2.0;
          j = (4 * k) + 3;
          A6[j] = (F[j] + F[k + 12]) / 2.0;
        }
        (void)memcpy(&F[0], &A6[0], 16U * (sizeof(double)));
      } else {
        double A4[16];
        double b_A6[16];
        double b_y;
        double c_y;
        double y;
        int ipiv[4];
        int blockFormat[3];
        int A6_tmp;
        int F_tmp;
        recomputeDiags = true;
        j = 3;
        while (recomputeDiags && (j <= ((int)((signed char)4)))) {
          b_i = j;
          while (recomputeDiags && (b_i <= ((int)((signed char)4)))) {
            recomputeDiags = (A[(b_i + (4 * (j - 3))) - 1] == 0.0);
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
            if (exptj != 0.0) {
              if (((j + 1) != ((int)((signed char)3))) &&
                  (A[(j + (4 * (j + 1))) + 2] != 0.0)) {
                recomputeDiags = false;
                exitg2 = true;
              } else {
                A6_tmp = j + (4 * (j + 1));
                if (A[b_i] != A[A6_tmp + 1]) {
                  recomputeDiags = false;
                  exitg2 = true;
                } else {
                  y = A[A6_tmp];
                  if (rtIsNaN(exptj)) {
                    b_y = rtNaN;
                  } else if (exptj < 0.0) {
                    b_y = -1.0;
                  } else {
                    b_y = (exptj > 0.0) ? 1.0 : 0.0;
                  }
                  if (rtIsNaN(y)) {
                    exptj = rtNaN;
                  } else if (y < 0.0) {
                    exptj = -1.0;
                  } else {
                    exptj = (y > 0.0) ? 1.0 : 0.0;
                  }
                  if ((b_y * exptj) != -1.0) {
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
        if (s != 0.0) {
          exptj = rt_powd_snf(2.0, s);
          y = rt_powd_snf(2.0, 2.0 * s);
          b_y = rt_powd_snf(2.0, 4.0 * s);
          c_y = rt_powd_snf(2.0, 6.0 * s);
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
            if (A[(j + (4 * j)) + 1] != 0.0) {
              blockFormat[j] = 2;
              blockFormat[j + 1] = 0;
              j += 2;
            } else if (A[(j + (4 * (j + 1))) + 2] == 0.0) {
              blockFormat[j] = 1;
              j++;
            } else {
              blockFormat[j] = 0;
              j++;
            }
          }
          if (A[11] != 0.0) {
            blockFormat[2] = 2;
          } else if ((blockFormat[1] == ((int)((signed char)0))) ||
                     (blockFormat[1] == ((int)((signed char)1)))) {
            blockFormat[2] = 1;
          } else {
            /* no actions */
          }
        }
        if (b_i == ((int)((signed char)3))) {
          (void)memcpy(&F[0], &A2[0], 16U * (sizeof(double)));
          F[0] = A2[0] + 60.0;
          F[5] += 60.0;
          F[10] += 60.0;
          F[15] += 60.0;
          (void)memset(&A6[0], 0,
                       (sizeof(double)) << ((unsigned int)((unsigned char)4)));
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
            V[k] = 12.0 * A2[k];
          }
          exptj = 120.0;
        } else if (b_i == ((int)((signed char)5))) {
          for (k = 0; k < 16; k++) {
            F[k] = A4[k] + (420.0 * A2[k]);
          }
          F[0] += 15120.0;
          F[5] += 15120.0;
          F[10] += 15120.0;
          F[15] += 15120.0;
          (void)memset(&A6[0], 0,
                       (sizeof(double)) << ((unsigned int)((unsigned char)4)));
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
            V[k] = (30.0 * A4[k]) + (3360.0 * A2[k]);
          }
          exptj = 30240.0;
        } else if (b_i == ((int)((signed char)7))) {
          for (k = 0; k < 16; k++) {
            F[k] = (b_A6[k] + (1512.0 * A4[k])) + (277200.0 * A2[k]);
          }
          F[0] += 8.64864E+6;
          F[5] += 8.64864E+6;
          F[10] += 8.64864E+6;
          F[15] += 8.64864E+6;
          (void)memset(&A6[0], 0,
                       (sizeof(double)) << ((unsigned int)((unsigned char)4)));
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
            V[k] =
                ((56.0 * b_A6[k]) + (25200.0 * A4[k])) + (1.99584E+6 * A2[k]);
          }
          exptj = 1.729728E+7;
        } else if (b_i == ((int)((signed char)9))) {
          (void)memset(&V[0], 0,
                       (sizeof(double)) << ((unsigned int)((unsigned char)4)));
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
            F[k] = ((V[k] + (3960.0 * b_A6[k])) + (2.16216E+6 * A4[k])) +
                   (3.027024E+8 * A2[k]);
          }
          F[0] += 8.8216128E+9;
          F[5] += 8.8216128E+9;
          F[10] += 8.8216128E+9;
          F[15] += 8.8216128E+9;
          (void)memset(&A6[0], 0,
                       (sizeof(double)) << ((unsigned int)((unsigned char)4)));
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
            V[k] = (((90.0 * V[k]) + (110880.0 * b_A6[k])) +
                    (3.027024E+7 * A4[k])) +
                   (2.0756736E+9 * A2[k]);
          }
          exptj = 1.76432256E+10;
        } else {
          for (k = 0; k < 16; k++) {
            exptj = b_A6[k];
            y = A4[k];
            b_y = A2[k];
            F[k] = ((3.352212864E+10 * exptj) + (1.05594705216E+13 * y)) +
                   (1.1873537964288E+15 * b_y);
            A6[k] = (exptj + (16380.0 * y)) + (4.08408E+7 * b_y);
          }
          F[0] += 3.238237626624E+16;
          F[5] += 3.238237626624E+16;
          F[10] += 3.238237626624E+16;
          F[15] += 3.238237626624E+16;
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
                       (sizeof(double)) << ((unsigned int)((unsigned char)4)));
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
            A6[k] = ((182.0 * b_A6[k]) + (960960.0 * A4[k])) +
                    (1.32324192E+9 * A2[k]);
          }
          for (k = 0; k < 4; k++) {
            for (b_k = 0; b_k < 4; b_k++) {
              j = k + (4 * b_k);
              V[j] = ((((((b_A6[k] * A6[4 * b_k]) +
                          (b_A6[k + 4] * A6[(4 * b_k) + 1])) +
                         (b_A6[k + 8] * A6[(4 * b_k) + 2])) +
                        (b_A6[k + 12] * A6[(4 * b_k) + 3])) +
                       (6.704425728E+11 * b_A6[j])) +
                      (1.29060195264E+14 * A4[j])) +
                     (7.7717703038976E+15 * A2[j]);
            }
          }
          exptj = 6.476475253248E+16;
        }
        V[0] += exptj;
        V[5] += exptj;
        V[10] += exptj;
        V[15] += exptj;
        for (k = 0; k < 16; k++) {
          exptj = F[k];
          V[k] -= exptj;
          exptj *= 2.0;
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
            if (F[A6_tmp] != 0.0) {
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
            if (exptj != 0.0) {
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
                       (sizeof(double)) << ((unsigned int)((unsigned char)4)));
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
          (void)memcpy(&F[0], &A6[0], 16U * (sizeof(double)));
          if (recomputeDiags) {
            for (k = 0; k < 16; k++) {
              A[k] *= 2.0;
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

static int getExpmParams(const double A[16], double A2[16], double A4[16],
                         double A6[16], double *s)
{
  double dv[16];
  double y[16];
  double b_s;
  double d6;
  double d8;
  double d_s;
  double e_s;
  double eta1;
  double eta3;
  int A2_tmp;
  int b_A2_tmp;
  int b_i;
  int eint;
  int k;
  int m;
  bool exitg1;
  bool guard1;
  bool guard2;
  bool guard3;
  bool guard4;
  b_s = 0.0;
  (void)memset(&A2[0], 0,
               (sizeof(double)) << ((unsigned int)((unsigned char)4)));
  for (k = 0; k < 4; k++) {
    d_s = A2[4 * k];
    m = (4 * k) + 1;
    A2_tmp = (4 * k) + 2;
    b_A2_tmp = (4 * k) + 3;
    for (b_i = 0; b_i < 4; b_i++) {
      e_s = A[b_i + (4 * k)];
      d_s += A[4 * b_i] * e_s;
      A2[m] += A[(4 * b_i) + 1] * e_s;
      A2[A2_tmp] += A[(4 * b_i) + 2] * e_s;
      A2[b_A2_tmp] += A[(4 * b_i) + 3] * e_s;
    }
    A2[4 * k] = d_s;
  }
  (void)memset(&A4[0], 0,
               (sizeof(double)) << ((unsigned int)((unsigned char)4)));
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
               (sizeof(double)) << ((unsigned int)((unsigned char)4)));
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
  d_s = 0.0;
  m = 0;
  exitg1 = false;
  while ((!exitg1) && (m < ((int)((signed char)4)))) {
    e_s = ((fabs(A6[4 * m]) + fabs(A6[(4 * m) + 1])) + fabs(A6[(4 * m) + 2])) +
          fabs(A6[(4 * m) + 3]);
    if (rtIsNaN(e_s)) {
      d_s = rtNaN;
      exitg1 = true;
    } else {
      if (e_s > d_s) {
        d_s = e_s;
      }
      m++;
    }
  }
  d6 = rt_powd_snf(d_s, 0.16666666666666666);
  e_s = 0.0;
  m = 0;
  exitg1 = false;
  while ((!exitg1) && (m < ((int)((signed char)4)))) {
    d_s = ((fabs(A4[4 * m]) + fabs(A4[(4 * m) + 1])) + fabs(A4[(4 * m) + 2])) +
          fabs(A4[(4 * m) + 3]);
    if (rtIsNaN(d_s)) {
      e_s = rtNaN;
      exitg1 = true;
    } else {
      if (d_s > e_s) {
        e_s = d_s;
      }
      m++;
    }
  }
  eta1 = fmax(rt_powd_snf(e_s, 0.25), d6);
  guard1 = false;
  guard2 = false;
  guard3 = false;
  guard4 = false;
  if (eta1 <= 0.01495585217958292) {
    for (k = 0; k < 16; k++) {
      dv[k] = 0.19285012468241128 * fabs(A[k]);
    }
    mpower(dv, 7.0, y);
    eta3 = 0.0;
    m = 0;
    exitg1 = false;
    while ((!exitg1) && (m < ((int)((signed char)4)))) {
      e_s = ((fabs(y[4 * m]) + fabs(y[(4 * m) + 1])) + fabs(y[(4 * m) + 2])) +
            fabs(y[(4 * m) + 3]);
      if (rtIsNaN(e_s)) {
        eta3 = rtNaN;
        exitg1 = true;
      } else {
        if (e_s > eta3) {
          eta3 = e_s;
        }
        m++;
      }
    }
    e_s = 0.0;
    m = 0;
    exitg1 = false;
    while ((!exitg1) && (m < ((int)((signed char)4)))) {
      d_s = ((fabs(A[4 * m]) + fabs(A[(4 * m) + 1])) + fabs(A[(4 * m) + 2])) +
            fabs(A[(4 * m) + 3]);
      if (rtIsNaN(d_s)) {
        e_s = rtNaN;
        exitg1 = true;
      } else {
        if (d_s > e_s) {
          e_s = d_s;
        }
        m++;
      }
    }
    if (fmax(ceil(b_log2((2.0 * (eta3 / e_s)) / 2.2204460492503131E-16) / 6.0),
             0.0) == 0.0) {
      m = 3;
    } else {
      guard4 = true;
    }
  } else {
    guard4 = true;
  }
  if (guard4) {
    if (eta1 <= 0.253939833006323) {
      for (k = 0; k < 16; k++) {
        dv[k] = 0.12321872304378752 * fabs(A[k]);
      }
      mpower(dv, 11.0, y);
      eta1 = 0.0;
      m = 0;
      exitg1 = false;
      while ((!exitg1) && (m < ((int)((signed char)4)))) {
        e_s = ((fabs(y[4 * m]) + fabs(y[(4 * m) + 1])) + fabs(y[(4 * m) + 2])) +
              fabs(y[(4 * m) + 3]);
        if (rtIsNaN(e_s)) {
          eta1 = rtNaN;
          exitg1 = true;
        } else {
          if (e_s > eta1) {
            eta1 = e_s;
          }
          m++;
        }
      }
      e_s = 0.0;
      m = 0;
      exitg1 = false;
      while ((!exitg1) && (m < ((int)((signed char)4)))) {
        d_s = ((fabs(A[4 * m]) + fabs(A[(4 * m) + 1])) + fabs(A[(4 * m) + 2])) +
              fabs(A[(4 * m) + 3]);
        if (rtIsNaN(d_s)) {
          e_s = rtNaN;
          exitg1 = true;
        } else {
          if (d_s > e_s) {
            e_s = d_s;
          }
          m++;
        }
      }
      if (fmax(ceil(b_log2((2.0 * (eta1 / e_s)) / 2.2204460492503131E-16) /
                    10.0),
               0.0) == 0.0) {
        m = 5;
      } else {
        guard3 = true;
      }
    } else {
      guard3 = true;
    }
  }
  if (guard3) {
    mpower(A4, 2.0, y);
    e_s = 0.0;
    m = 0;
    exitg1 = false;
    while ((!exitg1) && (m < ((int)((signed char)4)))) {
      d_s = ((fabs(y[4 * m]) + fabs(y[(4 * m) + 1])) + fabs(y[(4 * m) + 2])) +
            fabs(y[(4 * m) + 3]);
      if (rtIsNaN(d_s)) {
        e_s = rtNaN;
        exitg1 = true;
      } else {
        if (d_s > e_s) {
          e_s = d_s;
        }
        m++;
      }
    }
    d8 = rt_powd_snf(e_s, 0.125);
    eta3 = fmax(d6, d8);
    if (eta3 <= 0.95041789961629319) {
      for (k = 0; k < 16; k++) {
        dv[k] = 0.090475336558796943 * fabs(A[k]);
      }
      mpower(dv, 15.0, y);
      eta1 = 0.0;
      m = 0;
      exitg1 = false;
      while ((!exitg1) && (m < ((int)((signed char)4)))) {
        e_s = ((fabs(y[4 * m]) + fabs(y[(4 * m) + 1])) + fabs(y[(4 * m) + 2])) +
              fabs(y[(4 * m) + 3]);
        if (rtIsNaN(e_s)) {
          eta1 = rtNaN;
          exitg1 = true;
        } else {
          if (e_s > eta1) {
            eta1 = e_s;
          }
          m++;
        }
      }
      e_s = 0.0;
      m = 0;
      exitg1 = false;
      while ((!exitg1) && (m < ((int)((signed char)4)))) {
        d_s = ((fabs(A[4 * m]) + fabs(A[(4 * m) + 1])) + fabs(A[(4 * m) + 2])) +
              fabs(A[(4 * m) + 3]);
        if (rtIsNaN(d_s)) {
          e_s = rtNaN;
          exitg1 = true;
        } else {
          if (d_s > e_s) {
            e_s = d_s;
          }
          m++;
        }
      }
      if (fmax(ceil(b_log2((2.0 * (eta1 / e_s)) / 2.2204460492503131E-16) /
                    14.0),
               0.0) == 0.0) {
        m = 7;
      } else {
        guard2 = true;
      }
    } else {
      guard2 = true;
    }
  }
  if (guard2) {
    if (eta3 <= 2.097847961257068) {
      for (k = 0; k < 16; k++) {
        dv[k] = 0.071467735648795785 * fabs(A[k]);
      }
      mpower(dv, 19.0, y);
      eta1 = 0.0;
      m = 0;
      exitg1 = false;
      while ((!exitg1) && (m < ((int)((signed char)4)))) {
        e_s = ((fabs(y[4 * m]) + fabs(y[(4 * m) + 1])) + fabs(y[(4 * m) + 2])) +
              fabs(y[(4 * m) + 3]);
        if (rtIsNaN(e_s)) {
          eta1 = rtNaN;
          exitg1 = true;
        } else {
          if (e_s > eta1) {
            eta1 = e_s;
          }
          m++;
        }
      }
      e_s = 0.0;
      m = 0;
      exitg1 = false;
      while ((!exitg1) && (m < ((int)((signed char)4)))) {
        d_s = ((fabs(A[4 * m]) + fabs(A[(4 * m) + 1])) + fabs(A[(4 * m) + 2])) +
              fabs(A[(4 * m) + 3]);
        if (rtIsNaN(d_s)) {
          e_s = rtNaN;
          exitg1 = true;
        } else {
          if (d_s > e_s) {
            e_s = d_s;
          }
          m++;
        }
      }
      if (fmax(ceil(b_log2((2.0 * (eta1 / e_s)) / 2.2204460492503131E-16) /
                    18.0),
               0.0) == 0.0) {
        m = 9;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
  }
  if (guard1) {
    double b_T[16];
    (void)memset(&y[0], 0,
                 (sizeof(double)) << ((unsigned int)((unsigned char)4)));
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
    e_s = 0.0;
    m = 0;
    exitg1 = false;
    while ((!exitg1) && (m < ((int)((signed char)4)))) {
      d_s = ((fabs(y[4 * m]) + fabs(y[(4 * m) + 1])) + fabs(y[(4 * m) + 2])) +
            fabs(y[(4 * m) + 3]);
      if (rtIsNaN(d_s)) {
        e_s = rtNaN;
        exitg1 = true;
      } else {
        if (d_s > e_s) {
          e_s = d_s;
        }
        m++;
      }
    }
    b_s = fmax(ceil(b_log2(fmin(eta3, fmax(d8, rt_powd_snf(e_s, 0.1))) /
                           5.3719203511481517)),
               0.0);
    d_s = rt_powd_snf(2.0, b_s);
    for (k = 0; k < 16; k++) {
      e_s = A[k] / d_s;
      b_T[k] = e_s;
      dv[k] = 0.05031554467093536 * fabs(e_s);
    }
    mpower(dv, 27.0, y);
    eta1 = 0.0;
    m = 0;
    exitg1 = false;
    while ((!exitg1) && (m < ((int)((signed char)4)))) {
      d_s = ((fabs(y[4 * m]) + fabs(y[(4 * m) + 1])) + fabs(y[(4 * m) + 2])) +
            fabs(y[(4 * m) + 3]);
      if (rtIsNaN(d_s)) {
        eta1 = rtNaN;
        exitg1 = true;
      } else {
        if (d_s > eta1) {
          eta1 = d_s;
        }
        m++;
      }
    }
    e_s = 0.0;
    m = 0;
    exitg1 = false;
    while ((!exitg1) && (m < ((int)((signed char)4)))) {
      d_s = ((fabs(b_T[4 * m]) + fabs(b_T[(4 * m) + 1])) +
             fabs(b_T[(4 * m) + 2])) +
            fabs(b_T[(4 * m) + 3]);
      if (rtIsNaN(d_s)) {
        e_s = rtNaN;
        exitg1 = true;
      } else {
        if (d_s > e_s) {
          e_s = d_s;
        }
        m++;
      }
    }
    b_s +=
        fmax(ceil(b_log2((2.0 * (eta1 / e_s)) / 2.2204460492503131E-16) / 26.0),
             0.0);
    if (rtIsInf(b_s)) {
      d_s = 0.0;
      m = 0;
      exitg1 = false;
      while ((!exitg1) && (m < ((int)((signed char)4)))) {
        e_s = ((fabs(A[4 * m]) + fabs(A[(4 * m) + 1])) + fabs(A[(4 * m) + 2])) +
              fabs(A[(4 * m) + 3]);
        if (rtIsNaN(e_s)) {
          d_s = rtNaN;
          exitg1 = true;
        } else {
          if (e_s > d_s) {
            d_s = e_s;
          }
          m++;
        }
      }
      d_s /= 5.3719203511481517;
      if ((!rtIsInf(d_s)) && (!rtIsNaN(d_s))) {
        d_s = frexp(d_s, &eint);
      } else {
        eint = 0;
      }
      b_s = (double)eint;
      if (d_s == 0.5) {
        b_s = ((double)eint) - 1.0;
      }
    }
    m = 13;
  }
  *s = b_s;
  return m;
}

static void inv(const double x[16], double y[16])
{
  double b_x[16];
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
    y[k] = 0.0;
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
    y[k + pipk] = 1.0;
    for (j = k + 1; j < 5; j++) {
      kAcol = (j + pipk) - 1;
      if (y[kAcol] != 0.0) {
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
      double d;
      kAcol = 4 * j;
      b_i = j + pipk;
      d = y[b_i];
      if (d != 0.0) {
        y[b_i] = d / b_x[j + kAcol];
        for (c_i = 0; c_i < j; c_i++) {
          y_tmp = c_i + pipk;
          y[y_tmp] -= y[b_i] * b_x[c_i + kAcol];
        }
      }
    }
  }
}

static void mpower(const double b_a[16], double b, double c[16])
{
  double aBuffer[16];
  double cBuffer[16];
  double c_a[16];
  int b_i;
  int b_k;
  int k;
  if (floor(b) == b) {
    double e;
    e = fabs(b);
    if (e <= 2.147483647E+9) {
      int b_n;
      int n;
      int nb;
      int nbitson;
      (void)memcpy(&c_a[0], &b_a[0], 16U * (sizeof(double)));
      n = (int)e;
      b_n = n;
      nbitson = 0;
      nb = -1;
      while (b_n > ((int)((signed char)0))) {
        nb++;
        if ((((unsigned int)b_n) & ((unsigned int)1U)) != ((unsigned int)0U)) {
          nbitson++;
        }
        b_n = asr_s32(b_n, 1U);
      }
      if (e <= 2.0) {
        if (b == 2.0) {
          (void)memset(&c[0], 0,
                       (sizeof(double)) << ((unsigned int)((unsigned char)4)));
          for (k = 0; k < 4; k++) {
            double d;
            int c_tmp;
            d = c[4 * k];
            b_n = (4 * k) + 1;
            nbitson = (4 * k) + 2;
            c_tmp = (4 * k) + 3;
            for (b_i = 0; b_i < 4; b_i++) {
              double d1;
              d1 = b_a[b_i + (4 * k)];
              d += b_a[4 * b_i] * d1;
              c[b_n] += b_a[(4 * b_i) + 1] * d1;
              c[nbitson] += b_a[(4 * b_i) + 2] * d1;
              c[c_tmp] += b_a[(4 * b_i) + 3] * d1;
            }
            c[4 * k] = d;
          }
        } else if (b == 1.0) {
          (void)memcpy(&c[0], &b_a[0], 16U * (sizeof(double)));
        } else if (b == -1.0) {
          inv(b_a, c);
        } else if (b == -2.0) {
          (void)memset(&c_a[0], 0,
                       (sizeof(double)) << ((unsigned int)((unsigned char)4)));
          for (k = 0; k < 4; k++) {
            double d;
            int c_tmp;
            d = c_a[4 * k];
            b_n = (4 * k) + 1;
            nbitson = (4 * k) + 2;
            c_tmp = (4 * k) + 3;
            for (b_i = 0; b_i < 4; b_i++) {
              double d1;
              d1 = b_a[b_i + (4 * k)];
              d += b_a[4 * b_i] * d1;
              c_a[b_n] += b_a[(4 * b_i) + 1] * d1;
              c_a[nbitson] += b_a[(4 * b_i) + 2] * d1;
              c_a[c_tmp] += b_a[(4 * b_i) + 3] * d1;
            }
            c_a[4 * k] = d;
          }
          inv(c_a, c);
        } else {
          bool lsb;
          lsb = false;
          for (k = 0; k < 16; k++) {
            if (lsb || (rtIsNaN(b_a[k]))) {
              lsb = true;
            }
          }
          if (lsb) {
            for (k = 0; k < 16; k++) {
              c[k] = rtNaN;
            }
          } else {
            (void)memset(&c[0], 0, 16U * (sizeof(double)));
            c[0] = 1.0;
            c[5] = 1.0;
            c[10] = 1.0;
            c[15] = 1.0;
          }
        }
      } else {
        double d;
        double d1;
        int c_tmp;
        bool aBufferInUse;
        bool first;
        bool lsb;
        first = true;
        aBufferInUse = false;
        lsb = ((((unsigned int)nbitson) & ((unsigned int)1U)) !=
               ((unsigned int)0U));
        if ((lsb && (b < 0.0)) || ((!lsb) && (b >= 0.0))) {
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
                  (void)memcpy(&cBuffer[0], &aBuffer[0],
                               16U * (sizeof(double)));
                } else {
                  (void)memcpy(&cBuffer[0], &c_a[0], 16U * (sizeof(double)));
                }
              } else if (aBufferInUse) {
                (void)memcpy(&c[0], &aBuffer[0], 16U * (sizeof(double)));
              } else {
                (void)memcpy(&c[0], &c_a[0], 16U * (sizeof(double)));
              }
            } else {
              if (aBufferInUse) {
                if (lsb) {
                  (void)memset(&c[0], 0,
                               (sizeof(double))
                                   << ((unsigned int)((unsigned char)4)));
                  for (k = 0; k < 4; k++) {
                    d = c[4 * k];
                    b_n = (4 * k) + 1;
                    nbitson = (4 * k) + 2;
                    c_tmp = (4 * k) + 3;
                    for (b_i = 0; b_i < 4; b_i++) {
                      d1 = aBuffer[b_i + (4 * k)];
                      d += cBuffer[4 * b_i] * d1;
                      c[b_n] += cBuffer[(4 * b_i) + 1] * d1;
                      c[nbitson] += cBuffer[(4 * b_i) + 2] * d1;
                      c[c_tmp] += cBuffer[(4 * b_i) + 3] * d1;
                    }
                    c[4 * k] = d;
                  }
                } else {
                  (void)memset(&cBuffer[0], 0,
                               (sizeof(double))
                                   << ((unsigned int)((unsigned char)4)));
                  for (k = 0; k < 4; k++) {
                    d = cBuffer[4 * k];
                    b_n = (4 * k) + 1;
                    nbitson = (4 * k) + 2;
                    c_tmp = (4 * k) + 3;
                    for (b_i = 0; b_i < 4; b_i++) {
                      d1 = aBuffer[b_i + (4 * k)];
                      d += c[4 * b_i] * d1;
                      cBuffer[b_n] += c[(4 * b_i) + 1] * d1;
                      cBuffer[nbitson] += c[(4 * b_i) + 2] * d1;
                      cBuffer[c_tmp] += c[(4 * b_i) + 3] * d1;
                    }
                    cBuffer[4 * k] = d;
                  }
                }
              } else if (lsb) {
                (void)memset(&c[0], 0,
                             (sizeof(double))
                                 << ((unsigned int)((unsigned char)4)));
                for (k = 0; k < 4; k++) {
                  d = c[4 * k];
                  b_n = (4 * k) + 1;
                  nbitson = (4 * k) + 2;
                  c_tmp = (4 * k) + 3;
                  for (b_i = 0; b_i < 4; b_i++) {
                    d1 = c_a[b_i + (4 * k)];
                    d += cBuffer[4 * b_i] * d1;
                    c[b_n] += cBuffer[(4 * b_i) + 1] * d1;
                    c[nbitson] += cBuffer[(4 * b_i) + 2] * d1;
                    c[c_tmp] += cBuffer[(4 * b_i) + 3] * d1;
                  }
                  c[4 * k] = d;
                }
              } else {
                (void)memset(&cBuffer[0], 0,
                             (sizeof(double))
                                 << ((unsigned int)((unsigned char)4)));
                for (k = 0; k < 4; k++) {
                  d = cBuffer[4 * k];
                  b_n = (4 * k) + 1;
                  nbitson = (4 * k) + 2;
                  c_tmp = (4 * k) + 3;
                  for (b_i = 0; b_i < 4; b_i++) {
                    d1 = c_a[b_i + (4 * k)];
                    d += c[4 * b_i] * d1;
                    cBuffer[b_n] += c[(4 * b_i) + 1] * d1;
                    cBuffer[nbitson] += c[(4 * b_i) + 2] * d1;
                    cBuffer[c_tmp] += c[(4 * b_i) + 3] * d1;
                  }
                  cBuffer[4 * k] = d;
                }
              }
              lsb = !lsb;
            }
          }
          n = asr_s32(n, 1U);
          if (aBufferInUse) {
            (void)memset(&c_a[0], 0,
                         (sizeof(double))
                             << ((unsigned int)((unsigned char)4)));
            for (k = 0; k < 4; k++) {
              d = c_a[4 * k];
              b_n = (4 * k) + 1;
              nbitson = (4 * k) + 2;
              c_tmp = (4 * k) + 3;
              for (b_i = 0; b_i < 4; b_i++) {
                d1 = aBuffer[b_i + (4 * k)];
                d += aBuffer[4 * b_i] * d1;
                c_a[b_n] += aBuffer[(4 * b_i) + 1] * d1;
                c_a[nbitson] += aBuffer[(4 * b_i) + 2] * d1;
                c_a[c_tmp] += aBuffer[(4 * b_i) + 3] * d1;
              }
              c_a[4 * k] = d;
            }
          } else {
            (void)memset(&aBuffer[0], 0,
                         (sizeof(double))
                             << ((unsigned int)((unsigned char)4)));
            for (k = 0; k < 4; k++) {
              d = aBuffer[4 * k];
              b_n = (4 * k) + 1;
              nbitson = (4 * k) + 2;
              c_tmp = (4 * k) + 3;
              for (b_i = 0; b_i < 4; b_i++) {
                d1 = c_a[b_i + (4 * k)];
                d += c_a[4 * b_i] * d1;
                aBuffer[b_n] += c_a[(4 * b_i) + 1] * d1;
                aBuffer[nbitson] += c_a[(4 * b_i) + 2] * d1;
                aBuffer[c_tmp] += c_a[(4 * b_i) + 3] * d1;
              }
              aBuffer[4 * k] = d;
            }
          }
          aBufferInUse = !aBufferInUse;
        }
        if (first) {
          if (b < 0.0) {
            if (aBufferInUse) {
              inv(aBuffer, c);
            } else {
              inv(c_a, c);
            }
          } else if (aBufferInUse) {
            (void)memcpy(&c[0], &aBuffer[0], 16U * (sizeof(double)));
          } else {
            (void)memcpy(&c[0], &c_a[0], 16U * (sizeof(double)));
          }
        } else if (b < 0.0) {
          if (aBufferInUse) {
            (void)memset(&cBuffer[0], 0,
                         (sizeof(double))
                             << ((unsigned int)((unsigned char)4)));
            for (k = 0; k < 4; k++) {
              d = cBuffer[4 * k];
              b_n = (4 * k) + 1;
              nbitson = (4 * k) + 2;
              c_tmp = (4 * k) + 3;
              for (b_i = 0; b_i < 4; b_i++) {
                d1 = aBuffer[b_i + (4 * k)];
                d += c[4 * b_i] * d1;
                cBuffer[b_n] += c[(4 * b_i) + 1] * d1;
                cBuffer[nbitson] += c[(4 * b_i) + 2] * d1;
                cBuffer[c_tmp] += c[(4 * b_i) + 3] * d1;
              }
              cBuffer[4 * k] = d;
            }
          } else {
            (void)memset(&cBuffer[0], 0,
                         (sizeof(double))
                             << ((unsigned int)((unsigned char)4)));
            for (k = 0; k < 4; k++) {
              d = cBuffer[4 * k];
              b_n = (4 * k) + 1;
              nbitson = (4 * k) + 2;
              c_tmp = (4 * k) + 3;
              for (b_i = 0; b_i < 4; b_i++) {
                d1 = c_a[b_i + (4 * k)];
                d += c[4 * b_i] * d1;
                cBuffer[b_n] += c[(4 * b_i) + 1] * d1;
                cBuffer[nbitson] += c[(4 * b_i) + 2] * d1;
                cBuffer[c_tmp] += c[(4 * b_i) + 3] * d1;
              }
              cBuffer[4 * k] = d;
            }
          }
          inv(cBuffer, c);
        } else if (aBufferInUse) {
          (void)memset(&c[0], 0,
                       (sizeof(double)) << ((unsigned int)((unsigned char)4)));
          for (k = 0; k < 4; k++) {
            d = c[4 * k];
            b_n = (4 * k) + 1;
            nbitson = (4 * k) + 2;
            c_tmp = (4 * k) + 3;
            for (b_i = 0; b_i < 4; b_i++) {
              d1 = aBuffer[b_i + (4 * k)];
              d += cBuffer[4 * b_i] * d1;
              c[b_n] += cBuffer[(4 * b_i) + 1] * d1;
              c[nbitson] += cBuffer[(4 * b_i) + 2] * d1;
              c[c_tmp] += cBuffer[(4 * b_i) + 3] * d1;
            }
            c[4 * k] = d;
          }
        } else {
          (void)memset(&c[0], 0,
                       (sizeof(double)) << ((unsigned int)((unsigned char)4)));
          for (k = 0; k < 4; k++) {
            d = c[4 * k];
            b_n = (4 * k) + 1;
            nbitson = (4 * k) + 2;
            c_tmp = (4 * k) + 3;
            for (b_i = 0; b_i < 4; b_i++) {
              d1 = c_a[b_i + (4 * k)];
              d += cBuffer[4 * b_i] * d1;
              c[b_n] += cBuffer[(4 * b_i) + 1] * d1;
              c[nbitson] += cBuffer[(4 * b_i) + 2] * d1;
              c[c_tmp] += cBuffer[(4 * b_i) + 3] * d1;
            }
            c[4 * k] = d;
          }
        }
      }
    } else {
      (void)memcpy(&c_a[0], &b_a[0], 16U * (sizeof(double)));
      if (!rtIsInf(b)) {
        bool lsb;
        lsb = true;
        double ed2;
        int exitg1;
        do {
          double d;
          double d1;
          int b_n;
          int c_tmp;
          int nbitson;
          exitg1 = 0;
          ed2 = floor(e / 2.0);
          if ((2.0 * ed2) != e) {
            if (lsb) {
              (void)memcpy(&c[0], &c_a[0], 16U * (sizeof(double)));
              lsb = false;
            } else {
              (void)memset(&cBuffer[0], 0,
                           (sizeof(double))
                               << ((unsigned int)((unsigned char)4)));
              for (k = 0; k < 4; k++) {
                d = cBuffer[4 * k];
                b_n = (4 * k) + 1;
                nbitson = (4 * k) + 2;
                c_tmp = (4 * k) + 3;
                for (b_i = 0; b_i < 4; b_i++) {
                  d1 = c_a[b_i + (4 * k)];
                  d += c[4 * b_i] * d1;
                  cBuffer[b_n] += c[(4 * b_i) + 1] * d1;
                  cBuffer[nbitson] += c[(4 * b_i) + 2] * d1;
                  cBuffer[c_tmp] += c[(4 * b_i) + 3] * d1;
                }
                cBuffer[4 * k] = d;
              }
              (void)memcpy(&c[0], &cBuffer[0], 16U * (sizeof(double)));
            }
          }
          if (ed2 == 0.0) {
            exitg1 = 1;
          } else {
            e = ed2;
            (void)memset(&cBuffer[0], 0,
                         (sizeof(double))
                             << ((unsigned int)((unsigned char)4)));
            for (k = 0; k < 4; k++) {
              d = cBuffer[4 * k];
              b_n = (4 * k) + 1;
              nbitson = (4 * k) + 2;
              c_tmp = (4 * k) + 3;
              for (b_i = 0; b_i < 4; b_i++) {
                d1 = c_a[b_i + (4 * k)];
                d += c_a[4 * b_i] * d1;
                cBuffer[b_n] += c_a[(4 * b_i) + 1] * d1;
                cBuffer[nbitson] += c_a[(4 * b_i) + 2] * d1;
                cBuffer[c_tmp] += c_a[(4 * b_i) + 3] * d1;
              }
              cBuffer[4 * k] = d;
            }
            (void)memcpy(&c_a[0], &cBuffer[0], 16U * (sizeof(double)));
          }
        } while (exitg1 == ((int)((signed char)0)));
        if (b < 0.0) {
          (void)memcpy(&c_a[0], &c[0], 16U * (sizeof(double)));
          inv(c_a, c);
        }
      } else {
        for (k = 0; k < 16; k++) {
          c[k] = rtNaN;
        }
      }
    }
  }
}

static void mrdiv(const double A[30], const double b_B[9], double d_Y[30])
{
  double d_A[9];
  double a21;
  double maxval;
  int k;
  int r1;
  int r2;
  int r3;
  int rtemp;
  (void)memcpy(&d_A[0], &b_B[0], 9U * (sizeof(double)));
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = fabs(b_B[0]);
  a21 = fabs(b_B[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }
  if (fabs(b_B[2]) > maxval) {
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
  if (fabs(d_A[r3 + 3]) > fabs(d_A[r2 + 3])) {
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

static void recomputeBlockDiag(const double A[16], double F[16],
                               const int blockFormat[3])
{
  double avg;
  double expa11;
  double expa22;
  double x12;
  if (blockFormat[0] != ((int)((signed char)0))) {
    if (blockFormat[0] == ((int)((signed char)1))) {
      expa11 = exp(A[0]);
      expa22 = exp(A[5]);
      avg = (A[0] + A[5]) / 2.0;
      if (fmax(avg, fabs(A[0] - A[5]) / 2.0) < 709.782712893384) {
        x12 = (A[5] - A[0]) / 2.0;
        if (x12 == 0.0) {
          x12 = 1.0;
        } else {
          x12 = sinh(x12) / x12;
        }
        x12 *= A[4] * exp(avg);
      } else {
        x12 = (A[4] * (expa22 - expa11)) / (A[5] - A[0]);
      }
      F[0] = expa11;
      F[4] = x12;
      F[5] = expa22;
    } else if (blockFormat[0] == ((int)((signed char)2))) {
      x12 = sqrt(fabs(A[1] * A[4]));
      avg = exp(A[0]);
      if (x12 == 0.0) {
        expa11 = 1.0;
      } else {
        expa11 = sin(x12) / x12;
      }
      F[0] = avg * cos(x12);
      F[1] = (avg * A[1]) * expa11;
      F[4] = (avg * A[4]) * expa11;
      F[5] = F[0];
    } else {
      /* no actions */
    }
  }
  if (blockFormat[1] != ((int)((signed char)0))) {
    if (blockFormat[1] == ((int)((signed char)1))) {
      expa11 = exp(A[5]);
      expa22 = exp(A[10]);
      avg = (A[5] + A[10]) / 2.0;
      if (fmax(avg, fabs(A[5] - A[10]) / 2.0) < 709.782712893384) {
        x12 = (A[10] - A[5]) / 2.0;
        if (x12 == 0.0) {
          x12 = 1.0;
        } else {
          x12 = sinh(x12) / x12;
        }
        x12 *= A[9] * exp(avg);
      } else {
        x12 = (A[9] * (expa22 - expa11)) / (A[10] - A[5]);
      }
      F[5] = expa11;
      F[9] = x12;
      F[10] = expa22;
    } else if (blockFormat[1] == ((int)((signed char)2))) {
      x12 = sqrt(fabs(A[6] * A[9]));
      avg = exp(A[5]);
      if (x12 == 0.0) {
        expa11 = 1.0;
      } else {
        expa11 = sin(x12) / x12;
      }
      F[5] = avg * cos(x12);
      F[6] = (avg * A[6]) * expa11;
      F[9] = (avg * A[9]) * expa11;
      F[10] = F[5];
    } else {
      /* no actions */
    }
  }
  if (blockFormat[2] != ((int)((signed char)0))) {
    if (blockFormat[2] == ((int)((signed char)1))) {
      expa11 = exp(A[10]);
      expa22 = exp(A[15]);
      avg = (A[10] + A[15]) / 2.0;
      if (fmax(avg, fabs(A[10] - A[15]) / 2.0) < 709.782712893384) {
        x12 = (A[15] - A[10]) / 2.0;
        if (x12 == 0.0) {
          x12 = 1.0;
        } else {
          x12 = sinh(x12) / x12;
        }
        x12 *= A[14] * exp(avg);
      } else {
        x12 = (A[14] * (expa22 - expa11)) / (A[15] - A[10]);
      }
      F[10] = expa11;
      F[14] = x12;
      F[15] = expa22;
    } else if (blockFormat[2] == ((int)((signed char)2))) {
      x12 = sqrt(fabs(A[11] * A[14]));
      avg = exp(A[10]);
      if (x12 == 0.0) {
        expa11 = 1.0;
      } else {
        expa11 = sin(x12) / x12;
      }
      F[10] = avg * cos(x12);
      F[11] = (avg * A[11]) * expa11;
      F[14] = (avg * A[14]) * expa11;
      F[15] = F[10];
    } else {
      /* no actions */
    }
  }
  if (blockFormat[2] == ((int)((signed char)0))) {
    F[15] = exp(A[15]);
  }
}

static void rotateRight(int n, double z[16], int iz0, const double cs[6],
                        int ic0, int is0)
{
  int b_i;
  int j;
  b_i = n - 1;
  for (j = b_i; j >= 1; j--) {
    double ctemp;
    double stemp;
    int offsetj;
    int offsetjp1;
    ctemp = cs[(ic0 + j) - 2];
    stemp = cs[(is0 + j) - 2];
    offsetj = (((j - 1) * 4) + iz0) - 2;
    offsetjp1 = ((j * 4) + iz0) - 2;
    if ((ctemp != 1.0) || (stemp != 0.0)) {
      double d;
      double temp;
      temp = z[offsetjp1 + 1];
      d = z[offsetj + 1];
      z[offsetjp1 + 1] = (ctemp * temp) - (stemp * d);
      d = (stemp * temp) + (ctemp * d);
      z[offsetj + 1] = d;
      temp = z[offsetjp1 + 2];
      d = z[offsetj + 2];
      z[offsetjp1 + 2] = (ctemp * temp) - (stemp * d);
      d = (stemp * temp) + (ctemp * d);
      z[offsetj + 2] = d;
      temp = z[offsetjp1 + 3];
      d = z[offsetj + 3];
      z[offsetjp1 + 3] = (ctemp * temp) - (stemp * d);
      d = (stemp * temp) + (ctemp * d);
      z[offsetj + 3] = d;
      temp = z[offsetjp1 + 4];
      d = z[offsetj + 4];
      z[offsetjp1 + 4] = (ctemp * temp) - (stemp * d);
      d = (stemp * temp) + (ctemp * d);
      z[offsetj + 4] = d;
    }
  }
}

static double rt_powd_snf(double u0, double u1)
{
  double y;
  if ((rtIsNaN(u0)) || (rtIsNaN(u1))) {
    y = rtNaN;
  } else {
    double d;
    y = fabs(u0);
    d = fabs(u1);
    if (rtIsInf(u1)) {
      if (y == 1.0) {
        y = 1.0;
      } else if (y > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d == 0.0) {
      y = 1.0;
    } else if (d == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }
  return y;
}

static double xdlaev2(double b_a, double b, double c, double *rt2, double *cs1,
                      double *sn1)
{
  double ab;
  double acmn;
  double acmx;
  double adf;
  double df;
  double rt1;
  double sm;
  double tb;
  int sgn1;
  int sgn2;
  sm = b_a + c;
  df = b_a - c;
  adf = fabs(df);
  tb = b + b;
  ab = fabs(tb);
  if (fabs(b_a) > fabs(c)) {
    acmx = b_a;
    acmn = c;
  } else {
    acmx = c;
    acmn = b_a;
  }
  if (adf > ab) {
    rt1 = ab / adf;
    adf *= sqrt((rt1 * rt1) + 1.0);
  } else if (adf < ab) {
    adf /= ab;
    adf = ab * sqrt((adf * adf) + 1.0);
  } else {
    adf = ab * 1.4142135623730951;
  }
  if (sm < 0.0) {
    rt1 = 0.5 * (sm - adf);
    sgn1 = -1;
    *rt2 = ((acmx / rt1) * acmn) - ((b / rt1) * b);
  } else if (sm > 0.0) {
    rt1 = 0.5 * (sm + adf);
    sgn1 = 1;
    *rt2 = ((acmx / rt1) * acmn) - ((b / rt1) * b);
  } else {
    rt1 = 0.5 * adf;
    *rt2 = -0.5 * adf;
    sgn1 = 1;
  }
  if (df >= 0.0) {
    adf += df;
    sgn2 = 1;
  } else {
    adf = df - adf;
    sgn2 = -1;
  }
  if (fabs(adf) > ab) {
    adf = (-tb) / adf;
    *sn1 = 1.0 / sqrt((adf * adf) + 1.0);
    *cs1 = adf * (*sn1);
  } else if (ab == 0.0) {
    *cs1 = 1.0;
    *sn1 = 0.0;
  } else {
    adf = (-adf) / tb;
    *cs1 = 1.0 / sqrt((adf * adf) + 1.0);
    *sn1 = adf * (*cs1);
  }
  if (sgn1 == sgn2) {
    adf = *cs1;
    *cs1 = -(*sn1);
    *sn1 = adf;
  }
  return rt1;
}

static double xnrm2(int n, const double x[16], int ix0)
{
  double y;
  int k;
  y = 0.0;
  if (n >= ((int)((signed char)1))) {
    if (n == ((int)((signed char)1))) {
      y = fabs(x[ix0 - 1]);
    } else {
      double scale;
      int kend;
      scale = 3.3121686421112381E-170;
      kend = ix0 + n;
      for (k = ix0; k < kend; k++) {
        double absxk;
        absxk = fabs(x[k - 1]);
        if (absxk > scale) {
          double t;
          t = scale / absxk;
          y = ((y * t) * t) + 1.0;
          scale = absxk;
        } else {
          double t;
          t = absxk / scale;
          y += t * t;
        }
      }
      y = scale * sqrt(y);
    }
  }
  return y;
}

static int xsyheev(double A[16], double b_W[4])
{
  double work[4];
  double absx;
  double anrm;
  int b_i;
  int c_i;
  int e_i;
  int exitg1;
  int info;
  int j;
  int offset;
  bool exitg2;
  info = 0;
  anrm = 0.0;
  offset = 0;
  exitg2 = false;
  while ((!exitg2) && (offset < ((int)((signed char)4)))) {
    b_i = 0;
    do {
      exitg1 = 0;
      if (b_i <= offset) {
        absx = fabs(A[b_i + (4 * offset)]);
        if (rtIsNaN(absx)) {
          anrm = rtNaN;
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
  if ((rtIsInf(anrm)) || (rtIsNaN(anrm))) {
    b_W[0] = rtNaN;
    b_W[1] = rtNaN;
    b_W[2] = rtNaN;
    b_W[3] = rtNaN;
    for (j = 0; j < 16; j++) {
      A[j] = rtNaN;
    }
  } else {
    double e[3];
    double tau[3];
    bool guard1;
    bool iscale;
    iscale = false;
    guard1 = false;
    if ((anrm > 0.0) && (anrm < 1.0010415475915505E-146)) {
      iscale = true;
      anrm = 1.0010415475915505E-146 / anrm;
      guard1 = true;
    } else if (anrm > 9.9895953610111751E+145) {
      iscale = true;
      anrm = 9.9895953610111751E+145 / anrm;
      guard1 = true;
    } else {
      /* no actions */
    }
    if (guard1) {
      double cfromc;
      bool notdone;
      absx = anrm;
      cfromc = 1.0;
      notdone = true;
      while (notdone) {
        double cfrom1;
        double cto1;
        double mul;
        cfrom1 = cfromc * 2.0041683600089728E-292;
        cto1 = absx / 4.9896007738368E+291;
        if ((fabs(cfrom1) > absx) && (absx != 0.0)) {
          mul = 2.0041683600089728E-292;
          cfromc = cfrom1;
        } else if (cto1 > fabs(cfromc)) {
          mul = 4.9896007738368E+291;
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
      A[offset] = 0.0;
      b_i = j + 3;
      for (c_i = b_i; c_i < 5; c_i++) {
        A[(c_i + offset) - 1] = A[(c_i + (4 * j)) - 1];
      }
    }
    A[0] = 1.0;
    A[1] = 0.0;
    A[2] = 0.0;
    A[3] = 0.0;
    work[0] = 0.0;
    work[1] = 0.0;
    work[2] = 0.0;
    work[3] = 0.0;
    for (e_i = 2; e_i >= 0; e_i--) {
      int iaii;
      iaii = (e_i + (e_i * 4)) + 10;
      if ((e_i + 1) < ((int)((signed char)3))) {
        int lastv;
        A[iaii - 5] = 1.0;
        if (tau[e_i] != 0.0) {
          lastv = 3 - e_i;
          offset = iaii - e_i;
          while ((lastv > ((int)((signed char)0))) && (A[offset - 3] == 0.0)) {
            lastv--;
            offset--;
          }
          info = 1 - e_i;
          exitg2 = false;
          while ((!exitg2) && ((info + 1) > ((int)((signed char)0)))) {
            offset = iaii + (info * 4);
            b_i = offset;
            do {
              exitg1 = 0;
              if (b_i <= ((offset + lastv) - 1)) {
                if (A[b_i - 1] != 0.0) {
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
                         ((unsigned int)((int)(info + 1))) * (sizeof(double)));
            b_i = iaii + (4 * info);
            for (c_i = iaii; c_i <= b_i; c_i += 4) {
              absx = 0.0;
              offset = c_i + lastv;
              for (j = c_i; j < offset; j++) {
                absx += A[j - 1] * A[((iaii + j) - c_i) - 5];
              }
              offset = asr_s32(c_i - iaii, 2U);
              work[offset] += absx;
            }
          }
          if (!((-tau[e_i]) == 0.0)) {
            offset = iaii;
            for (j = 0; j <= info; j++) {
              absx = work[j];
              if (absx != 0.0) {
                absx *= -tau[e_i];
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
        b_i = (iaii - e_i) - 2;
        for (j = offset; j <= b_i; j++) {
          A[j - 1] *= -tau[e_i];
        }
      }
      A[iaii - 5] = 1.0 - tau[e_i];
      for (j = 0; j < e_i; j++) {
        A[(iaii - j) - 6] = 0.0;
      }
    }
    info = xzsteqr(b_W, e, A);
    if (info != ((int)((signed char)0))) {
      b_W[0] = rtNaN;
      b_W[1] = rtNaN;
      b_W[2] = rtNaN;
      b_W[3] = rtNaN;
      for (j = 0; j < 16; j++) {
        A[j] = rtNaN;
      }
    } else if (iscale) {
      absx = 1.0 / anrm;
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

static int xzgetrf(double A[16], int ipiv[4])
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
    double smax;
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
    smax = fabs(A[jj]);
    for (k = 2; k < jA; k++) {
      double s;
      s = fabs(A[(b + k) - 1]);
      if (s > smax) {
        b_a = k - 1;
        smax = s;
      }
    }
    if (A[jj + b_a] != 0.0) {
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
      if (smax != 0.0) {
        b_a = jA + 6;
        jp1j = (jA - j) + 8;
        for (ijA = b_a; ijA <= jp1j; ijA++) {
          A[ijA - 1] += A[((jj + ijA) - jA) - 5] * (-smax);
        }
      }
      jA += 4;
    }
  }
  if ((info == ((int)((signed char)0))) && (!(A[15] != 0.0))) {
    info = 4;
  }
  return info;
}

static double xzlartg(double b_f, double g, double *sn, double *b_r)
{
  double cs;
  double g1;
  cs = fabs(b_f);
  g1 = fabs(g);
  if (g == 0.0) {
    cs = 1.0;
    *sn = 0.0;
    *b_r = b_f;
  } else if (b_f == 0.0) {
    cs = 0.0;
    if (g >= 0.0) {
      *sn = 1.0;
    } else {
      *sn = -1.0;
    }
    *b_r = g1;
  } else if ((((cs > 1.4916681462400413E-154) &&
               (cs < 4.7403759540545887E+153)) &&
              (g1 > 1.4916681462400413E-154)) &&
             (g1 < 4.7403759540545887E+153)) {
    double d;
    d = sqrt((b_f * b_f) + (g * g));
    cs /= d;
    *b_r = d;
    if (!(b_f >= 0.0)) {
      *b_r = -d;
    }
    *sn = g / (*b_r);
  } else {
    double d;
    double gs;
    g1 = fmin(4.49423283715579E+307,
              fmax(2.2250738585072014E-308, fmax(cs, g1)));
    cs = b_f / g1;
    gs = g / g1;
    d = sqrt((cs * cs) + (gs * gs));
    cs = fabs(cs) / d;
    *b_r = d;
    if (!(b_f >= 0.0)) {
      *b_r = -d;
    }
    *sn = gs / (*b_r);
    *b_r *= g1;
  }
  return cs;
}

static void xzlascl(double cfrom, double cto, int m, double A[4], int iA0)
{
  double cfromc;
  double ctoc;
  int b_i;
  bool notdone;
  cfromc = cfrom;
  ctoc = cto;
  notdone = true;
  while (notdone) {
    double cfrom1;
    double cto1;
    double mul;
    cfrom1 = cfromc * 2.0041683600089728E-292;
    cto1 = ctoc / 4.9896007738368E+291;
    if ((fabs(cfrom1) > fabs(ctoc)) && (ctoc != 0.0)) {
      mul = 2.0041683600089728E-292;
      cfromc = cfrom1;
    } else if (fabs(cto1) > fabs(cfromc)) {
      mul = 4.9896007738368E+291;
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

static int xzsteqr(double d[4], double e[3], double z[16])
{
  double work[6];
  double b_r;
  double g_tmp;
  double s;
  double temp;
  int b_i;
  int b_ii;
  int c_l1;
  int info;
  int jtot;
  info = 0;
  for (b_i = 0; b_i < 6; b_i++) {
    work[b_i] = 0.0;
  }
  jtot = 0;
  c_l1 = 1;
  int exitg1;
  do {
    exitg1 = 0;
    if (c_l1 > ((int)((signed char)4))) {
      for (b_ii = 0; b_ii < 3; b_ii++) {
        double p;
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
      int f_l;
      int lend;
      int lendsv;
      int lsv;
      int m;
      bool exitg2;
      if (c_l1 > ((int)((signed char)1))) {
        e[c_l1 - 2] = 0.0;
      }
      m = c_l1;
      exitg2 = false;
      while ((!exitg2) && (m < ((int)((signed char)4)))) {
        temp = fabs(e[m - 1]);
        if (temp == 0.0) {
          exitg2 = true;
        } else if (temp <= ((sqrt(fabs(d[m - 1])) * sqrt(fabs(d[m]))) *
                            2.2204460492503131E-16)) {
          e[m - 1] = 0.0;
          exitg2 = true;
        } else {
          m++;
        }
      }
      f_l = c_l1 - 1;
      lsv = c_l1;
      lend = m;
      lendsv = m;
      c_l1 = m + 1;
      if (m != (f_l + 1)) {
        double anorm;
        int c_n_tmp;
        int ix;
        int k;
        c_n_tmp = m - f_l;
        if (c_n_tmp <= ((int)((signed char)0))) {
          anorm = 0.0;
        } else {
          anorm = fabs(d[(f_l + c_n_tmp) - 1]);
          k = 0;
          exitg2 = false;
          while ((!exitg2) && (k <= (c_n_tmp - 2))) {
            ix = f_l + k;
            temp = fabs(d[ix]);
            if (rtIsNaN(temp)) {
              anorm = rtNaN;
              exitg2 = true;
            } else {
              if (temp > anorm) {
                anorm = temp;
              }
              temp = fabs(e[ix]);
              if (rtIsNaN(temp)) {
                anorm = rtNaN;
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
        if (!(anorm == 0.0)) {
          if ((rtIsInf(anorm)) || (rtIsNaN(anorm))) {
            d[0] = rtNaN;
            d[1] = rtNaN;
            d[2] = rtNaN;
            d[3] = rtNaN;
            for (b_i = 0; b_i < 16; b_i++) {
              z[b_i] = rtNaN;
            }
            exitg1 = 1;
          } else {
            if (anorm > 2.2346346549904327E+153) {
              ix = 1;
              xzlascl(anorm, 2.2346346549904327E+153, c_n_tmp, d, f_l + 1);
              b_xzlascl(anorm, 2.2346346549904327E+153, c_n_tmp - 1, e,
                        f_l + 1);
            } else if (anorm < 3.02546243347603E-123) {
              ix = 2;
              xzlascl(anorm, 3.02546243347603E-123, c_n_tmp, d, f_l + 1);
              b_xzlascl(anorm, 3.02546243347603E-123, c_n_tmp - 1, e, f_l + 1);
            } else {
              /* no actions */
            }
            if (fabs(d[m - 1]) < fabs(d[f_l])) {
              lend = lsv;
              f_l = m - 1;
            }
            if (lend > (f_l + 1)) {
              int exitg4;
              do {
                exitg4 = 0;
                if ((f_l + 1) != lend) {
                  m = f_l + 1;
                  exitg2 = false;
                  while ((!exitg2) && (m < lend)) {
                    temp = fabs(e[m - 1]);
                    if ((temp * temp) <=
                        (((4.9303806576313238E-32 * fabs(d[m - 1])) *
                          fabs(d[m])) +
                         2.2250738585072014E-308)) {
                      exitg2 = true;
                    } else {
                      m++;
                    }
                  }
                } else {
                  m = lend;
                }
                if (m < lend) {
                  e[m - 1] = 0.0;
                }
                if (m == (f_l + 1)) {
                  f_l++;
                  if ((f_l + 1) > lend) {
                    exitg4 = 1;
                  }
                } else if (m == (f_l + 2)) {
                  d[f_l] = xdlaev2(d[f_l], e[f_l], d[f_l + 1], &temp,
                                   &work[f_l], &b_r);
                  d[f_l + 1] = temp;
                  work[f_l + 3] = b_r;
                  rotateRight(2, z, (f_l * 4) + 1, work, f_l + 1, f_l + 4);
                  e[f_l] = 0.0;
                  f_l += 2;
                  if ((f_l + 1) > lend) {
                    exitg4 = 1;
                  }
                } else if (jtot == ((int)((signed char)120))) {
                  exitg4 = 1;
                } else {
                  double c;
                  double g;
                  double p;
                  jtot++;
                  g = (d[f_l + 1] - d[f_l]) / (2.0 * e[f_l]);
                  temp = fabs(g);
                  if (temp < 1.0) {
                    temp = sqrt((temp * temp) + 1.0);
                  } else if (temp > 1.0) {
                    b_r = 1.0 / temp;
                    temp *= sqrt((b_r * b_r) + 1.0);
                  } else {
                    temp *= 1.4142135623730951;
                  }
                  if (!(g >= 0.0)) {
                    temp = -temp;
                  }
                  g = (d[m - 1] - d[f_l]) + (e[f_l] / (g + temp));
                  s = 1.0;
                  c = 1.0;
                  p = 0.0;
                  k = m - 1;
                  for (b_i = k; b_i >= (f_l + 1); b_i--) {
                    double b;
                    temp = e[b_i - 1];
                    b = c * temp;
                    c = xzlartg(g, s * temp, &s, &b_r);
                    if (b_i != (m - 1)) {
                      e[b_i] = b_r;
                    }
                    g = d[b_i] - p;
                    temp = ((d[b_i - 1] - g) * s) + ((2.0 * c) * b);
                    p = s * temp;
                    d[b_i] = g + p;
                    g = (c * temp) - b;
                    work[b_i - 1] = c;
                    work[b_i + 2] = -s;
                  }
                  rotateRight(m - f_l, z, (f_l * 4) + 1, work, f_l + 1,
                              f_l + 4);
                  d[f_l] -= p;
                  e[f_l] = g;
                }
              } while (exitg4 == ((int)((signed char)0)));
            } else {
              int exitg3;
              do {
                exitg3 = 0;
                if ((f_l + 1) != lend) {
                  m = f_l + 1;
                  exitg2 = false;
                  while ((!exitg2) && (m > lend)) {
                    temp = fabs(e[m - 2]);
                    if ((temp * temp) <=
                        (((4.9303806576313238E-32 * fabs(d[m - 1])) *
                          fabs(d[m - 2])) +
                         2.2250738585072014E-308)) {
                      exitg2 = true;
                    } else {
                      m--;
                    }
                  }
                } else {
                  m = lend;
                }
                if (m > lend) {
                  e[m - 2] = 0.0;
                }
                if (m == (f_l + 1)) {
                  f_l--;
                  if ((f_l + 1) < lend) {
                    exitg3 = 1;
                  }
                } else if (m == f_l) {
                  d[f_l - 1] = xdlaev2(d[f_l - 1], e[f_l - 1], d[f_l], &temp,
                                       &work[m - 1], &b_r);
                  d[f_l] = temp;
                  work[m + 2] = b_r;
                  b_rotateRight(2, z, ((f_l - 1) * 4) + 1, work, m, m + 3);
                  e[f_l - 1] = 0.0;
                  f_l -= 2;
                  if ((f_l + 1) < lend) {
                    exitg3 = 1;
                  }
                } else if (jtot == ((int)((signed char)120))) {
                  exitg3 = 1;
                } else {
                  double c;
                  double g;
                  double p;
                  jtot++;
                  g_tmp = e[f_l - 1];
                  g = (d[f_l - 1] - d[f_l]) / (2.0 * g_tmp);
                  temp = fabs(g);
                  if (temp < 1.0) {
                    temp = sqrt((temp * temp) + 1.0);
                  } else if (temp > 1.0) {
                    b_r = 1.0 / temp;
                    temp *= sqrt((b_r * b_r) + 1.0);
                  } else {
                    temp *= 1.4142135623730951;
                  }
                  if (!(g >= 0.0)) {
                    temp = -temp;
                  }
                  g = (d[m - 1] - d[f_l]) + (g_tmp / (g + temp));
                  s = 1.0;
                  c = 1.0;
                  p = 0.0;
                  for (b_i = m; b_i <= f_l; b_i++) {
                    double b;
                    temp = e[b_i - 1];
                    b = c * temp;
                    c = xzlartg(g, s * temp, &s, &g_tmp);
                    if (b_i != m) {
                      e[b_i - 2] = g_tmp;
                    }
                    g = d[b_i - 1] - p;
                    temp = ((d[b_i] - g) * s) + ((2.0 * c) * b);
                    p = s * temp;
                    d[b_i - 1] = g + p;
                    g = (c * temp) - b;
                    work[b_i - 1] = c;
                    work[b_i + 2] = s;
                  }
                  b_rotateRight((f_l - m) + 2, z, ((m - 1) * 4) + 1, work, m,
                                m + 3);
                  d[f_l] -= p;
                  e[f_l - 1] = g;
                }
              } while (exitg3 == ((int)((signed char)0)));
            }
            if (ix == ((int)((signed char)1))) {
              k = lendsv - lsv;
              xzlascl(2.2346346549904327E+153, anorm, k + 1, d, lsv);
              b_xzlascl(2.2346346549904327E+153, anorm, k, e, lsv);
            } else if (ix == ((int)((signed char)2))) {
              k = lendsv - lsv;
              xzlascl(3.02546243347603E-123, anorm, k + 1, d, lsv);
              b_xzlascl(3.02546243347603E-123, anorm, k, e, lsv);
            } else {
              /* no actions */
            }
            if (jtot >= ((int)((signed char)120))) {
              if (e[0] != 0.0) {
                info = 1;
              }
              if (e[1] != 0.0) {
                info++;
              }
              if (e[2] != 0.0) {
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

static void xzsyhetrd(double A[16], double b_D[4], double b_E[3], double tau[3])
{
  int b_i;
  int b_ii;
  int k;
  for (b_i = 0; b_i < 3; b_i++) {
    double beta1;
    double taui;
    double temp2;
    double xnorm;
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
    taui = 0.0;
    xnorm = xnrm2(2 - b_i, A, ix0);
    if (xnorm != 0.0) {
      beta1 = fabs(A[alpha_tmp_tmp + 1]);
      xnorm = fabs(xnorm);
      if (beta1 < xnorm) {
        beta1 /= xnorm;
        beta1 = xnorm * sqrt((beta1 * beta1) + 1.0);
      } else if (beta1 > xnorm) {
        xnorm /= beta1;
        beta1 *= sqrt((xnorm * xnorm) + 1.0);
      } else if (rtIsNaN(xnorm)) {
        beta1 = rtNaN;
      } else {
        beta1 *= 1.4142135623730951;
      }
      if (temp2 >= 0.0) {
        beta1 = -beta1;
      }
      if (fabs(beta1) < 1.0020841800044864E-292) {
        knt = 0;
        u0 = (ix0 - b_i) + 1;
        do {
          knt++;
          for (k = ix0; k <= u0; k++) {
            A[k - 1] *= 9.9792015476736E+291;
          }
          beta1 *= 9.9792015476736E+291;
          temp2 *= 9.9792015476736E+291;
        } while ((fabs(beta1) < 1.0020841800044864E-292) &&
                 (knt < ((int)((signed char)20))));
        xnorm = fabs(temp2);
        beta1 = fabs(xnrm2(2 - b_i, A, ix0));
        if (xnorm < beta1) {
          xnorm /= beta1;
          beta1 *= sqrt((xnorm * xnorm) + 1.0);
        } else if (xnorm > beta1) {
          beta1 /= xnorm;
          beta1 = xnorm * sqrt((beta1 * beta1) + 1.0);
        } else if (rtIsNaN(beta1)) {
          beta1 = rtNaN;
        } else {
          beta1 = xnorm * 1.4142135623730951;
        }
        if (temp2 >= 0.0) {
          beta1 = -beta1;
        }
        taui = (beta1 - temp2) / beta1;
        xnorm = 1.0 / (temp2 - beta1);
        for (k = ix0; k <= u0; k++) {
          A[k - 1] *= xnorm;
        }
        for (k = 0; k < knt; k++) {
          beta1 *= 1.0020841800044864E-292;
        }
        temp2 = beta1;
      } else {
        taui = (beta1 - temp2) / beta1;
        xnorm = 1.0 / (temp2 - beta1);
        u0 = (ix0 - b_i) + 1;
        for (k = ix0; k <= u0; k++) {
          A[k - 1] *= xnorm;
        }
        temp2 = beta1;
      }
    }
    b_E[b_i] = temp2;
    if (taui != 0.0) {
      int b_tau_tmp;
      int c_i;
      int tau_tmp;
      A[alpha_tmp_tmp + 1] = 1.0;
      for (k = b_i + 1; k < 4; k++) {
        tau[k - 1] = 0.0;
      }
      u0 = 2 - b_i;
      knt = 4 - b_i;
      for (k = 0; k <= u0; k++) {
        ix0 = b_i + k;
        beta1 = taui * A[(ix0 + (4 * b_i)) + 1];
        temp2 = 0.0;
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
      xnorm = 0.0;
      for (k = 0; k <= u0; k++) {
        xnorm += tau[b_i + k] * A[(alpha_tmp_tmp + k) + 1];
      }
      xnorm *= -0.5 * taui;
      if (!(xnorm == 0.0)) {
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

void filter_entry(filter_entryStackData *SD, double x[10], double b_P[100],
                  struct0_T *mem, double dt, const struct2_T *sens_in,
                  bool is_init)
{
  static const double b_y[16] = {1.0E-12, 0.0, 0.0, 0.0,    0.0,     1.0E-12,
                                 0.0,     0.0, 0.0, 0.0,    1.0E-12, 0.0,
                                 0.0,     0.0, 0.0, 1.0E-12};
  static const double y[9] = {0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5};
  static const signed char b[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  double F[100];
  double b_Q[100];
  double c_F[100];
  double b_G[70];
  double c_G[70];
  double Qc[49];
  double c_P[30];
  double d_H[30];
  double e_H[30];
  double Psi[12];
  double c_H[10];
  double c_K[10];
  double R_bn[9];
  double c_R[9];
  double dq[4];
  double w[3];
  int b_i;
  int c_i;
  int i1;
  if (!is_init) {
    double q[4];
    double theta[3];
    double absxk;
    double b_scale;
    double c_a;
    double d;
    double phi;
    double scale;
    double t;
    scale = mem->sens_filt.baro;
    if (sens_in->accel.status) {
      mem->sens_filt.accel[0] =
          ((1.0 - SD->pd->params.a_fast) * mem->sens_filt.accel[0]) +
          (SD->pd->params.a_fast * sens_in->accel.meas[0]);
      mem->sens_filt.accel[1] =
          ((1.0 - SD->pd->params.a_fast) * mem->sens_filt.accel[1]) +
          (SD->pd->params.a_fast * sens_in->accel.meas[1]);
      mem->sens_filt.accel[2] =
          ((1.0 - SD->pd->params.a_fast) * mem->sens_filt.accel[2]) +
          (SD->pd->params.a_fast * sens_in->accel.meas[2]);
    }
    if (sens_in->gyro.status) {
      mem->sens_filt.gyro[0] =
          ((1.0 - SD->pd->params.a_fast) * mem->sens_filt.gyro[0]) +
          (SD->pd->params.a_fast * sens_in->gyro.meas[0]);
      mem->sens_filt.gyro[1] =
          ((1.0 - SD->pd->params.a_fast) * mem->sens_filt.gyro[1]) +
          (SD->pd->params.a_fast * sens_in->gyro.meas[1]);
      mem->sens_filt.gyro[2] =
          ((1.0 - SD->pd->params.a_fast) * mem->sens_filt.gyro[2]) +
          (SD->pd->params.a_fast * sens_in->gyro.meas[2]);
    }
    if (sens_in->mag.status) {
      mem->sens_filt.mag[0] =
          ((1.0 - SD->pd->params.a_slow) * mem->sens_filt.mag[0]) +
          (SD->pd->params.a_slow * sens_in->mag.meas[0]);
      mem->sens_filt.mag[1] =
          ((1.0 - SD->pd->params.a_slow) * mem->sens_filt.mag[1]) +
          (SD->pd->params.a_slow * sens_in->mag.meas[1]);
      mem->sens_filt.mag[2] =
          ((1.0 - SD->pd->params.a_slow) * mem->sens_filt.mag[2]) +
          (SD->pd->params.a_slow * sens_in->mag.meas[2]);
    }
    if (sens_in->baro.status) {
      scale = ((1.0 - SD->pd->params.a_slow) * mem->sens_filt.baro) +
              (SD->pd->params.a_slow * sens_in->baro.meas);
    }
    mem->sens_filt.baro = scale;
    (void)memset(&x[0], 0, 10U * (sizeof(double)));
    scale = 3.3121686421112381E-170;
    absxk = fabs(mem->sens_filt.accel[0]);
    if (absxk > 3.3121686421112381E-170) {
      d = 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      d = t * t;
    }
    absxk = fabs(mem->sens_filt.accel[1]);
    if (absxk > scale) {
      t = scale / absxk;
      d = ((d * t) * t) + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      d += t * t;
    }
    absxk = fabs(mem->sens_filt.accel[2]);
    if (absxk > scale) {
      t = scale / absxk;
      d = ((d * t) * t) + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      d += t * t;
    }
    d = scale * sqrt(d);
    phi = (-mem->sens_filt.accel[0]) / d;
    w[0] = phi;
    c_a = phi * mem->sens_filt.mag[0];
    phi = (-mem->sens_filt.accel[1]) / d;
    w[1] = phi;
    c_a += phi * mem->sens_filt.mag[1];
    phi = (-mem->sens_filt.accel[2]) / d;
    c_a += phi * mem->sens_filt.mag[2];
    scale = 3.3121686421112381E-170;
    d = mem->sens_filt.mag[0] - (c_a * w[0]);
    theta[0] = d;
    absxk = fabs(d);
    if (absxk > 3.3121686421112381E-170) {
      b_scale = 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      b_scale = t * t;
    }
    d = mem->sens_filt.mag[1] - (c_a * w[1]);
    theta[1] = d;
    absxk = fabs(d);
    if (absxk > scale) {
      t = scale / absxk;
      b_scale = ((b_scale * t) * t) + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      b_scale += t * t;
    }
    d = mem->sens_filt.mag[2] - (c_a * phi);
    absxk = fabs(d);
    if (absxk > scale) {
      t = scale / absxk;
      b_scale = ((b_scale * t) * t) + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      b_scale += t * t;
    }
    b_scale = scale * sqrt(b_scale);
    absxk = theta[0] / b_scale;
    theta[0] = absxk;
    R_bn[0] = absxk;
    absxk = theta[1] / b_scale;
    theta[1] = absxk;
    R_bn[3] = absxk;
    absxk = d / b_scale;
    R_bn[1] = (w[1] * absxk) - (theta[1] * phi);
    R_bn[4] = (theta[0] * phi) - (w[0] * absxk);
    R_bn[7] = (w[0] * theta[1]) - (theta[0] * w[1]);
    scale = (R_bn[0] + R_bn[4]) + phi;
    if (scale > 0.0) {
      scale = sqrt(scale + 1.0) * 2.0;
      q[0] = 0.25 * scale;
      q[1] = (w[1] - R_bn[7]) / scale;
      q[2] = (absxk - w[0]) / scale;
      q[3] = (R_bn[1] - R_bn[3]) / scale;
    } else if ((R_bn[0] > R_bn[4]) && (R_bn[0] > phi)) {
      scale = sqrt(((R_bn[0] + 1.0) - R_bn[4]) - phi) * 2.0;
      q[0] = (w[1] - R_bn[7]) / scale;
      q[1] = 0.25 * scale;
      q[2] = (R_bn[1] + R_bn[3]) / scale;
      q[3] = (w[0] + absxk) / scale;
    } else if (R_bn[4] > phi) {
      scale = sqrt(((R_bn[4] + 1.0) - R_bn[0]) - phi) * 2.0;
      q[0] = (absxk - w[0]) / scale;
      q[1] = (R_bn[1] + R_bn[3]) / scale;
      q[2] = 0.25 * scale;
      q[3] = (w[1] + R_bn[7]) / scale;
    } else {
      scale = sqrt(((phi + 1.0) - R_bn[0]) - R_bn[4]) * 2.0;
      q[0] = (R_bn[1] - R_bn[3]) / scale;
      q[1] = (w[0] + absxk) / scale;
      q[2] = (w[1] + R_bn[7]) / scale;
      q[3] = 0.25 * scale;
    }
    scale = b_norm(q);
    q[0] /= scale;
    q[1] /= scale;
    q[2] /= scale;
    q[3] /= scale;
    if (q[0] < 0.0) {
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
    (void)memcpy(&b_P[0], &SD->pd->params.P0[0], 100U * (sizeof(double)));
  } else {
    double b_a[9];
    double q[4];
    double theta[3];
    double absxk;
    double b_scale;
    double c_a;
    double d;
    double g_magnitude;
    double phi;
    double scale;
    double t;
    int F_tmp;
    int H_tmp;
    if (sens_in->gyro.status) {
      double dv[16];
      double e_a[16];
      double b_v[7];
      double K_tmp;
      q[0] = x[0];
      q[1] = x[1];
      q[2] = x[2];
      q[3] = x[3];
      scale = 3.3121686421112381E-170;
      g_magnitude = sens_in->gyro.meas[0] - x[4];
      w[0] = g_magnitude;
      d = g_magnitude * dt;
      theta[0] = d;
      absxk = fabs(d);
      if (absxk > 3.3121686421112381E-170) {
        phi = 1.0;
        scale = absxk;
      } else {
        t = absxk / 3.3121686421112381E-170;
        phi = t * t;
      }
      g_magnitude = sens_in->gyro.meas[1] - x[5];
      w[1] = g_magnitude;
      d = g_magnitude * dt;
      theta[1] = d;
      absxk = fabs(d);
      if (absxk > scale) {
        t = scale / absxk;
        phi = ((phi * t) * t) + 1.0;
        scale = absxk;
      } else {
        t = absxk / scale;
        phi += t * t;
      }
      g_magnitude = sens_in->gyro.meas[2] - x[6];
      d = g_magnitude * dt;
      theta[2] = d;
      absxk = fabs(d);
      if (absxk > scale) {
        t = scale / absxk;
        phi = ((phi * t) * t) + 1.0;
        scale = absxk;
      } else {
        t = absxk / scale;
        phi += t * t;
      }
      phi = scale * sqrt(phi);
      if (phi < 1.0E-8) {
        dq[0] = 1.0;
        for (b_i = 0; b_i < 3; b_i++) {
          scale = theta[b_i];
          dq[b_i + 1] = scale / 2.0;
          Psi[4 * b_i] = (-scale) / 4.0;
          Psi[(4 * b_i) + 1] = y[3 * b_i];
          Psi[(4 * b_i) + 2] = y[(3 * b_i) + 1];
          Psi[(4 * b_i) + 3] = y[(3 * b_i) + 2];
        }
      } else {
        scale = phi / 2.0;
        absxk = sin(scale);
        scale = cos(scale);
        dq[0] = scale;
        c_a = theta[0] / phi;
        theta[0] = c_a;
        dq[1] = c_a * absxk;
        c_a = theta[1] / phi;
        theta[1] = c_a;
        dq[2] = c_a * absxk;
        c_a = d / phi;
        theta[2] = c_a;
        dq[3] = c_a * absxk;
        for (b_i = 0; b_i < 3; b_i++) {
          R_bn[3 * b_i] = theta[0] * theta[b_i];
          R_bn[(3 * b_i) + 1] = theta[1] * theta[b_i];
          R_bn[(3 * b_i) + 2] = c_a * theta[b_i];
        }
        d = -0.5 * absxk;
        t = absxk / phi;
        (void)memset(&c_R[0], 0, 9U * (sizeof(double)));
        absxk = 0.5 * scale;
        c_R[0] = 1.0;
        Psi[0] = d * theta[0];
        c_R[4] = 1.0;
        Psi[4] = d * theta[1];
        c_R[8] = 1.0;
        Psi[8] = d * c_a;
        for (b_i = 0; b_i < 3; b_i++) {
          scale = R_bn[3 * b_i];
          Psi[(4 * b_i) + 1] = (t * (c_R[3 * b_i] - scale)) + (absxk * scale);
          F_tmp = (3 * b_i) + 1;
          scale = R_bn[F_tmp];
          Psi[(4 * b_i) + 2] = (t * (c_R[F_tmp] - scale)) + (absxk * scale);
          F_tmp = (3 * b_i) + 2;
          scale = R_bn[F_tmp];
          Psi[(4 * b_i) + 3] = (t * (c_R[F_tmp] - scale)) + (absxk * scale);
        }
      }
      scale = x[0];
      c_K[1] = ((scale * dq[1]) + (dq[0] * x[1])) +
               ((x[2] * dq[3]) - (dq[2] * x[3]));
      c_K[4] = x[4];
      c_K[2] = ((scale * dq[2]) + (dq[0] * x[2])) +
               ((dq[1] * x[3]) - (x[1] * dq[3]));
      c_K[5] = x[5];
      c_K[3] = ((scale * dq[3]) + (dq[0] * x[3])) +
               ((x[1] * dq[2]) - (dq[1] * x[2]));
      c_K[6] = x[6];
      c_K[0] =
          (x[0] * dq[0]) - (((x[1] * dq[1]) + (x[2] * dq[2])) + (x[3] * dq[3]));
      c_K[7] = x[7];
      c_K[8] = x[8] + (x[7] * dt);
      K_tmp = dt * dt;
      c_K[9] = (x[9] + (x[8] * dt)) + ((0.5 * x[7]) * K_tmp);
      (void)memcpy(&x[0], &c_K[0], 10U * (sizeof(double)));
      (void)memset(&F[0], 0, 100U * (sizeof(double)));
      scale = 0.5 * dt;
      d = scale * 0.0;
      e_a[0] = d;
      c_a = scale * (-w[0]);
      e_a[4] = c_a;
      absxk = scale * (-w[1]);
      e_a[8] = absxk;
      b_scale = scale * (-g_magnitude);
      e_a[12] = b_scale;
      phi = scale * w[0];
      e_a[1] = phi;
      e_a[5] = d;
      t = scale * g_magnitude;
      e_a[9] = t;
      e_a[13] = absxk;
      scale *= w[1];
      e_a[2] = scale;
      e_a[6] = b_scale;
      e_a[10] = d;
      e_a[14] = phi;
      e_a[3] = t;
      e_a[7] = scale;
      e_a[11] = c_a;
      e_a[15] = d;
      expm(e_a, dv);
      for (b_i = 0; b_i < 4; b_i++) {
        F[10 * b_i] = dv[4 * b_i];
        F[(10 * b_i) + 1] = dv[(4 * b_i) + 1];
        F[(10 * b_i) + 2] = dv[(4 * b_i) + 2];
        F[(10 * b_i) + 3] = dv[(4 * b_i) + 3];
      }
      absxk = (-dt) * q[0];
      e_a[0] = absxk;
      t = (-dt) * (-q[1]);
      e_a[4] = t;
      d = (-dt) * (-q[2]);
      e_a[8] = d;
      scale = (-dt) * (-q[3]);
      e_a[12] = scale;
      c_a = (-dt) * q[1];
      e_a[1] = c_a;
      e_a[5] = absxk;
      e_a[9] = scale;
      scale = (-dt) * q[2];
      e_a[13] = scale;
      e_a[2] = scale;
      scale = (-dt) * q[3];
      e_a[6] = scale;
      e_a[10] = absxk;
      e_a[14] = t;
      e_a[3] = scale;
      e_a[7] = d;
      e_a[11] = c_a;
      e_a[15] = absxk;
      for (b_i = 0; b_i < 3; b_i++) {
        F_tmp = 10 * (b_i + 4);
        scale = 0.0;
        absxk = 0.0;
        t = 0.0;
        d = 0.0;
        for (c_i = 0; c_i < 4; c_i++) {
          c_a = Psi[c_i + (4 * b_i)];
          scale += e_a[4 * c_i] * c_a;
          absxk += e_a[(4 * c_i) + 1] * c_a;
          t += e_a[(4 * c_i) + 2] * c_a;
          d += e_a[(4 * c_i) + 3] * c_a;
        }
        F[F_tmp + 3] = d;
        F[F_tmp + 2] = t;
        F[F_tmp + 1] = absxk;
        F[F_tmp] = scale;
      }
      (void)memset(&R_bn[0], 0, 9U * (sizeof(double)));
      R_bn[0] = 1.0;
      R_bn[4] = 1.0;
      R_bn[8] = 1.0;
      for (b_i = 0; b_i < 3; b_i++) {
        F_tmp = 10 * (b_i + 4);
        F[F_tmp + 4] = R_bn[3 * b_i];
        F[F_tmp + 5] = R_bn[(3 * b_i) + 1];
        F[F_tmp + 6] = R_bn[(3 * b_i) + 2];
      }
      F[77] = 1.0;
      F[78] = dt;
      F[88] = 1.0;
      F[79] = 0.5 * K_tmp;
      F[89] = dt;
      F[99] = 1.0;
      (void)memset(&b_G[0], 0, 70U * (sizeof(double)));
      b_G[67] = 1.0;
      absxk = SD->pd->b_params.sg * SD->pd->b_params.sg;
      scale = SD->pd->b_params.sbg * SD->pd->b_params.sbg;
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
        b_v[b_i] = absxk;
        b_v[b_i + 3] = scale;
      }
      d = SD->pd->b_params.sa * SD->pd->b_params.sa;
      b_v[6] = d;
      (void)memset(&Qc[0], 0, 49U * (sizeof(double)));
      for (b_i = 0; b_i < 7; b_i++) {
        Qc[b_i + (7 * b_i)] = b_v[b_i];
      }
      (void)memset(&c_G[0], 0, 70U * (sizeof(double)));
      for (b_i = 0; b_i < 7; b_i++) {
        for (c_i = 0; c_i < 7; c_i++) {
          scale = Qc[c_i + (7 * b_i)];
          for (i1 = 0; i1 < 10; i1++) {
            F_tmp = i1 + (10 * b_i);
            c_G[F_tmp] += b_G[i1 + (10 * c_i)] * scale;
          }
        }
      }
      (void)memset(&b_Q[0], 0, 100U * (sizeof(double)));
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 7; c_i++) {
          scale = b_G[b_i + (10 * c_i)];
          for (i1 = 0; i1 < 10; i1++) {
            F_tmp = i1 + (10 * b_i);
            b_Q[F_tmp] += c_G[i1 + (10 * c_i)] * scale;
          }
        }
      }
      for (b_i = 0; b_i < 100; b_i++) {
        b_Q[b_i] *= dt;
      }
      absxk = rt_powd_snf(dt, 3.0);
      b_Q[77] = d * dt;
      scale = d * (K_tmp / 2.0);
      b_Q[87] = scale;
      t = d * (absxk / 6.0);
      b_Q[97] = t;
      b_Q[78] = scale;
      b_Q[88] = d * (absxk / 3.0);
      scale = d * (rt_powd_snf(dt, 4.0) / 8.0);
      b_Q[98] = scale;
      b_Q[79] = t;
      b_Q[89] = scale;
      b_Q[99] = d * (rt_powd_snf(dt, 5.0) / 20.0);
      for (b_i = 0; b_i < 4; b_i++) {
        b_Q[10 * b_i] += b_y[4 * b_i];
        F_tmp = (10 * b_i) + 1;
        b_Q[F_tmp] += b_y[(4 * b_i) + 1];
        F_tmp = (10 * b_i) + 2;
        b_Q[F_tmp] += b_y[(4 * b_i) + 2];
        F_tmp = (10 * b_i) + 3;
        b_Q[F_tmp] += b_y[(4 * b_i) + 3];
      }
      (void)memset(&c_F[0], 0, 100U * (sizeof(double)));
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 10; c_i++) {
          scale = b_P[c_i + (10 * b_i)];
          for (i1 = 0; i1 < 10; i1++) {
            F_tmp = i1 + (10 * b_i);
            c_F[F_tmp] += F[i1 + (10 * c_i)] * scale;
          }
        }
      }
      for (c_i = 0; c_i < 10; c_i++) {
        for (i1 = 0; i1 < 10; i1++) {
          scale = 0.0;
          for (b_i = 0; b_i < 10; b_i++) {
            scale += c_F[c_i + (10 * b_i)] * F[i1 + (10 * b_i)];
          }
          F_tmp = c_i + (10 * i1);
          b_P[F_tmp] = scale + b_Q[F_tmp];
        }
      }
      scale = b_norm(&x[0]);
      x[0] /= scale;
      x[1] /= scale;
      x[2] /= scale;
      x[3] /= scale;
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 10; c_i++) {
          F_tmp = c_i + (10 * b_i);
          F[F_tmp] = (b_P[F_tmp] + b_P[b_i + (10 * c_i)]) / 2.0;
        }
      }
      (void)memcpy(&b_P[0], &F[0], 100U * (sizeof(double)));
    }
    if (sens_in->accel.status) {
      double d_K[30];
      scale = 3.3121686421112381E-170;
      absxk = fabs(mem->sens_filt.accel[0]);
      if (absxk > 3.3121686421112381E-170) {
        g_magnitude = 1.0;
        scale = absxk;
      } else {
        t = absxk / 3.3121686421112381E-170;
        g_magnitude = t * t;
      }
      absxk = fabs(mem->sens_filt.accel[1]);
      if (absxk > scale) {
        t = scale / absxk;
        g_magnitude = ((g_magnitude * t) * t) + 1.0;
        scale = absxk;
      } else {
        t = absxk / scale;
        g_magnitude += t * t;
      }
      absxk = fabs(mem->sens_filt.accel[2]);
      if (absxk > scale) {
        t = scale / absxk;
        g_magnitude = ((g_magnitude * t) * t) + 1.0;
        scale = absxk;
      } else {
        t = absxk / scale;
        g_magnitude += t * t;
      }
      g_magnitude = scale * sqrt(g_magnitude);
      w[0] = 0.0;
      w[1] = 0.0;
      w[2] = x[7] - g_magnitude;
      scale = b_norm(&x[0]);
      q[0] = x[0] / scale;
      q[1] = x[1] / scale;
      q[2] = x[2] / scale;
      q[3] = x[3] / scale;
      scale = 0.0;
      t = 2.0 * q[0];
      for (b_i = 0; b_i < 3; b_i++) {
        absxk = q[b_i + 1];
        scale += absxk * absxk;
        c_R[3 * b_i] = absxk * q[1];
        c_R[(3 * b_i) + 1] = absxk * q[2];
        c_R[(3 * b_i) + 2] = absxk * q[3];
      }
      scale = (q[0] * q[0]) - scale;
      absxk = t * 0.0;
      b_a[0] = absxk;
      b_a[1] = t * (-q[3]);
      b_a[2] = t * q[2];
      b_a[3] = t * q[3];
      b_a[4] = absxk;
      b_a[5] = t * (-q[1]);
      b_a[6] = t * (-q[2]);
      b_a[7] = t * q[1];
      b_a[8] = absxk;
      for (b_i = 0; b_i < 3; b_i++) {
        R_bn[3 * b_i] =
            ((scale * ((double)b[b_i])) + (2.0 * c_R[3 * b_i])) + b_a[3 * b_i];
        F_tmp = (3 * b_i) + 1;
        R_bn[F_tmp] =
            ((scale * ((double)b[b_i + 3])) + (2.0 * c_R[F_tmp])) + b_a[F_tmp];
        F_tmp = (3 * b_i) + 2;
        R_bn[F_tmp] =
            ((scale * ((double)b[b_i + 6])) + (2.0 * c_R[F_tmp])) + b_a[F_tmp];
      }
      (void)memset(&d_H[0], 0, 30U * (sizeof(double)));
      scale = 2.0 * x[0];
      absxk = scale * 0.0;
      d_H[0] = absxk - (2.0 * ((x[2] * w[2]) - (0.0 * x[3])));
      d_H[1] = absxk - (2.0 * ((0.0 * x[3]) - (x[1] * w[2])));
      d_H[2] = (scale * w[2]) - (2.0 * ((x[1] * 0.0) - (0.0 * x[2])));
      phi = 2.0 * (((x[1] * 0.0) + (x[2] * 0.0)) + (w[2] * x[3]));
      scale = 2.0 * x[0];
      absxk = scale * 0.0;
      b_a[0] = absxk;
      b_a[3] = scale * (-w[2]);
      b_a[6] = absxk;
      b_a[1] = scale * w[2];
      b_a[4] = absxk;
      d = scale * -0.0;
      b_a[7] = d;
      c_a = 0.0;
      b_scale = 3.3121686421112381E-170;
      for (b_i = 0; b_i < 3; b_i++) {
        int b_H_tmp;
        F_tmp = (3 * b_i) + 2;
        b_a[F_tmp] = d;
        scale = x[b_i + 1];
        absxk = -0.0 * scale;
        H_tmp = 3 * (b_i + 1);
        t = w[b_i];
        d_H[H_tmp] =
            ((absxk + (phi * ((double)b[3 * b_i]))) + ((2.0 * x[1]) * t)) +
            b_a[3 * b_i];
        b_H_tmp = (3 * b_i) + 1;
        d_H[H_tmp + 1] =
            ((absxk + (phi * ((double)b[b_H_tmp]))) + ((2.0 * x[2]) * t)) +
            b_a[b_H_tmp];
        d_H[H_tmp + 2] =
            ((((-2.0 * w[2]) * scale) + (phi * ((double)b[F_tmp]))) +
             ((2.0 * x[3]) * w[b_i])) +
            d;
        d_H[b_i + 21] = R_bn[b_i + 6];
        scale = fabs(sens_in->accel.meas[b_i]);
        if (scale > b_scale) {
          absxk = b_scale / scale;
          c_a = ((c_a * absxk) * absxk) + 1.0;
          b_scale = scale;
        } else {
          absxk = scale / b_scale;
          c_a += absxk * absxk;
        }
      }
      c_a = b_scale * sqrt(c_a);
      scale = fabs(c_a - g_magnitude) / 0.5;
      scale *= scale;
      for (b_i = 0; b_i < 9; b_i++) {
        c_R[b_i] = SD->pd->c_params.R_accel[b_i] * (scale + 1.0);
      }
      for (b_i = 0; b_i < 3; b_i++) {
        for (c_i = 0; c_i < 10; c_i++) {
          d_K[c_i + (10 * b_i)] = d_H[b_i + (3 * c_i)];
        }
      }
      (void)memset(&e_H[0], 0, 30U * (sizeof(double)));
      for (b_i = 0; b_i < 10; b_i++) {
        scale = e_H[3 * b_i];
        F_tmp = (3 * b_i) + 1;
        H_tmp = (3 * b_i) + 2;
        for (c_i = 0; c_i < 10; c_i++) {
          absxk = b_P[c_i + (10 * b_i)];
          scale += d_H[3 * c_i] * absxk;
          e_H[F_tmp] += d_H[(3 * c_i) + 1] * absxk;
          e_H[H_tmp] += d_H[(3 * c_i) + 2] * absxk;
        }
        e_H[3 * b_i] = scale;
      }
      (void)memset(&c_P[0], 0, 30U * (sizeof(double)));
      for (i1 = 0; i1 < 3; i1++) {
        for (b_i = 0; b_i < 10; b_i++) {
          scale = d_K[b_i + (10 * i1)];
          for (c_i = 0; c_i < 10; c_i++) {
            F_tmp = c_i + (10 * i1);
            c_P[F_tmp] += b_P[c_i + (10 * b_i)] * scale;
          }
        }
        for (c_i = 0; c_i < 3; c_i++) {
          scale = 0.0;
          for (b_i = 0; b_i < 10; b_i++) {
            scale += e_H[i1 + (3 * b_i)] * d_K[b_i + (10 * c_i)];
          }
          F_tmp = i1 + (3 * c_i);
          b_a[F_tmp] = scale + c_R[F_tmp];
        }
      }
      mrdiv(c_P, b_a, d_K);
      for (b_i = 0; b_i < 3; b_i++) {
        theta[b_i] = sens_in->accel.meas[b_i] -
                     (((R_bn[b_i] * 0.0) + (R_bn[b_i + 3] * 0.0)) +
                      (R_bn[b_i + 6] * w[2]));
      }
      scale = theta[0];
      absxk = theta[1];
      t = theta[2];
      for (b_i = 0; b_i < 10; b_i++) {
        x[b_i] += ((d_K[b_i] * scale) + (d_K[b_i + 10] * absxk)) +
                  (d_K[b_i + 20] * t);
      }
      (void)memset(&F[0], 0, 100U * (sizeof(double)));
      for (b_i = 0; b_i < 10; b_i++) {
        F[b_i + (10 * b_i)] = 1.0;
      }
      for (b_i = 0; b_i < 10; b_i++) {
        scale = d_K[b_i];
        absxk = d_K[b_i + 10];
        t = d_K[b_i + 20];
        for (c_i = 0; c_i < 10; c_i++) {
          F_tmp = b_i + (10 * c_i);
          b_Q[F_tmp] =
              F[F_tmp] -
              (((scale * d_H[3 * c_i]) + (absxk * d_H[(3 * c_i) + 1])) +
               (t * d_H[(3 * c_i) + 2]));
        }
      }
      (void)memset(&F[0], 0, 100U * (sizeof(double)));
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 10; c_i++) {
          scale = b_P[c_i + (10 * b_i)];
          for (i1 = 0; i1 < 10; i1++) {
            F_tmp = i1 + (10 * b_i);
            F[F_tmp] += b_Q[i1 + (10 * c_i)] * scale;
          }
        }
      }
      (void)memset(&c_P[0], 0, 30U * (sizeof(double)));
      for (b_i = 0; b_i < 3; b_i++) {
        for (c_i = 0; c_i < 3; c_i++) {
          scale = c_R[c_i + (3 * b_i)];
          for (i1 = 0; i1 < 10; i1++) {
            F_tmp = i1 + (10 * b_i);
            c_P[F_tmp] += d_K[i1 + (10 * c_i)] * scale;
          }
        }
      }
      (void)memset(&b_P[0], 0, 100U * (sizeof(double)));
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 10; c_i++) {
          scale = b_Q[b_i + (10 * c_i)];
          for (i1 = 0; i1 < 10; i1++) {
            F_tmp = i1 + (10 * b_i);
            b_P[F_tmp] += F[i1 + (10 * c_i)] * scale;
          }
        }
      }
      (void)memset(&F[0], 0, 100U * (sizeof(double)));
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 3; c_i++) {
          scale = d_K[b_i + (10 * c_i)];
          for (i1 = 0; i1 < 10; i1++) {
            F_tmp = i1 + (10 * b_i);
            F[F_tmp] += c_P[i1 + (10 * c_i)] * scale;
          }
        }
      }
      for (b_i = 0; b_i < 100; b_i++) {
        b_P[b_i] += F[b_i];
      }
      scale = b_norm(&x[0]);
      x[0] /= scale;
      x[1] /= scale;
      x[2] /= scale;
      x[3] /= scale;
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 10; c_i++) {
          F_tmp = c_i + (10 * b_i);
          F[F_tmp] = (b_P[F_tmp] + b_P[b_i + (10 * c_i)]) / 2.0;
        }
      }
      (void)memcpy(&b_P[0], &F[0], 100U * (sizeof(double)));
    }
    if (sens_in->mag.status) {
      scale = b_norm(&x[0]);
      q[0] = x[0] / scale;
      q[1] = x[1] / scale;
      q[2] = x[2] / scale;
      q[3] = x[3] / scale;
      scale = 0.0;
      t = 2.0 * q[0];
      for (b_i = 0; b_i < 3; b_i++) {
        absxk = q[b_i + 1];
        scale += absxk * absxk;
        c_R[3 * b_i] = q[1] * absxk;
        c_R[(3 * b_i) + 1] = q[2] * absxk;
        c_R[(3 * b_i) + 2] = q[3] * absxk;
      }
      scale = (q[0] * q[0]) - scale;
      absxk = t * 0.0;
      b_a[0] = absxk;
      b_a[3] = t * (-q[3]);
      b_a[6] = t * q[2];
      b_a[1] = t * q[3];
      b_a[4] = absxk;
      b_a[7] = t * (-q[1]);
      b_a[2] = t * (-q[2]);
      b_a[5] = t * q[1];
      b_a[8] = absxk;
      for (b_i = 0; b_i < 9; b_i++) {
        c_R[b_i] = ((scale * ((double)b[b_i])) + (2.0 * c_R[b_i])) + b_a[b_i];
      }
      (void)memset(&w[0], 0, 3U * (sizeof(double)));
      c_a = w[0];
      b_scale = w[1];
      for (b_i = 0; b_i < 3; b_i++) {
        scale = sens_in->mag.meas[b_i];
        c_a += c_R[3 * b_i] * scale;
        b_scale += c_R[(3 * b_i) + 1] * scale;
      }
      d = (c_a * c_a) + (b_scale * b_scale);
      (void)memset(&Psi[0], 0, 12U * (sizeof(double)));
      scale = 2.0 * x[0];
      Psi[0] = (scale * sens_in->mag.meas[0]) +
               (2.0 * ((x[2] * sens_in->mag.meas[2]) -
                       (sens_in->mag.meas[1] * x[3])));
      Psi[1] = (scale * sens_in->mag.meas[1]) +
               (2.0 * ((sens_in->mag.meas[0] * x[3]) -
                       (x[1] * sens_in->mag.meas[2])));
      Psi[2] = (scale * sens_in->mag.meas[2]) +
               (2.0 * ((x[1] * sens_in->mag.meas[1]) -
                       (sens_in->mag.meas[0] * x[2])));
      t = 2.0 *
          (((sens_in->mag.meas[0] * x[1]) + (sens_in->mag.meas[1] * x[2])) +
           (sens_in->mag.meas[2] * x[3]));
      scale = 2.0 * x[0];
      absxk = scale * 0.0;
      b_a[0] = absxk;
      b_a[3] = scale * (-sens_in->mag.meas[2]);
      b_a[6] = scale * sens_in->mag.meas[1];
      b_a[1] = scale * sens_in->mag.meas[2];
      b_a[4] = absxk;
      b_a[7] = scale * (-sens_in->mag.meas[0]);
      b_a[2] = scale * (-sens_in->mag.meas[1]);
      b_a[5] = scale * sens_in->mag.meas[0];
      b_a[8] = absxk;
      for (b_i = 0; b_i < 3; b_i++) {
        scale = x[b_i + 1];
        H_tmp = 3 * (b_i + 1);
        Psi[H_tmp] = ((((-2.0 * sens_in->mag.meas[0]) * scale) +
                       (t * ((double)b[3 * b_i]))) +
                      ((2.0 * x[1]) * sens_in->mag.meas[b_i])) -
                     b_a[3 * b_i];
        F_tmp = (3 * b_i) + 1;
        Psi[H_tmp + 1] = ((((-2.0 * sens_in->mag.meas[1]) * scale) +
                           (t * ((double)b[F_tmp]))) +
                          ((2.0 * x[2]) * sens_in->mag.meas[b_i])) -
                         b_a[F_tmp];
        F_tmp = (3 * b_i) + 2;
        Psi[H_tmp + 2] = ((((-2.0 * sens_in->mag.meas[2]) * scale) +
                           (t * ((double)b[F_tmp]))) +
                          ((2.0 * x[3]) * sens_in->mag.meas[b_i])) -
                         b_a[F_tmp];
      }
      (void)memset(&c_H[0], 0, 10U * (sizeof(double)));
      scale = (-b_scale) / d;
      absxk = c_a / d;
      for (b_i = 0; b_i < 4; b_i++) {
        c_H[b_i] = ((scale * Psi[3 * b_i]) + (absxk * Psi[(3 * b_i) + 1])) +
                   (0.0 * Psi[(3 * b_i) + 2]);
      }
      d = (SD->pd->d_params.R_mag *
           (((sens_in->mag.meas[0] * sens_in->mag.meas[0]) +
             (sens_in->mag.meas[1] * sens_in->mag.meas[1])) +
            (sens_in->mag.meas[2] * sens_in->mag.meas[2]))) /
          d;
      scale = b_atan2(b_scale, c_a);
      t = b_atan2(sin(-scale), cos(-scale));
      (void)memset(&c_K[0], 0, 10U * (sizeof(double)));
      scale = 0.0;
      for (b_i = 0; b_i < 10; b_i++) {
        absxk = c_K[b_i];
        for (c_i = 0; c_i < 10; c_i++) {
          absxk += c_H[c_i] * b_P[c_i + (10 * b_i)];
        }
        c_K[b_i] = absxk;
        scale += absxk * c_H[b_i];
      }
      absxk = scale + d;
      for (b_i = 0; b_i < 10; b_i++) {
        scale = 0.0;
        for (c_i = 0; c_i < 10; c_i++) {
          scale += b_P[b_i + (10 * c_i)] * c_H[c_i];
        }
        scale /= absxk;
        c_K[b_i] = scale;
        x[b_i] += scale * t;
      }
      (void)memset(&F[0], 0, 100U * (sizeof(double)));
      for (b_i = 0; b_i < 10; b_i++) {
        F[b_i + (10 * b_i)] = 1.0;
      }
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 10; c_i++) {
          F_tmp = c_i + (10 * b_i);
          b_Q[F_tmp] = F[F_tmp] - (c_K[c_i] * c_H[b_i]);
        }
      }
      (void)memset(&F[0], 0, 100U * (sizeof(double)));
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 10; c_i++) {
          scale = b_P[c_i + (10 * b_i)];
          for (i1 = 0; i1 < 10; i1++) {
            F_tmp = i1 + (10 * b_i);
            F[F_tmp] += b_Q[i1 + (10 * c_i)] * scale;
          }
        }
      }
      (void)memset(&b_P[0], 0, 100U * (sizeof(double)));
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 10; c_i++) {
          scale = b_Q[b_i + (10 * c_i)];
          for (i1 = 0; i1 < 10; i1++) {
            F_tmp = i1 + (10 * b_i);
            b_P[F_tmp] += F[i1 + (10 * c_i)] * scale;
          }
        }
      }
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 10; c_i++) {
          F[c_i + (10 * b_i)] = (c_K[c_i] * d) * c_K[b_i];
        }
      }
      for (b_i = 0; b_i < 100; b_i++) {
        b_P[b_i] += F[b_i];
      }
      scale = b_norm(&x[0]);
      x[0] /= scale;
      x[1] /= scale;
      x[2] /= scale;
      x[3] /= scale;
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 10; c_i++) {
          F_tmp = c_i + (10 * b_i);
          F[F_tmp] = (b_P[F_tmp] + b_P[b_i + (10 * c_i)]) / 2.0;
        }
      }
      (void)memcpy(&b_P[0], &F[0], 100U * (sizeof(double)));
    }
    if (sens_in->baro.status) {
      scale = 1.0 - ((0.0065 * x[9]) / 288.15);
      (void)memset(&c_H[0], 0, 10U * (sizeof(double)));
      c_H[9] = ((((-mem->sens_filt.baro) * 5.2559) * 0.0065) / 288.15) *
               rt_powd_snf(scale, 4.2559);
      t = sens_in->baro.meas -
          (mem->sens_filt.baro * rt_powd_snf(scale, 5.2559));
      (void)memset(&c_K[0], 0, 10U * (sizeof(double)));
      scale = 0.0;
      for (b_i = 0; b_i < 10; b_i++) {
        absxk = c_K[b_i];
        for (c_i = 0; c_i < 10; c_i++) {
          absxk += c_H[c_i] * b_P[c_i + (10 * b_i)];
        }
        c_K[b_i] = absxk;
        scale += absxk * c_H[b_i];
      }
      absxk = scale + SD->pd->e_params.R_baro;
      for (b_i = 0; b_i < 10; b_i++) {
        scale = 0.0;
        for (c_i = 0; c_i < 10; c_i++) {
          scale += b_P[b_i + (10 * c_i)] * c_H[c_i];
        }
        scale /= absxk;
        c_K[b_i] = scale;
        x[b_i] += scale * t;
      }
      (void)memset(&F[0], 0, 100U * (sizeof(double)));
      for (b_i = 0; b_i < 10; b_i++) {
        F[b_i + (10 * b_i)] = 1.0;
      }
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 10; c_i++) {
          F_tmp = c_i + (10 * b_i);
          b_Q[F_tmp] = F[F_tmp] - (c_K[c_i] * c_H[b_i]);
        }
      }
      (void)memset(&F[0], 0, 100U * (sizeof(double)));
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 10; c_i++) {
          scale = b_P[c_i + (10 * b_i)];
          for (i1 = 0; i1 < 10; i1++) {
            F_tmp = i1 + (10 * b_i);
            F[F_tmp] += b_Q[i1 + (10 * c_i)] * scale;
          }
        }
      }
      (void)memset(&b_P[0], 0, 100U * (sizeof(double)));
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 10; c_i++) {
          scale = b_Q[b_i + (10 * c_i)];
          for (i1 = 0; i1 < 10; i1++) {
            F_tmp = i1 + (10 * b_i);
            b_P[F_tmp] += F[i1 + (10 * c_i)] * scale;
          }
        }
      }
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 10; c_i++) {
          F[c_i + (10 * b_i)] = (c_K[c_i] * SD->pd->e_params.R_baro) * c_K[b_i];
        }
      }
      for (b_i = 0; b_i < 100; b_i++) {
        b_P[b_i] += F[b_i];
      }
      scale = b_norm(&x[0]);
      x[0] /= scale;
      x[1] /= scale;
      x[2] /= scale;
      x[3] /= scale;
      for (b_i = 0; b_i < 10; b_i++) {
        for (c_i = 0; c_i < 10; c_i++) {
          F_tmp = c_i + (10 * b_i);
          F[F_tmp] = (b_P[F_tmp] + b_P[b_i + (10 * c_i)]) / 2.0;
        }
      }
      (void)memcpy(&b_P[0], &F[0], 100U * (sizeof(double)));
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
