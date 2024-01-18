//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// EKFCorrectorAdditive.cpp
//
// Code generation for function 'EKFCorrectorAdditive'
//

// Include files
#include "EKFCorrectorAdditive.h"
#include "EKF_Node_rtwutil.h"
#include "rt_nonfinite.h"
#include "trisolve.h"
#include "xnrm2.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace matlabshared {
namespace tracking {
namespace internal {
void EKFCorrectorAdditive::correctStateAndSqrtCovariance(
    double x[70], double S[4900], const double residue[14],
    const double Pxy[980], const double Sy[196], const double H[980],
    const double Rsqrt[196])
{
  static double M[5880];
  double A[4900];
  double C[4900];
  double K[980];
  double b_Pxy[980];
  double b_Sy[196];
  double tau[70];
  double work[70];
  double bkj;
  double d;
  int coffset;
  int i;
  int i1;
  int k;
  int knt;
  for (i = 0; i < 70; i++) {
    for (i1 = 0; i1 < 14; i1++) {
      b_Pxy[i1 + 14 * i] = Pxy[i + 70 * i1];
    }
  }
  ::coder::internal::trisolve(Sy, b_Pxy);
  for (i = 0; i < 14; i++) {
    for (i1 = 0; i1 < 14; i1++) {
      b_Sy[i1 + 14 * i] = Sy[i + 14 * i1];
    }
  }
  ::coder::internal::b_trisolve(b_Sy, b_Pxy);
  for (i = 0; i < 14; i++) {
    for (i1 = 0; i1 < 70; i1++) {
      K[i1 + 70 * i] = b_Pxy[i + 14 * i1];
    }
  }
  for (i = 0; i < 70; i++) {
    d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      d += K[i + 70 * i1] * residue[i1];
    }
    x[i] += d;
  }
  for (i = 0; i < 980; i++) {
    b_Pxy[i] = -K[i];
  }
  for (i = 0; i < 70; i++) {
    for (i1 = 0; i1 < 70; i1++) {
      d = 0.0;
      for (coffset = 0; coffset < 14; coffset++) {
        d += b_Pxy[i + 70 * coffset] * H[coffset + 14 * i1];
      }
      A[i + 70 * i1] = d;
    }
  }
  for (int b_i{0}; b_i < 70; b_i++) {
    coffset = b_i + 70 * b_i;
    A[coffset]++;
  }
  for (int j{0}; j < 70; j++) {
    coffset = j * 70;
    std::memset(&C[coffset], 0, 70U * sizeof(double));
    for (k = 0; k < 70; k++) {
      bkj = A[k * 70 + j];
      for (int b_i{0}; b_i < 70; b_i++) {
        knt = coffset + b_i;
        C[knt] += S[b_i * 70 + k] * bkj;
      }
    }
  }
  for (i = 0; i < 14; i++) {
    for (i1 = 0; i1 < 70; i1++) {
      d = 0.0;
      for (coffset = 0; coffset < 14; coffset++) {
        d += K[i1 + 70 * coffset] * Rsqrt[coffset + 14 * i];
      }
      b_Pxy[i + 14 * i1] = d;
    }
  }
  for (int b_i{0}; b_i < 70; b_i++) {
    std::copy(&C[b_i * 70],
              &C[static_cast<int>(static_cast<unsigned int>(b_i * 70) + 70U)],
              &M[b_i * 84]);
    std::copy(
        &b_Pxy[b_i * 14],
        &b_Pxy[static_cast<int>(static_cast<unsigned int>(b_i * 14) + 14U)],
        &M[b_i * 84 + 70]);
    tau[b_i] = 0.0;
    work[b_i] = 0.0;
  }
  for (int b_i{0}; b_i < 70; b_i++) {
    double atmp;
    int ii;
    ii = b_i * 84 + b_i;
    atmp = M[ii];
    coffset = ii + 2;
    tau[b_i] = 0.0;
    bkj = ::coder::internal::blas::f_xnrm2(83 - b_i, M, ii + 2);
    if (bkj != 0.0) {
      double beta1;
      d = M[ii];
      beta1 = rt_hypotd_snf(d, bkj);
      if (d >= 0.0) {
        beta1 = -beta1;
      }
      if (std::abs(beta1) < 1.0020841800044864E-292) {
        knt = 0;
        i = (ii - b_i) + 84;
        do {
          knt++;
          for (k = coffset; k <= i; k++) {
            M[k - 1] *= 9.9792015476736E+291;
          }
          beta1 *= 9.9792015476736E+291;
          atmp *= 9.9792015476736E+291;
        } while ((std::abs(beta1) < 1.0020841800044864E-292) && (knt < 20));
        beta1 = rt_hypotd_snf(
            atmp, ::coder::internal::blas::f_xnrm2(83 - b_i, M, ii + 2));
        if (atmp >= 0.0) {
          beta1 = -beta1;
        }
        tau[b_i] = (beta1 - atmp) / beta1;
        bkj = 1.0 / (atmp - beta1);
        for (k = coffset; k <= i; k++) {
          M[k - 1] *= bkj;
        }
        for (k = 0; k < knt; k++) {
          beta1 *= 1.0020841800044864E-292;
        }
        atmp = beta1;
      } else {
        tau[b_i] = (beta1 - d) / beta1;
        bkj = 1.0 / (d - beta1);
        i = (ii - b_i) + 84;
        for (k = coffset; k <= i; k++) {
          M[k - 1] *= bkj;
        }
        atmp = beta1;
      }
    }
    M[ii] = atmp;
    if (b_i + 1 < 70) {
      int lastc;
      int lastv;
      M[ii] = 1.0;
      if (tau[b_i] != 0.0) {
        bool exitg2;
        lastv = 84 - b_i;
        coffset = (ii - b_i) + 83;
        while ((lastv > 0) && (M[coffset] == 0.0)) {
          lastv--;
          coffset--;
        }
        lastc = 68 - b_i;
        exitg2 = false;
        while ((!exitg2) && (lastc + 1 > 0)) {
          int exitg1;
          coffset = (ii + lastc * 84) + 84;
          k = coffset;
          do {
            exitg1 = 0;
            if (k + 1 <= coffset + lastv) {
              if (M[k] != 0.0) {
                exitg1 = 1;
              } else {
                k++;
              }
            } else {
              lastc--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);
          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        lastc = -1;
      }
      if (lastv > 0) {
        knt = ii + 85;
        if (lastc + 1 != 0) {
          std::memset(&work[0], 0,
                      static_cast<unsigned int>(lastc + 1) * sizeof(double));
          i = (ii + 84 * lastc) + 85;
          for (int j{knt}; j <= i; j += 84) {
            bkj = 0.0;
            i1 = (j + lastv) - 1;
            for (k = j; k <= i1; k++) {
              bkj += M[k - 1] * M[(ii + k) - j];
            }
            coffset = div_nde_s32_floor((j - ii) - 85, 84);
            work[coffset] += bkj;
          }
        }
        if (!(-tau[b_i] == 0.0)) {
          coffset = ii;
          for (int j{0}; j <= lastc; j++) {
            d = work[j];
            if (d != 0.0) {
              bkj = d * -tau[b_i];
              i = coffset + 85;
              i1 = lastv + coffset;
              for (knt = i; knt <= i1 + 84; knt++) {
                M[knt - 1] += M[((ii + knt) - coffset) - 85] * bkj;
              }
            }
            coffset += 84;
          }
        }
      }
      M[ii] = atmp;
    }
  }
  for (int j{0}; j < 70; j++) {
    for (int b_i{0}; b_i <= j; b_i++) {
      A[b_i + 70 * j] = M[b_i + 84 * j];
    }
    i = j + 2;
    if (i <= 70) {
      std::memset(&A[(j * 70 + i) + -1], 0,
                  static_cast<unsigned int>(-i + 71) * sizeof(double));
    }
  }
  for (i = 0; i < 70; i++) {
    for (i1 = 0; i1 < 70; i1++) {
      S[i1 + 70 * i] = A[i + 70 * i1];
    }
  }
}

} // namespace internal
} // namespace tracking
} // namespace matlabshared
} // namespace coder

// End of code generation (EKFCorrectorAdditive.cpp)
