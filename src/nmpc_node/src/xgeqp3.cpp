//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xgeqp3.cpp
//
// Code generation for function 'xgeqp3'
//

// Include files
#include "xgeqp3.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include "xzgeqp3.h"
#include "xzlarf.h"
#include "xzlarfg.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace internal {
namespace lapack {
void xgeqp3(::coder::array<double, 2U> &A, int b_m, int n,
            ::coder::array<int, 1U> &jpvt, ::coder::array<double, 1U> &tau)
{
  array<double, 1U> vn1;
  array<double, 1U> vn2;
  array<double, 1U> work;
  double temp;
  int i;
  int ij;
  int ma;
  int minmana;
  int minmn_tmp;
  ma = A.size(0);
  ij = A.size(0);
  minmana = A.size(1);
  if (ij <= minmana) {
    minmana = ij;
  }
  if (b_m <= n) {
    minmn_tmp = b_m;
  } else {
    minmn_tmp = n;
  }
  tau.set_size(minmana);
  for (i = 0; i < minmana; i++) {
    tau[i] = 0.0;
  }
  if ((A.size(0) == 0) || (A.size(1) == 0) || (minmn_tmp < 1)) {
    for (int ii{0}; ii < n; ii++) {
      jpvt[ii] = ii + 1;
    }
  } else {
    int ii;
    int ix;
    int nfxd;
    int temp_tmp;
    nfxd = 0;
    for (ii = 0; ii < n; ii++) {
      if (jpvt[ii] != 0) {
        nfxd++;
        if (ii + 1 != nfxd) {
          ix = ii * ma;
          minmana = (nfxd - 1) * ma;
          for (int k{0}; k < b_m; k++) {
            temp_tmp = ix + k;
            temp = A[temp_tmp];
            i = minmana + k;
            A[temp_tmp] = A[i];
            A[i] = temp;
          }
          jpvt[ii] = jpvt[nfxd - 1];
          jpvt[nfxd - 1] = ii + 1;
        } else {
          jpvt[ii] = ii + 1;
        }
      } else {
        jpvt[ii] = ii + 1;
      }
    }
    if (nfxd > minmn_tmp) {
      nfxd = minmn_tmp;
    }
    reflapack::qrf(A, b_m, n, nfxd, tau);
    if (nfxd < minmn_tmp) {
      double d;
      ma = A.size(0);
      work.set_size(A.size(1));
      ij = A.size(1);
      vn1.set_size(A.size(1));
      vn2.set_size(A.size(1));
      for (i = 0; i < ij; i++) {
        work[i] = 0.0;
        vn1[i] = 0.0;
        vn2[i] = 0.0;
      }
      i = nfxd + 1;
      for (ii = i; ii <= n; ii++) {
        d = blas::xnrm2(b_m - nfxd, A, (nfxd + (ii - 1) * ma) + 1);
        vn1[ii - 1] = d;
        vn2[ii - 1] = d;
      }
      for (int b_i{i}; b_i <= minmn_tmp; b_i++) {
        double s;
        int ip1;
        int mmi;
        int nmi;
        ip1 = b_i + 1;
        ij = (b_i - 1) * ma;
        ii = (ij + b_i) - 1;
        nmi = (n - b_i) + 1;
        mmi = b_m - b_i;
        if (nmi < 1) {
          minmana = -2;
        } else {
          minmana = -1;
          if (nmi > 1) {
            temp = std::abs(vn1[b_i - 1]);
            for (int k{2}; k <= nmi; k++) {
              s = std::abs(vn1[(b_i + k) - 2]);
              if (s > temp) {
                minmana = k - 2;
                temp = s;
              }
            }
          }
        }
        nfxd = b_i + minmana;
        if (nfxd + 1 != b_i) {
          ix = nfxd * ma;
          for (int k{0}; k < b_m; k++) {
            temp_tmp = ix + k;
            temp = A[temp_tmp];
            minmana = ij + k;
            A[temp_tmp] = A[minmana];
            A[minmana] = temp;
          }
          minmana = jpvt[nfxd];
          jpvt[nfxd] = jpvt[b_i - 1];
          jpvt[b_i - 1] = minmana;
          vn1[nfxd] = vn1[b_i - 1];
          vn2[nfxd] = vn2[b_i - 1];
        }
        if (b_i < b_m) {
          temp = A[ii];
          d = reflapack::xzlarfg(mmi + 1, temp, A, ii + 2);
          tau[b_i - 1] = d;
          A[ii] = temp;
        } else {
          d = 0.0;
          tau[b_i - 1] = 0.0;
        }
        if (b_i < n) {
          temp = A[ii];
          A[ii] = 1.0;
          reflapack::xzlarf(mmi + 1, nmi - 1, ii + 1, d, A, (ii + ma) + 1, ma,
                            work);
          A[ii] = temp;
        }
        for (ii = ip1; ii <= n; ii++) {
          ij = b_i + (ii - 1) * ma;
          d = vn1[ii - 1];
          if (d != 0.0) {
            temp = std::abs(A[ij - 1]) / d;
            temp = 1.0 - temp * temp;
            if (temp < 0.0) {
              temp = 0.0;
            }
            s = d / vn2[ii - 1];
            s = temp * (s * s);
            if (s <= 1.4901161193847656E-8) {
              if (b_i < b_m) {
                d = blas::xnrm2(mmi, A, ij + 1);
                vn1[ii - 1] = d;
                vn2[ii - 1] = d;
              } else {
                vn1[ii - 1] = 0.0;
                vn2[ii - 1] = 0.0;
              }
            } else {
              vn1[ii - 1] = d * std::sqrt(temp);
            }
          }
        }
      }
    }
  }
}

} // namespace lapack
} // namespace internal
} // namespace coder

// End of code generation (xgeqp3.cpp)
