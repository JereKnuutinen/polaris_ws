//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// svd.cpp
//
// Code generation for function 'svd'
//

// Include files
#include "svd.h"
#include "rt_nonfinite.h"
#include "xaxpy.h"
#include "xdotc.h"
#include "xnrm2.h"
#include "xrot.h"
#include "xrotg.h"
#include "xswap.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace internal {
void b_svd(const double A[196], double U[196], double s[14], double V[196])
{
  double b_A[196];
  double e[14];
  double work[14];
  double nrm;
  double rt;
  double snorm;
  double sqds;
  int b_m;
  int ii;
  int jj;
  int qjj;
  int qp1;
  int qp1jj;
  int qq;
  std::copy(&A[0], &A[196], &b_A[0]);
  std::memset(&s[0], 0, 14U * sizeof(double));
  std::memset(&e[0], 0, 14U * sizeof(double));
  std::memset(&work[0], 0, 14U * sizeof(double));
  std::memset(&U[0], 0, 196U * sizeof(double));
  std::memset(&V[0], 0, 196U * sizeof(double));
  for (int q{0}; q < 13; q++) {
    bool apply_transform;
    qp1 = q + 2;
    qp1jj = q + 14 * q;
    qq = qp1jj + 1;
    apply_transform = false;
    nrm = blas::c_xnrm2(14 - q, b_A, qp1jj + 1);
    if (nrm > 0.0) {
      apply_transform = true;
      if (b_A[qp1jj] < 0.0) {
        nrm = -nrm;
      }
      s[q] = nrm;
      if (std::abs(nrm) >= 1.0020841800044864E-292) {
        nrm = 1.0 / nrm;
        qjj = (qp1jj - q) + 14;
        for (int k{qq}; k <= qjj; k++) {
          b_A[k - 1] *= nrm;
        }
      } else {
        qjj = (qp1jj - q) + 14;
        for (int k{qq}; k <= qjj; k++) {
          b_A[k - 1] /= s[q];
        }
      }
      b_A[qp1jj]++;
      s[q] = -s[q];
    } else {
      s[q] = 0.0;
    }
    for (jj = qp1; jj < 15; jj++) {
      qjj = q + 14 * (jj - 1);
      if (apply_transform) {
        blas::b_xaxpy(
            14 - q,
            -(blas::b_xdotc(14 - q, b_A, qp1jj + 1, b_A, qjj + 1) / b_A[qp1jj]),
            qp1jj + 1, b_A, qjj + 1);
      }
      e[jj - 1] = b_A[qjj];
    }
    for (ii = q + 1; ii < 15; ii++) {
      qjj = (ii + 14 * q) - 1;
      U[qjj] = b_A[qjj];
    }
    if (q + 1 <= 12) {
      nrm = blas::d_xnrm2(13 - q, e, q + 2);
      if (nrm == 0.0) {
        e[q] = 0.0;
      } else {
        if (e[q + 1] < 0.0) {
          e[q] = -nrm;
        } else {
          e[q] = nrm;
        }
        nrm = e[q];
        if (std::abs(e[q]) >= 1.0020841800044864E-292) {
          nrm = 1.0 / e[q];
          for (int k{qp1}; k < 15; k++) {
            e[k - 1] *= nrm;
          }
        } else {
          for (int k{qp1}; k < 15; k++) {
            e[k - 1] /= nrm;
          }
        }
        e[q + 1]++;
        e[q] = -e[q];
        for (ii = qp1; ii < 15; ii++) {
          work[ii - 1] = 0.0;
        }
        for (jj = qp1; jj < 15; jj++) {
          blas::c_xaxpy(13 - q, e[jj - 1], b_A, (q + 14 * (jj - 1)) + 2, work,
                        q + 2);
        }
        for (jj = qp1; jj < 15; jj++) {
          blas::d_xaxpy(13 - q, -e[jj - 1] / e[q + 1], work, q + 2, b_A,
                        (q + 14 * (jj - 1)) + 2);
        }
      }
      for (ii = qp1; ii < 15; ii++) {
        V[(ii + 14 * q) - 1] = e[ii - 1];
      }
    }
  }
  b_m = 12;
  s[13] = b_A[195];
  e[12] = b_A[194];
  e[13] = 0.0;
  std::memset(&U[182], 0, 14U * sizeof(double));
  U[195] = 1.0;
  for (int q{12}; q >= 0; q--) {
    qp1 = q + 2;
    qq = q + 14 * q;
    if (s[q] != 0.0) {
      for (jj = qp1; jj < 15; jj++) {
        qjj = (q + 14 * (jj - 1)) + 1;
        blas::b_xaxpy(14 - q,
                      -(blas::b_xdotc(14 - q, U, qq + 1, U, qjj) / U[qq]),
                      qq + 1, U, qjj);
      }
      for (ii = q + 1; ii < 15; ii++) {
        qjj = (ii + 14 * q) - 1;
        U[qjj] = -U[qjj];
      }
      U[qq]++;
      for (ii = 0; ii < q; ii++) {
        U[ii + 14 * q] = 0.0;
      }
    } else {
      std::memset(&U[q * 14], 0, 14U * sizeof(double));
      U[qq] = 1.0;
    }
  }
  for (int q{13}; q >= 0; q--) {
    if ((q + 1 <= 12) && (e[q] != 0.0)) {
      qp1 = q + 2;
      qjj = (q + 14 * q) + 2;
      for (jj = qp1; jj < 15; jj++) {
        qp1jj = (q + 14 * (jj - 1)) + 2;
        blas::b_xaxpy(13 - q,
                      -(blas::b_xdotc(13 - q, V, qjj, V, qp1jj) / V[qjj - 1]),
                      qjj, V, qp1jj);
      }
    }
    std::memset(&V[q * 14], 0, 14U * sizeof(double));
    V[q + 14 * q] = 1.0;
  }
  qq = 0;
  snorm = 0.0;
  for (int q{0}; q < 14; q++) {
    nrm = s[q];
    if (nrm != 0.0) {
      rt = std::abs(nrm);
      nrm /= rt;
      s[q] = rt;
      if (q + 1 < 14) {
        e[q] /= nrm;
      }
      qp1jj = 14 * q;
      qjj = qp1jj + 14;
      for (int k{qp1jj + 1}; k <= qjj; k++) {
        U[k - 1] *= nrm;
      }
    }
    if (q + 1 < 14) {
      nrm = e[q];
      if (nrm != 0.0) {
        rt = std::abs(nrm);
        nrm = rt / nrm;
        e[q] = rt;
        s[q + 1] *= nrm;
        qp1jj = 14 * (q + 1);
        qjj = qp1jj + 14;
        for (int k{qp1jj + 1}; k <= qjj; k++) {
          V[k - 1] *= nrm;
        }
      }
    }
    snorm = std::fmax(snorm, std::fmax(std::abs(s[q]), std::abs(e[q])));
  }
  while ((b_m + 2 > 0) && (qq < 75)) {
    bool exitg1;
    jj = b_m + 1;
    ii = b_m + 1;
    exitg1 = false;
    while (!(exitg1 || (ii == 0))) {
      nrm = std::abs(e[ii - 1]);
      if ((nrm <=
           2.2204460492503131E-16 * (std::abs(s[ii - 1]) + std::abs(s[ii]))) ||
          (nrm <= 1.0020841800044864E-292) ||
          ((qq > 20) && (nrm <= 2.2204460492503131E-16 * snorm))) {
        e[ii - 1] = 0.0;
        exitg1 = true;
      } else {
        ii--;
      }
    }
    if (ii == b_m + 1) {
      qjj = 4;
    } else {
      qp1jj = b_m + 2;
      qjj = b_m + 2;
      exitg1 = false;
      while ((!exitg1) && (qjj >= ii)) {
        qp1jj = qjj;
        if (qjj == ii) {
          exitg1 = true;
        } else {
          nrm = 0.0;
          if (qjj < b_m + 2) {
            nrm = std::abs(e[qjj - 1]);
          }
          if (qjj > ii + 1) {
            nrm += std::abs(e[qjj - 2]);
          }
          rt = std::abs(s[qjj - 1]);
          if ((rt <= 2.2204460492503131E-16 * nrm) ||
              (rt <= 1.0020841800044864E-292)) {
            s[qjj - 1] = 0.0;
            exitg1 = true;
          } else {
            qjj--;
          }
        }
      }
      if (qp1jj == ii) {
        qjj = 3;
      } else if (qp1jj == b_m + 2) {
        qjj = 1;
      } else {
        qjj = 2;
        ii = qp1jj;
      }
    }
    switch (qjj) {
    case 1: {
      rt = e[b_m];
      e[b_m] = 0.0;
      for (int k{jj}; k >= ii + 1; k--) {
        double sm;
        sm = blas::xrotg(s[k - 1], rt, sqds);
        if (k > ii + 1) {
          double b;
          b = e[k - 2];
          rt = -sqds * b;
          e[k - 2] = b * sm;
        }
        blas::b_xrot(V, 14 * (k - 1) + 1, 14 * (b_m + 1) + 1, sm, sqds);
      }
    } break;
    case 2: {
      rt = e[ii - 1];
      e[ii - 1] = 0.0;
      for (int k{ii + 1}; k <= b_m + 2; k++) {
        double b;
        double sm;
        sm = blas::xrotg(s[k - 1], rt, sqds);
        b = e[k - 1];
        rt = -sqds * b;
        e[k - 1] = b * sm;
        blas::b_xrot(U, 14 * (k - 1) + 1, 14 * (ii - 1) + 1, sm, sqds);
      }
    } break;
    case 3: {
      double b;
      double scale;
      double sm;
      nrm = s[b_m + 1];
      scale = std::fmax(
          std::fmax(std::fmax(std::fmax(std::abs(nrm), std::abs(s[b_m])),
                              std::abs(e[b_m])),
                    std::abs(s[ii])),
          std::abs(e[ii]));
      sm = nrm / scale;
      nrm = s[b_m] / scale;
      rt = e[b_m] / scale;
      sqds = s[ii] / scale;
      b = ((nrm + sm) * (nrm - sm) + rt * rt) / 2.0;
      nrm = sm * rt;
      nrm *= nrm;
      if ((b != 0.0) || (nrm != 0.0)) {
        rt = std::sqrt(b * b + nrm);
        if (b < 0.0) {
          rt = -rt;
        }
        rt = nrm / (b + rt);
      } else {
        rt = 0.0;
      }
      rt += (sqds + sm) * (sqds - sm);
      nrm = sqds * (e[ii] / scale);
      for (int k{ii + 1}; k <= jj; k++) {
        sm = blas::xrotg(rt, nrm, sqds);
        if (k > ii + 1) {
          e[k - 2] = rt;
        }
        nrm = e[k - 1];
        b = s[k - 1];
        e[k - 1] = sm * nrm - sqds * b;
        rt = sqds * s[k];
        s[k] *= sm;
        qjj = 14 * (k - 1) + 1;
        qp1jj = 14 * k + 1;
        blas::b_xrot(V, qjj, qp1jj, sm, sqds);
        s[k - 1] = sm * b + sqds * nrm;
        sm = blas::xrotg(s[k - 1], rt, sqds);
        b = e[k - 1];
        rt = sm * b + sqds * s[k];
        s[k] = -sqds * b + sm * s[k];
        nrm = sqds * e[k];
        e[k] *= sm;
        blas::b_xrot(U, qjj, qp1jj, sm, sqds);
      }
      e[b_m] = rt;
      qq++;
    } break;
    default:
      if (s[ii] < 0.0) {
        s[ii] = -s[ii];
        qp1jj = 14 * ii;
        qjj = qp1jj + 14;
        for (int k{qp1jj + 1}; k <= qjj; k++) {
          V[k - 1] = -V[k - 1];
        }
      }
      qp1 = ii + 1;
      while ((ii + 1 < 14) && (s[ii] < s[qp1])) {
        rt = s[ii];
        s[ii] = s[qp1];
        s[qp1] = rt;
        qjj = 14 * ii + 1;
        qp1jj = 14 * (ii + 1) + 1;
        blas::b_xswap(V, qjj, qp1jj);
        blas::b_xswap(U, qjj, qp1jj);
        ii = qp1;
        qp1++;
      }
      qq = 0;
      b_m--;
      break;
    }
  }
}

void svd(const double A[4900], double U[4900], double s[70], double V[4900])
{
  double b_A[4900];
  double e[70];
  double work[70];
  double nrm;
  double rt;
  double snorm;
  double sqds;
  int b_m;
  int ii;
  int jj;
  int qjj;
  int qp1;
  int qp1jj;
  int qq;
  std::copy(&A[0], &A[4900], &b_A[0]);
  std::memset(&s[0], 0, 70U * sizeof(double));
  std::memset(&e[0], 0, 70U * sizeof(double));
  std::memset(&work[0], 0, 70U * sizeof(double));
  std::memset(&U[0], 0, 4900U * sizeof(double));
  std::memset(&V[0], 0, 4900U * sizeof(double));
  for (int q{0}; q < 69; q++) {
    bool apply_transform;
    qp1 = q + 2;
    qp1jj = q + 70 * q;
    qq = qp1jj + 1;
    apply_transform = false;
    nrm = blas::xnrm2(70 - q, b_A, qp1jj + 1);
    if (nrm > 0.0) {
      apply_transform = true;
      if (b_A[qp1jj] < 0.0) {
        nrm = -nrm;
      }
      s[q] = nrm;
      if (std::abs(nrm) >= 1.0020841800044864E-292) {
        nrm = 1.0 / nrm;
        qjj = (qp1jj - q) + 70;
        for (int k{qq}; k <= qjj; k++) {
          b_A[k - 1] *= nrm;
        }
      } else {
        qjj = (qp1jj - q) + 70;
        for (int k{qq}; k <= qjj; k++) {
          b_A[k - 1] /= s[q];
        }
      }
      b_A[qp1jj]++;
      s[q] = -s[q];
    } else {
      s[q] = 0.0;
    }
    for (jj = qp1; jj < 71; jj++) {
      qjj = q + 70 * (jj - 1);
      if (apply_transform) {
        blas::xaxpy(
            70 - q,
            -(blas::xdotc(70 - q, b_A, qp1jj + 1, b_A, qjj + 1) / b_A[qp1jj]),
            qp1jj + 1, b_A, qjj + 1);
      }
      e[jj - 1] = b_A[qjj];
    }
    for (ii = q + 1; ii < 71; ii++) {
      qjj = (ii + 70 * q) - 1;
      U[qjj] = b_A[qjj];
    }
    if (q + 1 <= 68) {
      nrm = blas::b_xnrm2(69 - q, e, q + 2);
      if (nrm == 0.0) {
        e[q] = 0.0;
      } else {
        if (e[q + 1] < 0.0) {
          e[q] = -nrm;
        } else {
          e[q] = nrm;
        }
        nrm = e[q];
        if (std::abs(e[q]) >= 1.0020841800044864E-292) {
          nrm = 1.0 / e[q];
          for (int k{qp1}; k < 71; k++) {
            e[k - 1] *= nrm;
          }
        } else {
          for (int k{qp1}; k < 71; k++) {
            e[k - 1] /= nrm;
          }
        }
        e[q + 1]++;
        e[q] = -e[q];
        for (ii = qp1; ii < 71; ii++) {
          work[ii - 1] = 0.0;
        }
        for (jj = qp1; jj < 71; jj++) {
          blas::xaxpy(69 - q, e[jj - 1], b_A, (q + 70 * (jj - 1)) + 2, work,
                      q + 2);
        }
        for (jj = qp1; jj < 71; jj++) {
          blas::b_xaxpy(69 - q, -e[jj - 1] / e[q + 1], work, q + 2, b_A,
                        (q + 70 * (jj - 1)) + 2);
        }
      }
      for (ii = qp1; ii < 71; ii++) {
        V[(ii + 70 * q) - 1] = e[ii - 1];
      }
    }
  }
  b_m = 68;
  s[69] = b_A[4899];
  e[68] = b_A[4898];
  e[69] = 0.0;
  std::memset(&U[4830], 0, 70U * sizeof(double));
  U[4899] = 1.0;
  for (int q{68}; q >= 0; q--) {
    qp1 = q + 2;
    qq = q + 70 * q;
    if (s[q] != 0.0) {
      for (jj = qp1; jj < 71; jj++) {
        qjj = (q + 70 * (jj - 1)) + 1;
        blas::xaxpy(70 - q, -(blas::xdotc(70 - q, U, qq + 1, U, qjj) / U[qq]),
                    qq + 1, U, qjj);
      }
      for (ii = q + 1; ii < 71; ii++) {
        qjj = (ii + 70 * q) - 1;
        U[qjj] = -U[qjj];
      }
      U[qq]++;
      for (ii = 0; ii < q; ii++) {
        U[ii + 70 * q] = 0.0;
      }
    } else {
      std::memset(&U[q * 70], 0, 70U * sizeof(double));
      U[qq] = 1.0;
    }
  }
  for (int q{69}; q >= 0; q--) {
    if ((q + 1 <= 68) && (e[q] != 0.0)) {
      qp1 = q + 2;
      qjj = (q + 70 * q) + 2;
      for (jj = qp1; jj < 71; jj++) {
        qp1jj = (q + 70 * (jj - 1)) + 2;
        blas::xaxpy(69 - q,
                    -(blas::xdotc(69 - q, V, qjj, V, qp1jj) / V[qjj - 1]), qjj,
                    V, qp1jj);
      }
    }
    std::memset(&V[q * 70], 0, 70U * sizeof(double));
    V[q + 70 * q] = 1.0;
  }
  qq = 0;
  snorm = 0.0;
  for (int q{0}; q < 70; q++) {
    nrm = s[q];
    if (nrm != 0.0) {
      rt = std::abs(nrm);
      nrm /= rt;
      s[q] = rt;
      if (q + 1 < 70) {
        e[q] /= nrm;
      }
      qp1jj = 70 * q;
      qjj = qp1jj + 70;
      for (int k{qp1jj + 1}; k <= qjj; k++) {
        U[k - 1] *= nrm;
      }
    }
    if (q + 1 < 70) {
      nrm = e[q];
      if (nrm != 0.0) {
        rt = std::abs(nrm);
        nrm = rt / nrm;
        e[q] = rt;
        s[q + 1] *= nrm;
        qp1jj = 70 * (q + 1);
        qjj = qp1jj + 70;
        for (int k{qp1jj + 1}; k <= qjj; k++) {
          V[k - 1] *= nrm;
        }
      }
    }
    snorm = std::fmax(snorm, std::fmax(std::abs(s[q]), std::abs(e[q])));
  }
  while ((b_m + 2 > 0) && (qq < 75)) {
    bool exitg1;
    jj = b_m + 1;
    ii = b_m + 1;
    exitg1 = false;
    while (!(exitg1 || (ii == 0))) {
      nrm = std::abs(e[ii - 1]);
      if ((nrm <=
           2.2204460492503131E-16 * (std::abs(s[ii - 1]) + std::abs(s[ii]))) ||
          (nrm <= 1.0020841800044864E-292) ||
          ((qq > 20) && (nrm <= 2.2204460492503131E-16 * snorm))) {
        e[ii - 1] = 0.0;
        exitg1 = true;
      } else {
        ii--;
      }
    }
    if (ii == b_m + 1) {
      qjj = 4;
    } else {
      qp1jj = b_m + 2;
      qjj = b_m + 2;
      exitg1 = false;
      while ((!exitg1) && (qjj >= ii)) {
        qp1jj = qjj;
        if (qjj == ii) {
          exitg1 = true;
        } else {
          nrm = 0.0;
          if (qjj < b_m + 2) {
            nrm = std::abs(e[qjj - 1]);
          }
          if (qjj > ii + 1) {
            nrm += std::abs(e[qjj - 2]);
          }
          rt = std::abs(s[qjj - 1]);
          if ((rt <= 2.2204460492503131E-16 * nrm) ||
              (rt <= 1.0020841800044864E-292)) {
            s[qjj - 1] = 0.0;
            exitg1 = true;
          } else {
            qjj--;
          }
        }
      }
      if (qp1jj == ii) {
        qjj = 3;
      } else if (qp1jj == b_m + 2) {
        qjj = 1;
      } else {
        qjj = 2;
        ii = qp1jj;
      }
    }
    switch (qjj) {
    case 1: {
      rt = e[b_m];
      e[b_m] = 0.0;
      for (int k{jj}; k >= ii + 1; k--) {
        double sm;
        sm = blas::xrotg(s[k - 1], rt, sqds);
        if (k > ii + 1) {
          double b;
          b = e[k - 2];
          rt = -sqds * b;
          e[k - 2] = b * sm;
        }
        blas::xrot(V, 70 * (k - 1) + 1, 70 * (b_m + 1) + 1, sm, sqds);
      }
    } break;
    case 2: {
      rt = e[ii - 1];
      e[ii - 1] = 0.0;
      for (int k{ii + 1}; k <= b_m + 2; k++) {
        double b;
        double sm;
        sm = blas::xrotg(s[k - 1], rt, sqds);
        b = e[k - 1];
        rt = -sqds * b;
        e[k - 1] = b * sm;
        blas::xrot(U, 70 * (k - 1) + 1, 70 * (ii - 1) + 1, sm, sqds);
      }
    } break;
    case 3: {
      double b;
      double scale;
      double sm;
      nrm = s[b_m + 1];
      scale = std::fmax(
          std::fmax(std::fmax(std::fmax(std::abs(nrm), std::abs(s[b_m])),
                              std::abs(e[b_m])),
                    std::abs(s[ii])),
          std::abs(e[ii]));
      sm = nrm / scale;
      nrm = s[b_m] / scale;
      rt = e[b_m] / scale;
      sqds = s[ii] / scale;
      b = ((nrm + sm) * (nrm - sm) + rt * rt) / 2.0;
      nrm = sm * rt;
      nrm *= nrm;
      if ((b != 0.0) || (nrm != 0.0)) {
        rt = std::sqrt(b * b + nrm);
        if (b < 0.0) {
          rt = -rt;
        }
        rt = nrm / (b + rt);
      } else {
        rt = 0.0;
      }
      rt += (sqds + sm) * (sqds - sm);
      nrm = sqds * (e[ii] / scale);
      for (int k{ii + 1}; k <= jj; k++) {
        sm = blas::xrotg(rt, nrm, sqds);
        if (k > ii + 1) {
          e[k - 2] = rt;
        }
        nrm = e[k - 1];
        b = s[k - 1];
        e[k - 1] = sm * nrm - sqds * b;
        rt = sqds * s[k];
        s[k] *= sm;
        qjj = 70 * (k - 1) + 1;
        qp1jj = 70 * k + 1;
        blas::xrot(V, qjj, qp1jj, sm, sqds);
        s[k - 1] = sm * b + sqds * nrm;
        sm = blas::xrotg(s[k - 1], rt, sqds);
        b = e[k - 1];
        rt = sm * b + sqds * s[k];
        s[k] = -sqds * b + sm * s[k];
        nrm = sqds * e[k];
        e[k] *= sm;
        blas::xrot(U, qjj, qp1jj, sm, sqds);
      }
      e[b_m] = rt;
      qq++;
    } break;
    default:
      if (s[ii] < 0.0) {
        s[ii] = -s[ii];
        qp1jj = 70 * ii;
        qjj = qp1jj + 70;
        for (int k{qp1jj + 1}; k <= qjj; k++) {
          V[k - 1] = -V[k - 1];
        }
      }
      qp1 = ii + 1;
      while ((ii + 1 < 70) && (s[ii] < s[qp1])) {
        rt = s[ii];
        s[ii] = s[qp1];
        s[qp1] = rt;
        qjj = 70 * ii + 1;
        qp1jj = 70 * (ii + 1) + 1;
        blas::xswap(V, qjj, qp1jj);
        blas::xswap(U, qjj, qp1jj);
        ii = qp1;
        qp1++;
      }
      qq = 0;
      b_m--;
      break;
    }
  }
}

} // namespace internal
} // namespace coder

// End of code generation (svd.cpp)
