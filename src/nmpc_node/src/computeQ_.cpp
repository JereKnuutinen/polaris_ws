//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeQ_.cpp
//
// Code generation for function 'computeQ_'
//

// Include files
#include "computeQ_.h"
#include "NMPC_Node_internal_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace QRManager {
void computeQ_(e_struct_T &obj, int nrows)
{
  ::coder::array<double, 2U> *A;
  array<double, 1U> work;
  int b_m;
  int i;
  int iQR0;
  int lastc;
  int lastv;
  int lda;
  int n;
  lastv = obj.minRowCol;
  for (int idx{0}; idx < lastv; idx++) {
    iQR0 = obj.ldq * idx + idx;
    n = obj.mrows - idx;
    for (lastc = 0; lastc <= n - 2; lastc++) {
      i = (iQR0 + lastc) + 1;
      obj.Q[i] = obj.QR[i];
    }
  }
  b_m = obj.mrows;
  lda = obj.ldq;
  if (nrows >= 1) {
    int ia;
    int itau;
    i = nrows - 1;
    for (n = lastv; n <= i; n++) {
      ia = n * lda;
      iQR0 = b_m - 1;
      for (int b_i{0}; b_i <= iQR0; b_i++) {
        obj.Q[ia + b_i] = 0.0;
      }
      obj.Q[ia + n] = 1.0;
    }
    unsigned int unnamed_idx_0;
    itau = obj.minRowCol - 1;
    unnamed_idx_0 = static_cast<unsigned int>(obj.Q.size(1));
    work.set_size(static_cast<int>(unnamed_idx_0));
    iQR0 = static_cast<int>(unnamed_idx_0);
    for (i = 0; i < iQR0; i++) {
      work[i] = 0.0;
    }
    for (int b_i{obj.minRowCol}; b_i >= 1; b_i--) {
      int iaii;
      iaii = b_i + (b_i - 1) * lda;
      if (b_i < nrows) {
        int jA;
        obj.Q[iaii - 1] = 1.0;
        jA = iaii + lda;
        if (obj.tau[itau] != 0.0) {
          bool exitg2;
          lastv = (b_m - b_i) + 1;
          iQR0 = (iaii + b_m) - b_i;
          while ((lastv > 0) && (obj.Q[iQR0 - 1] == 0.0)) {
            lastv--;
            iQR0--;
          }
          lastc = nrows - b_i;
          exitg2 = false;
          while ((!exitg2) && (lastc > 0)) {
            int exitg1;
            iQR0 = jA + (lastc - 1) * lda;
            ia = iQR0;
            do {
              exitg1 = 0;
              if (ia <= (iQR0 + lastv) - 1) {
                if (obj.Q[ia - 1] != 0.0) {
                  exitg1 = 1;
                } else {
                  ia++;
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
          lastc = 0;
        }
        if (lastv > 0) {
          double c;
          if (lastc != 0) {
            for (n = 0; n < lastc; n++) {
              work[n] = 0.0;
            }
            n = 0;
            i = jA + lda * (lastc - 1);
            for (int idx{jA}; lda < 0 ? idx >= i : idx <= i; idx += lda) {
              c = 0.0;
              iQR0 = (idx + lastv) - 1;
              for (ia = idx; ia <= iQR0; ia++) {
                c += obj.Q[ia - 1] * obj.Q[((iaii + ia) - idx) - 1];
              }
              work[n] = work[n] + c;
              n++;
            }
          }
          c = -obj.tau[itau];
          A = &obj.Q;
          if (!(c == 0.0)) {
            for (n = 0; n < lastc; n++) {
              if (work[n] != 0.0) {
                double temp;
                temp = work[n] * c;
                i = lastv + jA;
                for (iQR0 = jA; iQR0 < i; iQR0++) {
                  (*A)[iQR0 - 1] =
                      (*A)[iQR0 - 1] + (*A)[((iaii + iQR0) - jA) - 1] * temp;
                }
              }
              jA += lda;
            }
          }
        }
      }
      if (b_i < b_m) {
        iQR0 = iaii + 1;
        i = (iaii + b_m) - b_i;
        for (lastv = iQR0; lastv <= i; lastv++) {
          obj.Q[lastv - 1] = -obj.tau[itau] * obj.Q[lastv - 1];
        }
      }
      obj.Q[iaii - 1] = 1.0 - obj.tau[itau];
      for (n = 0; n <= b_i - 2; n++) {
        obj.Q[(iaii - n) - 2] = 0.0;
      }
      itau--;
    }
  }
}

} // namespace QRManager
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (computeQ_.cpp)
