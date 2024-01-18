//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// EKFCorrectorAdditive.h
//
// Code generation for function 'EKFCorrectorAdditive'
//

#ifndef EKFCORRECTORADDITIVE_H
#define EKFCORRECTORADDITIVE_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace coder {
namespace matlabshared {
namespace tracking {
namespace internal {
class EKFCorrectorAdditive {
public:
  static void correctStateAndSqrtCovariance(double x[70], double S[4900],
                                            const double residue[14],
                                            const double Pxy[980],
                                            const double Sy[196],
                                            const double H[980],
                                            const double Rsqrt[196]);
};

} // namespace internal
} // namespace tracking
} // namespace matlabshared
} // namespace coder

#endif
// End of code generation (EKFCorrectorAdditive.h)
