//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// driver1.h
//
// Code generation for function 'driver1'
//

#ifndef DRIVER1_H
#define DRIVER1_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct d_struct_T;

struct struct_T;

namespace coder {
namespace internal {
class i_stickyStruct;

}
} // namespace coder
struct o_struct_T;

struct h_struct_T;

struct i_struct_T;

struct e_struct_T;

struct f_struct_T;

struct g_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
void driver(const ::coder::array<double, 1U> &bineq, const double lb[113],
            const double ub[113], d_struct_T &b_TrialState,
            struct_T &b_MeritFunction,
            const ::coder::internal::i_stickyStruct &FcnEvaluator,
            const o_struct_T &FiniteDifferences, h_struct_T &memspace,
            i_struct_T &WorkingSet, e_struct_T &b_QRManager,
            f_struct_T &b_CholManager, g_struct_T &QPObjective,
            const ::coder::array<double, 1U> &fscales_lineq_constraint,
            const ::coder::array<double, 1U> &fscales_cineq_constraint,
            double Hessian[12769]);

}
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (driver1.h)
