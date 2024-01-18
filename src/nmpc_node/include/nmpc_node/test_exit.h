//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// test_exit.h
//
// Code generation for function 'test_exit'
//

#ifndef TEST_EXIT_H
#define TEST_EXIT_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct h_struct_T;

struct struct_T;

struct i_struct_T;

struct d_struct_T;

struct e_struct_T;

struct b_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
void b_test_exit(b_struct_T &Flags, h_struct_T &memspace,
                 struct_T &b_MeritFunction,
                 const ::coder::array<double, 1U> &fscales_lineq_constraint,
                 const ::coder::array<double, 1U> &fscales_cineq_constraint,
                 i_struct_T &WorkingSet, d_struct_T &b_TrialState,
                 e_struct_T &b_QRManager, const double lb[113],
                 const double ub[113]);

bool test_exit(h_struct_T &memspace, struct_T &b_MeritFunction,
               const ::coder::array<double, 1U> &fscales_lineq_constraint,
               const ::coder::array<double, 1U> &fscales_cineq_constraint,
               i_struct_T &WorkingSet, d_struct_T &b_TrialState,
               e_struct_T &b_QRManager, const double lb[113],
               const double ub[113], bool &Flags_fevalOK, bool &Flags_done,
               bool &Flags_stepAccepted, bool &Flags_failedLineSearch,
               int &Flags_stepType);

} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (test_exit.h)
