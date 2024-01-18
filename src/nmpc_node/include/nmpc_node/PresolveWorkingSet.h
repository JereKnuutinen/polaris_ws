//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// PresolveWorkingSet.h
//
// Code generation for function 'PresolveWorkingSet'
//

#ifndef PRESOLVEWORKINGSET_H
#define PRESOLVEWORKINGSET_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct d_struct_T;

struct h_struct_T;

struct i_struct_T;

struct e_struct_T;

struct n_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace initialize {
void PresolveWorkingSet(d_struct_T &solution, h_struct_T &memspace,
                        i_struct_T &workingset, e_struct_T &qrmanager,
                        const n_struct_T &options);

}
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (PresolveWorkingSet.h)
