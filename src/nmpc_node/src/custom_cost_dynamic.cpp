//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// custom_cost_dynamic.cpp
//
// Code generation for function 'custom_cost_dynamic'
//

// Include files
#include "custom_cost_dynamic.h"
#include "NMPC_Node_data.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions
double custom_cost_dynamic(const double X[112], const double U[16],
                           const double data_References[21])
{
  double x_ref[98];
  double J;
  double a;
  double b_index;
  double dist;
  double index0;
  //  Np = prediction horizon + 1
  std::memset(&x_ref[0], 0, 98U * sizeof(double));
  x_ref[0] = data_References[0];
  x_ref[7] = data_References[7];
  b_index = Index;
  for (int j{0}; j < 6; j++) {
    double a_tmp;
    double b_a_tmp;
    double b_min;
    int i;
    //  find the nearest points for the predicted states
    //  from reference data
    index0 = b_index;
    a_tmp = X[j + 1];
    a = a_tmp - ref_test[static_cast<int>(b_index) - 1];
    b_a_tmp = X[j + 9];
    dist = b_a_tmp - ref_test[static_cast<int>(b_index) + 21999];
    b_min = std::sqrt(a * a + dist * dist);
    i = static_cast<int>((static_cast<float>(b_index) + 100.0F) +
                         (1.0F - static_cast<float>(b_index)));
    for (int ii{0}; ii < i; ii++) {
      double b_ii;
      b_ii = index0 + static_cast<double>(ii);
      a = a_tmp - ref_test[static_cast<int>(b_ii) - 1];
      dist = b_a_tmp - ref_test[static_cast<int>(b_ii) + 21999];
      dist = std::sqrt(a * a + dist * dist);
      if (dist < b_min) {
        b_min = dist;
        b_index = b_ii;
      }
    }
    x_ref[j + 1] = ref_test[static_cast<int>(b_index) - 1];
    x_ref[j + 8] = ref_test[static_cast<int>(b_index) + 21999];
  }
  // state references:
  // state reference weights
  // q=zeros(1,13);
  // Q=diag(q);
  // input weights
  // input ref
  // outputs
  // v_in=sqrt(X(:,4).^2+X(:,5).^2+X(:,6).^2);
  // output references
  // output weight
  J = 0.0;
  for (int j{0}; j < 7; j++) {
    // calculate distance between the predicted states and reference points
    a = X[j + 1] - x_ref[j];
    dist = X[j + 9] - x_ref[j + 7];
    a = std::sqrt(a * a + dist * dist);
    // distance
    // cost from the distance
    // cost_X=(X(i+1,:)-x_ref(i,:))*Q*(X(i+1,:)-x_ref(i,:))'; %cost from the
    // states
    dist = U[j + 1];
    b_index = U[j + 9];
    // cost from the inputs
    // cost from the outputs
    index0 = X[j + 25] - 1.0;
    J += (20.0 * (a * a) +
          ((dist + b_index * 0.0) * dist + (dist * 0.0 + b_index) * b_index)) +
         index0 * 10.0 * index0;
    // *Weight;
  }
  return J;
}

// End of code generation (custom_cost_dynamic.cpp)
