//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// extendedKalmanFilter.h
//
// Code generation for function 'extendedKalmanFilter'
//

#ifndef EXTENDEDKALMANFILTER_H
#define EXTENDEDKALMANFILTER_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace coder {
class extendedKalmanFilter {
public:
  extendedKalmanFilter *init();
  void set_ProcessNoise(const double b_value[4900]);
  void set_MeasurementNoise(const double b_value[196]);
  void set_StateCovariance(const double b_value[4900]);
  void correct(const double z[14]);
  void get_StateCovariance(double b_value[4900]);
  void predict(const double varargin_1[3]);
  double pState[70];

protected:
  double pSqrtStateCovariance[4900];
  double pSqrtStateCovarianceScalar;
  bool pIsSetStateCovariance;
  double pSqrtProcessNoise[4900];
  double pSqrtProcessNoiseScalar;
  bool pIsSetProcessNoise;
  double pSqrtMeasurementNoise[196];
  double pSqrtMeasurementNoiseScalar;
  bool pIsSetMeasurementNoise;
  bool pIsValidStateTransitionFcn;
  bool pIsValidMeasurementFcn;
  bool pIsFirstCallPredict;
  bool pIsFirstCallCorrect;
};

} // namespace coder

#endif
// End of code generation (extendedKalmanFilter.h)
