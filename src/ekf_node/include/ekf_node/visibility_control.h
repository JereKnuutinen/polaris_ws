#ifndef EKF_NODE__VISIBILITY_CONTROL_H_
#define EKF_NODE__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define EKF_NODE_EXPORT __attribute__ ((dllexport))
    #define EKF_NODE_IMPORT __attribute__ ((dllimport))
  #else
    #define EKF_NODE_EXPORT __declspec(dllexport)
    #define EKF_NODE_IMPORT __declspec(dllimport)
  #endif
  #ifdef EKF_NODE_BUILDING_LIBRARY
    #define EKF_NODE_PUBLIC EKF_NODE_EXPORT
  #else
    #define EKF_NODE_PUBLIC EKF_NODE_IMPORT
  #endif
  #define EKF_NODE_PUBLIC_TYPE EKF_NODE_PUBLIC
  #define EKF_NODE_LOCAL
#else
  #define EKF_NODE_EXPORT __attribute__ ((visibility("default")))
  #define EKF_NODE_IMPORT
  #if __GNUC__ >= 4
    #define EKF_NODE_PUBLIC __attribute__ ((visibility("default")))
    #define EKF_NODE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define EKF_NODE_PUBLIC
    #define EKF_NODE_LOCAL
  #endif
  #define EKF_NODE_PUBLIC_TYPE
#endif
#endif  // EKF_NODE__VISIBILITY_CONTROL_H_
// Generated 18-Jan-2024 15:26:09
// Copyright 2019-2020 The MathWorks, Inc.
