#ifndef NMPC_NODE__VISIBILITY_CONTROL_H_
#define NMPC_NODE__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define NMPC_NODE_EXPORT __attribute__ ((dllexport))
    #define NMPC_NODE_IMPORT __attribute__ ((dllimport))
  #else
    #define NMPC_NODE_EXPORT __declspec(dllexport)
    #define NMPC_NODE_IMPORT __declspec(dllimport)
  #endif
  #ifdef NMPC_NODE_BUILDING_LIBRARY
    #define NMPC_NODE_PUBLIC NMPC_NODE_EXPORT
  #else
    #define NMPC_NODE_PUBLIC NMPC_NODE_IMPORT
  #endif
  #define NMPC_NODE_PUBLIC_TYPE NMPC_NODE_PUBLIC
  #define NMPC_NODE_LOCAL
#else
  #define NMPC_NODE_EXPORT __attribute__ ((visibility("default")))
  #define NMPC_NODE_IMPORT
  #if __GNUC__ >= 4
    #define NMPC_NODE_PUBLIC __attribute__ ((visibility("default")))
    #define NMPC_NODE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define NMPC_NODE_PUBLIC
    #define NMPC_NODE_LOCAL
  #endif
  #define NMPC_NODE_PUBLIC_TYPE
#endif
#endif  // NMPC_NODE__VISIBILITY_CONTROL_H_
// Generated 31-Jul-2023 12:53:04
// Copyright 2019-2020 The MathWorks, Inc.
