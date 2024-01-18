//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// Publisher.h
//
// Code generation for function 'Publisher'
//

#ifndef PUBLISHER_H
#define PUBLISHER_H

// Include files
#include "rtwtypes.h"
#include "mlroscpp_pub.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct geometry_msgs_AccelStruct_T;

// Type Definitions
namespace coder {
namespace ros {
class Publisher {
public:
  Publisher *init();
  static void rosmessage(geometry_msgs_AccelStruct_T &msgFromPub);
  char TopicName[9];
  double BufferSize;
  bool IsLatching;
  std::unique_ptr<
      MATLABPublisher<geometry_msgs::Accel, geometry_msgs_AccelStruct_T>>
      PublisherHelper;
};

class b_Publisher {
public:
  b_Publisher *init();
  char TopicName[14];
  double BufferSize;
  bool IsLatching;
  std::unique_ptr<
      MATLABPublisher<geometry_msgs::Accel, geometry_msgs_AccelStruct_T>>
      PublisherHelper;
};

} // namespace ros
} // namespace coder

#endif
// End of code generation (Publisher.h)
