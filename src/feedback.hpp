#ifndef FEEDBACK_HPP
#define FEEDBACK_HPP

#include "Message.hpp"
#include "cstdint"

typedef struct FeedbackType {
  // feedback
  bool timing;
  float rps;
  int count;
} feedback_t;

//  create message
typedef sb::Message<feedback_t> Feedback;

#endif