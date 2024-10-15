#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "Message.hpp"
#include "cstdint"

typedef struct ControllerType {

  // axes
  float lx;
  float ly;
  float rx;
  float ry;
  float l2;
  float r2;

  // buttons
  bool cross;
  bool circle;
  bool triangle;
  bool square;
  bool r1;
} controller_t;

//  create message
typedef sb::Message<controller_t> Controller;

#endif