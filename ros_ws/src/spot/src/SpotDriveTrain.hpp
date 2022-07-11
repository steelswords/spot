#pragma once
#include "GPIOPin.hpp"

/** @class SpotDriveTrain
 * A class to abstract away the drive train on the Spot robot. Exposes simple
 * functions like goForward(), turnRight(), etc. */
class SpotDriveTrain
{
public:
  SpotDriveTrain(int rightForward, int rightBack, int leftForward, int leftBack);
  void goForward();
  void goBack();
  void stop();
  void turnRight();
  void turnLeft();
private:
  GPIOPin* m_rightForwardPin;
  GPIOPin* m_rightBackPin;
  GPIOPin* m_leftForwardPin;
  GPIOPin* m_leftBackPin;
};

