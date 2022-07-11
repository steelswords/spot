#include "SpotDriveTrain.hpp"

SpotDriveTrain::SpotDriveTrain(int rightForward, int rightBack, int leftForward, int leftBack)
{
  m_rightForwardPin = new GPIOPin(rightForward, GPIODirection::Out);
  m_leftForwardPin  = new GPIOPin(leftForward,  GPIODirection::Out);
  m_rightBackPin    = new GPIOPin(rightBack, GPIODirection::Out);
  m_leftBackPin     = new GPIOPin(leftBack,  GPIODirection::Out);
}

void SpotDriveTrain::goForward()
{
  m_rightForwardPin->write(true);
  m_leftForwardPin->write(true);
  m_leftBackPin->write(false);
  m_rightBackPin->write(false);
}

void SpotDriveTrain::goBack()
{
  m_rightForwardPin->write(false);
  m_leftForwardPin->write(false);
  m_leftBackPin->write(true);
  m_rightBackPin->write(true);
}

void SpotDriveTrain::stop()
{
  m_rightForwardPin->write(false);
  m_leftForwardPin->write(false);
  m_leftBackPin->write(false);
  m_rightBackPin->write(false);
}
void SpotDriveTrain::turnRight()
{
  m_rightForwardPin->write(false);
  m_leftForwardPin->write(true);
  m_leftBackPin->write(false);
  m_rightBackPin->write(true);
}
void SpotDriveTrain::turnLeft()
{
  m_rightForwardPin->write(true);
  m_leftForwardPin->write(false);
  m_leftBackPin->write(true);
  m_rightBackPin->write(false);
}
