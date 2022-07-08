#include <iostream>
#include <functional>
#include <map>
#include <unistd.h>

#include "GPIOPin.hpp"

#define NUM_DRIVE_MOTOR_PINS 4
class FidoDriveTrain
{
public:
  FidoDriveTrain(int rightForward, int rightBack, int leftForward, int leftBack);
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

FidoDriveTrain::FidoDriveTrain(int rightForward, int rightBack, int leftForward, int leftBack)
{
  m_rightForwardPin = new GPIOPin(rightForward, GPIODirection::Out);
  m_leftForwardPin  = new GPIOPin(leftForward,  GPIODirection::Out);
  m_rightBackPin    = new GPIOPin(rightBack, GPIODirection::Out);
  m_leftBackPin     = new GPIOPin(leftBack,  GPIODirection::Out);
}

void FidoDriveTrain::goForward()
{
  m_rightForwardPin->write(true);
  m_leftForwardPin->write(true);
  m_leftBackPin->write(false);
  m_rightBackPin->write(false);
}

void FidoDriveTrain::goBack()
{
  m_rightForwardPin->write(false);
  m_leftForwardPin->write(false);
  m_leftBackPin->write(true);
  m_rightBackPin->write(true);
}

void FidoDriveTrain::stop()
{
  m_rightForwardPin->write(false);
  m_leftForwardPin->write(false);
  m_leftBackPin->write(false);
  m_rightBackPin->write(false);
}
void FidoDriveTrain::turnRight()
{
  m_rightForwardPin->write(false);
  m_leftForwardPin->write(true);
  m_leftBackPin->write(false);
  m_rightBackPin->write(true);
}
void FidoDriveTrain::turnLeft()
{
  m_rightForwardPin->write(true);
  m_leftForwardPin->write(false);
  m_leftBackPin->write(true);
  m_rightBackPin->write(false);
}

void printMenu()
{
  std::cout << "w: Forward\n"
    << "s: Backward\n"
    << "a: Left\n"
    << "d: Right\n"
    << "x: Stop\n"
    << "e: Exit\n"
    << "=========================\n";
}

int main(int argc, char * argv[])
{
  std::cout << "Welcome to FIDO" << std::endl;
	//GPIOPin(string pin, GPIODirection direction); //Creates GPIOPin that controls GPIO{pin}, and 
  //FidoDriveTrain* drive = new FidoDriveTrain("11", "13", "15", "16");
  FidoDriveTrain* drive = new FidoDriveTrain(168, 78, 51, 77);
#if 0
  GPIOPin* rWheelForward = new GPIOPin("2", GPIODirection::Out);
  GPIOPin* rWheelBack = new GPIOPin("14", GPIODirection::Out);
  GPIOPin* lWheelForward = new GPIOPin("18", GPIODirection::Out);
  GPIOPin* lWheelBack = new GPIOPin("15", GPIODirection::Out);
  rWheelBack->setval_gpio("0");
  rWheelForward->setval_gpio("0");
  lWheelForward->setval_gpio("0");
  lWheelBack->setval_gpio("1");
  sleep(3);

  rWheelBack->setval_gpio("0");
  rWheelForward->setval_gpio("0");
  lWheelForward->setval_gpio("0");
  lWheelBack->setval_gpio("0");
#endif 
#if 0
  string stepperPins[NUM_STEPPER_MOTOR_PINS] = {"26", "19", "13", "6"};
  StepperMotor* stepper = new StepperMotor(stepperPins); 
  stepper->move(20.0f, false);
#endif
#if 0
  std::cout << "Going forward...\n";
  drive->goForward();
  sleep(4);
  std::cout << "Stopping.\n";
  drive->stop();
  sleep(2);
  std::cout << "Turn to the right\n";
  drive->turnRight();
  sleep(1);
  std::cout << "Going forward...\n";
  drive->goForward();
  sleep(3);
  std::cout << "Turn to the right\n";
  drive->turnLeft();
  sleep(2);
  std::cout << "Stopping.\n";
  drive->stop();
#endif

  bool running = true;
  std::map<std::string, std::function<void(void)>> menuOptions
  {
    {"w", [&drive](){drive->goForward();}},
    {"s", [&drive](){drive->goBack();}},
    {"a", [&drive](){drive->turnLeft();}},
    {"d", [&drive](){drive->turnRight();}},
    {"x", [&drive](){drive->stop();}},
    {"e", [&running](){running = false;}}
  };

  while (running)
  {
    printMenu();
    std::cout << "Enter command: ";
    std::string input;
    std::cin >> input;
    if (menuOptions.find(input) == menuOptions.end())
    {
      std::cout << "Invalid entry. Try again.\n";
    } 
    else
    {
      menuOptions[input]();
    }
    std::cout << "--------------------------------" << std::endl;
  }
  return 0;
}
