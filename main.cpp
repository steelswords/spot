#include <iostream>
#include <functional>
#include <map>
#include <unistd.h>

#include "GPIOPin.hpp"
#include "SpotDriveTrain.hpp"

#define GPIO_PIN_RIGHT_FORWARD 168
#define GPIO_PIN_RIGHT_BACKWARD 78
#define GPIO_PIN_LEFT_FORWARD 51
#define GPIO_PIN_LEFT_BACKWARD 77

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
  std::cout << "Welcome to the CLI SPOT Navigation System" << std::endl;
  SpotDriveTrain* drive = new SpotDriveTrain(GPIO_PIN_RIGHT_FORWARD,
      GPIO_PIN_RIGHT_BACKWARD, GPIO_PIN_LEFT_FORWARD, GPIO_PIN_LEFT_BACKWARD);

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
