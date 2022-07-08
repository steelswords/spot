#include "GPIOPin.hpp"
#include <iostream>
#include <chrono>
#include <thread>

// Pins are 168, 51, 77, 78

int main()
{
  GPIOPin pin1(51, GPIODirection::Out);
  pin1.setDebug(true);
  pin1.write(true);
  std::this_thread::sleep_for(std::chrono::seconds(2));
  pin1.write(false);
}
