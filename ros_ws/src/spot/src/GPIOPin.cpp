#include "GPIOPin.hpp"
#include <fstream>
#include <iostream>

GPIOPin::GPIOPin(int pin, GPIODirection direction)
  : pin(pin), direction(direction)
{
  exportPin();
  setPinDirection(direction);
  openPinFile();
}

GPIOPin::~GPIOPin()
{
  pinFile->close();
}

void GPIOPin::deregister()
{
  unexportPin();
}

void GPIOPin::exportPin()
{
  std::string operationMessage = "export GPIO pin " + std::to_string(pin);
  doPinOperation(getExportPinFilename(), std::to_string(pin), operationMessage);
}

void GPIOPin::doPinOperation(const std::string &filename,
    const std::string writeValue, const std::string &operationMessage)
{
  std::ofstream file;
  file.open(filename.c_str());
  if (!file)
  {
    std::cerr << "[ERROR] Could not open " << filename << " for writing. "
      "Unable to " << operationMessage << "!" << std::endl;
    return;
  }
  file << writeValue;
  file.close();
}

void GPIOPin::unexportPin()
{
  std::string operationMessage = "unexport GPIO pin " + std::to_string(pin);
  doPinOperation(getUnexportPinFilename(), std::to_string(pin), operationMessage);
}

void GPIOPin::setPinDirection(GPIODirection direction)
{
  std::string operationMessage = "set GPIO pin " + pinAsString() + " to have "
    "direction " + directionAsString(direction);
  doPinOperation(getPinDirectionFilename(), directionAsString(direction), operationMessage);
}

std::string GPIOPin::directionAsString(GPIODirection direction)
{
  return direction == GPIODirection::In ? "in" : "out";
}

void GPIOPin::openPinFile()
{
  pinFile = new std::fstream(getPinFilename().c_str());
}

void GPIOPin::write(bool value)
{
  if (!pinFile)
  {
    std::cerr << "[WARNING] Tried to write to GPIO pin " << pin << " without an "
      "open file handle. Attempting to fix..." << std::endl;
    openPinFile();
    if (!pinFile)
    {
      std::cerr << "[ERROR] Could not open file handler for GPIO pin " << pin << "!"
        << "Exiting write() function!!" << std::endl;
      return;
    }
  }
  int valueAsInt = static_cast<int>(value);
  *pinFile << valueAsInt << std::flush;
  if (debuggingOn)
  {
    std::cout << "Wrote" << valueAsInt << " to pin " << pin << std::endl;
  }
}
