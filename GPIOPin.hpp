#pragma once
#include <string>
#include <fstream>

enum GPIODirection
{
  In,
  Out
};

class GPIOPin
{
public:
  GPIOPin(int pin, GPIODirection direction);
  ~GPIOPin();
  void write(bool value);
  void setDebug(bool shouldDebug) { debuggingOn = shouldDebug; }
  std::string pinAsString() { return std::to_string(pin); }
  // Unexports the GPIO pin. Not called by default on decosntructing
  void deregister();
private:
  void exportPin();
  void unexportPin();
  void openPinFile();

  // Many operations done in this library follow a very similar pattern of
  // opening a file, checking if it had errors, displaying a message if it did,
  // and then writing a value to the file before closing it. This function
  // abstracts all that away.
  void doPinOperation(const std::string &filename, const std::string writeValue,
      const std::string &operationMessage);

  void setPinDirection(GPIODirection direction);

  std::fstream *pinFile {nullptr};
  /** @brief The Linux GPIO pin number */
  int pin;
  GPIODirection direction;
  bool debuggingOn {false};
protected:
  // These methods and members can be overwritten by derived classes to support
  // different directory structures.
  virtual std::string getExportPinFilename() { return "/sys/class/gpio/export"; }
  virtual std::string getUnexportPinFilename() { return "/sys/class/gpio/unexport"; }
  virtual std::string getPinBaseFilename() { return "/sys/class/gpio/gpio" + std::to_string(pin);}
  virtual std::string getPinFilename() { return getPinBaseFilename() + "/value"; }
  virtual std::string getPinDirectionFilename() { return getPinBaseFilename() + "/direction"; }

  virtual std::string directionAsString(GPIODirection direction);
};
