#pragma once

extern float pressure;

class PressureSensor {
private:
  float pressureMaxLimit;
  float pressureMinLimit;
  bool gasValveIsOpen = false; // Argon gas valve is open or not

public:
  PressureSensor(float maxPressureLimit, float minPressureLimit);
  
  bool newCalibrationInfoAvailable;
  
  void loop();

  void logInfoCalibration();

  bool valveIsOpen();

  float currentPressure();
};