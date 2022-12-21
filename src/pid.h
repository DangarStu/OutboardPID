#ifndef _PID_H_
#define _PID_H_

#include "sensesp/transforms/transform.h"

#define DIRECT     0
#define REVERSE    1
 
#define MANUAL     false
#define AUTOMATIC  true

namespace sensesp {

/**
 * @brief Acts as a basic PID controller. A PID controller takes an input and
 * produces and output that attempts to bring the input back to the set point
 * as efficiently as possible. As an example (the reason I wrote this class):
 * an engine has a coolant temperature measurement from a sensor and a set 
 * point that the engine coolant is trying to be kept at. The output drives
 * the water pump faster if the coolant is too hot and slower if the coolant
 * is too cold. A PID can be use to keep many systems at their set point.
 * A PID is called a PID because the algorithm has three components:
 * proportional, integral and derivative. Each of these components can be
 * multiplied by a coefficient to give them a weighting. These are the 
 * tuning parameters to dial the PID in to the specific situation.
 * 
 * @param _Kp The proportional coefficient
 * @param _Ki The integral coefficient
 * @param _Kd The derivative coefficient
 * @param _Sp The Setpoint
 * @param config_path The path to configure this transform in the Config UI
 **/

class PID : public Transform<float, float> {
 public:
  PID(float _Kp, float _Ki, float _Kd, float _Setpoint, String config_path = "");
  void set_input(float input, uint8_t input_channel = 0) override;
  float get_input() { return Input; };
  float get_setpoint() { return Setpoint; };
  void SetSetpoint(float _setpoint);
  void SetTunings(double _Kp, double _Ki, double _Kd);
  void SetSampleTime(unsigned int NewSampleTime); 
  void SetOutputLimits(double Min, double Max);
  void SetMode(int _Mode);
  void Restart();
  void SetControllerDirection(int Direction);
  virtual void get_configuration(JsonObject& doc) override;
  virtual bool set_configuration(const JsonObject& config) override;
  virtual String get_config_schema() override;

 private:
  float (*function_)(float);
  float Kp;
  float Ki;
  float Kd;
  float Setpoint;
  unsigned long lastTime = 0;
  double IntegralTerm, lastInput;
  double lastErr = 0;
  int ControllerDirection = DIRECT;
  bool Mode = AUTOMATIC;
  float Input;
  float Output;
  unsigned int SampleTime = 1000; // 1 second
  double outMin, outMax;
};

}  // namespace sensesp

#endif