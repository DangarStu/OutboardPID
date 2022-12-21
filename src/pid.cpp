#include "pid.h"

// PID code based on the wonderful tutorial by Brett Beauregard
// http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

namespace sensesp {


void PID::set_input(float input, uint8_t inputChannel) {
  // Update the latest input value
  Input = input;

  if(Mode == MANUAL)
    return;

  unsigned long now = millis();
  int timeChange = (now - lastTime);

  
  if(timeChange>=SampleTime) {
    // It's time to do the calculation
    // debugD("Input to PID is %F", input);
    // debugD("Setpoint is %F", Setpoint);

    // Compute the error which is the difference
    // between the Setpoint and the current Input
    double error = Input - Setpoint;

    // Compute Integral Term which is the sum of
    // the error multiplied by the Integral tuning
    // parameter.
    IntegralTerm+= (Ki * error);
    
    if(IntegralTerm > outMax) {
      // Make sure the Integral Term hasn't exceeded
      // The maximum output setting of the PID.
      IntegralTerm = outMax;
    } else if(IntegralTerm < outMin) {
      // Make sure the Integral Term hasn't exceeded
      // The maximum output setting of the PID.
      IntegralTerm = outMin;
    }

    double dInput = (Input - lastInput);
 
    // Compute PID Output
    Output = (Kp * error) + IntegralTerm - (Kd * dInput);
    // debugD("Proportional: %F", error);
    // debugD("Integral: %F", IntegralTerm);
    // debugD("Derivative: %F", dInput);

    if(Output > outMax) {
      // If the final output exceeds the maximum set it to the maximum.
      Output = outMax;
    } else {
      // If the final output is below the minimum set it to the minimum.
      if(Output < outMin) Output = outMin;
    }

    // Record the lastest input to compare it to the next one.
    lastInput = Input;

    // Record the current time to ensure the regular time interval of computes.
    lastTime = now;

    // Send the output.
    emit(Output);
  }

}

/*
 * The schema for the JSON configuration data.
 */
static const char SCHEMA_R1[] PROGMEM = R"({
    "type": "object",
    "properties": {
        "Kp": { 
          "title": "Proportion component",
          "type": "number" 
          },
        "Ki": { 
          "title": "Integral component",
          "type": "number" 
          },
        "Kd": { 
          "title": "Derivative component",
          "type": "number"
          },
        "Setpoint": {
          "title": "Setpoint",
          "type": "number"
          },
        "SampleTime": {
          "title": "Sample time in ms",
          "type": "number"
          },
        "ControllerDirection": {
          "title": "Controller direction",
          "type": "number"
          },
        "outMin": {
          "title": "Minimum PID output",
          "type": "number"
          },
        "outMax": {
          "title": "Maximum PID output",
          "type": "number"
          }
    }
  })";

String PID::get_config_schema() { return FPSTR(SCHEMA_R1); }

/*
 * Write the JSON configuration data into the class members.
 */
bool PID::set_configuration(const JsonObject& config) {
  String expected[] = {"Kp", "Ki", "Kd", "Setpoint", "SampleTime", "ControllerDirection", "outMin", "outMax"};
  for (auto str : expected) {
    if (!config.containsKey(str)) {
      return false;
    }
  }

  Kp = config["Kp"];
  Ki = config["Ki"];
  Kd = config["Kd"];
  Setpoint = config["Setpoint"];
  SampleTime = config["SampleTime"];
  ControllerDirection = config["ControllerDirection"];
  outMin = config["outMin"];
  outMax = config["outMax"];

  return true;
}

/** 
* Stores the current configuration data in a JsonObject.
* @param doc The JSON object to write the data too
*/
void PID::get_configuration(JsonObject& doc) {
  doc["Kp"] = Kp;
  doc["Ki"] = Ki;
  doc["Kd"] = Kd;
  doc["Setpoint"] = Setpoint;
  doc["SampleTime"] = SampleTime;
  doc["ControllerDirection"] = ControllerDirection;
  doc["outMin"] = outMin;
  doc["outMax"] = outMax;
}

/** 
 * @brief Configure the tuning variables for the PID Controller
 * @param _Kp This is the proportional component.
 * @param _Ki This is the integral component.
 * @param _Kd This is the derivative component.
 */
void PID::SetTunings(double _Kp, double _Ki, double _Kd)
{
  if (_Kp<0 || _Ki<0|| _Kd<0) return;
 
  double SampleTimeInSec = ((double)SampleTime)/1000;

  Kp = _Kp;
  Ki = _Ki * SampleTimeInSec;
  Kd / SampleTimeInSec;
 
  if(ControllerDirection == REVERSE)
   {
      Kp = (0 - Kp);
      Ki = (0 - Ki);
      Kd = (0 - Kd);
   }

   save_configuration ();
}
 
void PID::SetSampleTime(unsigned int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      Ki *= ratio;
      Kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }

   save_configuration ();
}
 
void PID::SetOutputLimits(double Min, double Max)
{
  if(Min > Max) return;

  outMin = Min;
  outMax = Max;
 
  if(Output > outMax) {
    Output = outMax;
  } else if(Output < outMin) {
    Output = outMin;
  }
 
  if(IntegralTerm > outMax) {
    IntegralTerm= outMax;
  } else if(IntegralTerm < outMin) {
    IntegralTerm= outMin;
  }
  save_configuration ();
}
 
void PID::SetMode(int _Mode)
{

  if(Mode == MANUAL && Mode == AUTOMATIC) {
    // Swapping from MANUAL to AUTOMATIC mode so run Restart()
    Restart();
  }

  Mode = _Mode;
     
  save_configuration ();
}
 
void PID::Restart()
{
   lastInput = Input;
   IntegralTerm = Output;

   // Check for limits
   if(IntegralTerm > outMax) {
     IntegralTerm= outMax;
   } else if(IntegralTerm < outMin) {
     IntegralTerm= outMin;
   } 
}
 
void PID::SetControllerDirection(int Direction)
{
   ControllerDirection = Direction;
      
  save_configuration ();
}

void PID::SetSetpoint(float _setpoint)
{
   Setpoint = _setpoint;
      
  save_configuration ();
}

PID::PID::PID(float _Kp, float _Ki, float _Kd, float _Setpoint, String config_path)
: Transform<float, float>(config_path) {
    Kp = _Kp;
    Ki = _Ki;
    Kd = _Kd;
    Setpoint = _Setpoint;

    load_configuration();
}

} // namespace sensesp