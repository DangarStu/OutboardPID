// Outboard Cooling Control

#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"
#include "sensesp/transforms/analogvoltage.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/voltagedivider.h"
#include "sensesp/transforms/moving_average.h"
#include "pid.h"
#include "temperature-interpolator.cpp"
#include <Arduino.h>

#include "secrets.h" // Git ignored file

using namespace sensesp;

reactesp::ReactESP app;

//Specify initial tuning parameters for the PID code.

// Weighting of Proportional component
float Kp { 1.0 };

// Weighting of Integral component
float Ki { 1.0 };

// Weighting of Derivative component
float Kd { 1.0 };

// Setpoint: the target temperature we are trying to maintain
// +273.15 to convert to Kelvin.
float Setpoint { 80.0 + 273.15 };

// The maximum amount the temperature can get above the setpoint
// before sounding the alarm
const float alarm_margin { 10 };

// GPIO number to use for the alarm buzzer.
// This is the pin to send Vext out from the optocoupler.
const uint8_t ALARM_PIN { 33 };

// GPIO number to use for the coolant sensor input
const uint8_t THERMISTOR_PIN { 36 };

// PWM properties to use for the Electronic Speed Controller
const int freq   { 5000 };
const uint8_t ESC_PIN { 15 };
const int ESC_CHANNEL { 0 };
const int resolution { 8 };

// Store the alarm state so it can be written to Signal K
// An easily acted upon by Signal K
uint8_t alarm_pin_state { LOW };

// Record the last time the pump went off
unsigned long last_change { 0 };

// The setup function performs one-time application initialization.
void setup() {
  #ifndef SERIAL_DEBUG_DISABLED
    SetupSerialDebug(115200);
  #endif

  pinMode(ALARM_PIN, OUTPUT); 

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;

  sensesp_app = (&builder)
    ->set_wifi(SSID, WIFI_PASSWORD) // From .gitignored user-created file 'secrets.h' 
    ->set_hostname("outboard")      // Visit http://outboard.local to configure
    ->set_sk_server(SIGNALK_IP, 80) // From .gitignored user-created file 'secrets.h' 
    ->get_app();

  // The sample time to set both the PID and Setpoint forwarder to
  const unsigned int sample_time = 1000;

  // Voltage sent into the voltage divider circuit that includes the analog
  // sender
  const float Vin { 3.29 };

  // The resistance, in ohms, of the fixed resistor (R1) in the voltage divider
  // circuit
  const float R2 { 330.0 };

  // An AnalogInput gets the value from the microcontroller's AnalogIn pin,
  // which is a value from 0 to 4095.
  auto* analog_input = new AnalogInput(THERMISTOR_PIN, 200, "", 4096);

  // Create the PIDControler here so we can save a reference to it and
  // configure it with parameters not set in the constructor.
  PID* PIDController = new PID(Kp, Ki, Kd, Setpoint, "/coolant/temp/controller");
  PIDController->SetOutputLimits(0, 255);
  PIDController->SetSampleTime(sample_time);

  // Setup the PWM configuration
  ledcSetup(ESC_CHANNEL, freq, resolution);
  ledcAttachPin(ESC_PIN, ESC_CHANNEL);

  // ------------------------------------------------------------------------------
  // A Lamba function to drive the water pump motor based on
  // the output from the PID controller
  // ------------------------------------------------------------------------------
  auto drive_motor = [](float input, float setpoint, float temperature) -> float {
    debugD("Output from PID is %F", input);

    // The PID output is already scaled 0-255 so simply write it out to the DAC
    ledcWrite(ESC_CHANNEL, static_cast<int>(input));

    return input;
  };

  // This is an array of parameter information, providing the keys and
  // descriptions required to display user-friendly values in the configuration
  // interface. These are two extra input parameters to drive_motor() above
  // and beyond the default 'input' parameter passed in by the chained value
  // producer.
  const ParamInfo* drive_motor_lambda_param_data = new ParamInfo[2]{
        {"setpoint", "PID Setpoint"}, {"temperature", "Temperature"}};

  // This is a LambaTransform that calls drive_motor(), passing two
  // additional parameters.
  auto drive_motor_lambda = new LambdaTransform<float, float, float, float>(
        drive_motor,
        PIDController->get_setpoint(),
        PIDController->get_input(),
        drive_motor_lambda_param_data,
        "");

  // ------------------------------------------------------------------------------
  // A Lamba function to activate the high temperature
  // alarm if current_temp is error_margin above setpoint
  // ------------------------------------------------------------------------------
  auto temp_alarm = [](float input, float setpoint, float error_margin) -> float {
    uint8_t new_state;
    // debugD("Current temperature is %F", input);

    // Determine how long since the last change of state
    int time_since_change = (millis() - last_change);

    if (input > setpoint + error_margin) {
      new_state = HIGH;
    } else {
      new_state = LOW;
    }

    if (new_state != alarm_pin_state) {
      // Store the current time at this change of state.
      last_change = millis();

      if (new_state == HIGH) { 
        debugD("Coolant over temp!");
      } else {
        debugD("Coolant back below temp.");
      }

      alarm_pin_state = new_state;
      digitalWrite(ALARM_PIN, alarm_pin_state);
    }

    if (alarm_pin_state == HIGH && time_since_change > 3000) {
      // Pump has been on for more than five minutes, sound
      // the super bad alarm

    }

    // Pass the input value on
    return input;
  };

  // This is an array of parameter information, providing the keys and
  // descriptions required to display user-friendly values in the configuration
  // interface. These are two extra input parameters to drive_motor() above
  // and beyond the default 'input' parameter passed in by the chained value
  // producer.
  const ParamInfo* temp_alarm_lambda_param_data = new ParamInfo[1]{
        {"alarm_margin", "Alarm margin"}};

  // This is a LambaTransform that calls sounds the over-temp alarm
  // if the current temperature is ever a set level over the setpoint.
  auto temp_alarm_lambda = new LambdaTransform<float, float, float, float>(
        temp_alarm,
        PIDController->get_setpoint(),
        alarm_margin,
        temp_alarm_lambda_param_data,
        "/coolant/temp/alarm_margin");
  // ------------------------------------------------------------------------------

  // Read the AnalogVoltage, smooth it with a MovingAverage, Calculate the resistance
  // of the thermistor using a VoltageDivider, convert that resistance to a temperature
  // using the TemperatureInterpolator, calibrate it with a Linear transform, pass
  // the temperature into the PID Controller to figure out what to do, control the
  // pump speed using that information then send the pump speed to Signal K.
  analog_input->connect_to(new AnalogVoltage())
    ->connect_to(new MovingAverage(20, 1.0, ""))
    ->connect_to(new VoltageDividerR2(R2, Vin, "/coolant/temp/sender"))
    ->connect_to(new TemperatureInterpolator("/coolant/temp/curve"))
    ->connect_to(new Linear(1.0, 0, "/coolant/temp/calibrate"))
    ->connect_to(temp_alarm_lambda)
    ->connect_to(new SKOutputNumeric<float>(
      "vessels.tinny.propulsion.outboard.coolant.temperature",
      "",
      new SKMetadata("K", "The current coolant temperature"))
    )
    ->connect_to(PIDController)
    ->connect_to(drive_motor_lambda)
    ->connect_to(new SKOutputNumeric<float>(
      "vessels.tinny.propulsion.outboard.revolutions",
      "",
      new SKMetadata("8BitDAC", "The current drive speed of the coolant pump."))
    );

  // A RepeatSensor that fetches the Setpoint from configuration
  auto setpoint = new RepeatSensor<float>(sample_time, [PIDController]() -> float {
    return(PIDController->get_setpoint());   
  });

  // Connect to this sensor to send the output to Signal K
  setpoint->connect_to(new SKOutputNumeric<float>(
      "vessels.tinny.propulsion.outboard.coolant.setpoint",
      "",
      new SKMetadata("K", "The target temperature of the coolant"))
    );

  // A RepeatSensor that fetches the state of the coolant temperature alarm.
  auto alarm_state = new RepeatSensor<uint8_t>(sample_time, []() -> uint8_t {
    return (alarm_pin_state);   
  });

  // Connect to this sensor to send the output to Signal K
  alarm_state->connect_to(new SKOutputNumeric<float>(
      "vessels.tinny.propulsion.outboard.coolant.alarm",
      "",
      new SKMetadata("Boolean", "The state of the over temp alarm."))
    );

  Serial.println(WiFi.status());
  Serial.println(WiFi.localIP());
  
  // Start networking and other SensESP internals
  sensesp_app->start();
}

void loop() { 
  app.tick(); 
}