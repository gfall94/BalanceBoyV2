#include <Arduino.h>
#include <SimpleFOC.h>

//AS5600 configuration
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
// Motor instance
// BLDCMotor motor = BLDCMotor(11, 13.7, 20);
BLDCMotor motor = BLDCMotor(11);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
// Gain calculation shown at https://community.simplefoc.com/t/b-g431b-esc1-current-control/521/21
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f / 7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// Reporting cycle
uint32_t reportCycle = 13; // in milliseconds ~75Hz
uint32_t last_report;

// instantiate the commander
Commander command = Commander(Serial);

uint32_t last_receive_time = 0;
void doTarget(char *cmd) { 
  command.motion(&motor, cmd); 

  // Benchmark: Time since last receive
  // uint32_t now = micros();
  // uint32_t latency = now - last_receive_time;
  // last_receive_time = now;

  // send back the latency in µs
  // Serial.print(F("Latency(us):"));
  // Serial.println(latency);
}

void setup()
{

  delay(5000);
  // use monitoring with serial
  Serial.begin(921600);

  // initialize encoder sensor hardware
  // configure i2C
  Wire.setClock(400000);
  // initialise magnetic sensor hardware
  sensor.init();

  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  // pwm frequency
  // driver.pwm_frequency = 8000;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);
  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // link current sense and the driver
  currentSense.linkDriver(&driver);

  // current sensing
  currentSense.init();
  // no need for aligning
  // currentSense.skip_align = false;
  motor.linkCurrentSense(&currentSense);

  // choose torque control mode
  motor.torque_controller = TorqueControlType::dc_current;

  // aligning voltage [V]
  motor.voltage_sensor_align = 3;
  // index search velocity [rad/s]
  motor.velocity_index_search = 3;

  // set motion control loop to be used
  motor.controller = MotionControlType::torque;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  // motor.PID_velocity.P = 0.2;
  // motor.PID_velocity.I = 20;
  // default voltage_power_supply
  // motor.voltage_limit = 12;
  motor.voltage_limit = 8;
  motor.current_limit = 1.2;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  // motor.PID_velocity.output_ramp = 1000;

  // velocity low pass filtering time constant
  // motor.LPF_velocity.Tf = 0.01;

  // angle P controller
  // motor.P_angle.P = 20;
  // maximum velocity of the position control
  // motor.velocity_limit = 4;

  // comment out if not needed
  // motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  // add target command T
  // command.add('T', doTarget, "target angle");
  command.add('T', doTarget);
  command.verbose = VerboseMode::nothing;

  delay(1000);
  // Serial.println(F("Motor ready."));
  // Serial.println(F("Set the target angle using serial terminal:"));
  last_report = millis();
}

void loop()
{
  // user communication
  while (Serial.available()) {
    command.run();
  }

  // main FOC algorithm
  motor.loopFOC();

  // Motion control
  motor.move();

  // reporting
  if ((millis() - last_report) >= reportCycle)
  {
    // monitor variables for serial plotter
    // Serial.printf("%f,%f,%d\n", motor.shaft_angle, motor.shaft_velocity, 1000/(millis()-last_report));

    float position = motor.shaft_angle;
    float velocity = motor.shaft_velocity;

    // Start-Marker (STX)
    Serial.write(0x02);
    // send float as byte-array
    Serial.write((byte *)&position, sizeof(float));
    Serial.write((byte *)&velocity, sizeof(float));
    // End-Marker (ETX)
    Serial.write(0x03);

    last_report = millis();
  }

}
