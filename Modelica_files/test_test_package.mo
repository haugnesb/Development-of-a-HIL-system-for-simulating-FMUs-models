package TestPackage
  package Old_Models
    model Test_VFD_Servo3
      parameter Real Motorvoltage = 2.5 "Voltage of the motor";
      parameter Integer TorqueStartTime = 30 "Start time for the torque signalin seconds";
      parameter Real AmplitudeTorque = 5 "Height of torque signal";
      parameter Real TorqueSignalDamping = 0.2 "Damping of the torque signal";
      parameter Real TorqueSignalFrequency = 2 "Frequency of the torque signal";
      parameter Real TorqueSignalPhaseAngle = 30 "The phase angle of the Torque signal";
      parameter Real InitialTorque = 0.61 "The star torque level";
      parameter Real ServoStartTorque = 0.61 "Start torque for the servo motor";
      parameter Integer OnOff = 1 "Start (1) and stop (0) the motors";
      parameter Integer SpeedTorqueControl = 1 "Speed (0) or Torque (1) control";
      parameter Integer ForwardReverse = 0 "Forward (0) or Reverse (1) control";
      Modelica.Blocks.Sources.Constant ConstantVoltage(k = Motorvoltage) annotation(
        Placement(transformation(extent = {{-80, -98}, {-60, -78}})));
      Modelica.Blocks.Interfaces.IntegerOutput Start_Stop "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, -60}, {120, -40}})));
      Modelica.Blocks.Interfaces.RealOutput Servo_motor_speed "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, -98}, {120, -78}})));
      Modelica.Blocks.Interfaces.RealOutput Test_motor_torque "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, -20}, {120, 0}})));
      Modelica.Blocks.Interfaces.IntegerOutput Speed_Torque "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, 20}, {120, 40}})));
      Modelica.Blocks.Interfaces.IntegerOutput Forward_Reverse "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, 60}, {120, 80}})));
      Modelica.Blocks.Sources.IntegerConstant ForwardOrReverse(k = ForwardReverse) annotation(
        Placement(transformation(extent = {{-80, 60}, {-60, 80}})));
      Modelica.Blocks.Sources.IntegerConstant SpeedOrTorque(k = SpeedTorqueControl) annotation(
        Placement(transformation(extent = {{-80, 20}, {-60, 40}})));
      Modelica.Blocks.Sources.IntegerConstant StartOrStop(k = OnOff) annotation(
        Placement(transformation(extent = {{-80, -60}, {-60, -40}})));
      Modelica.Blocks.Sources.Constant ServoTorque(k = ServoStartTorque) annotation(
        Placement(transformation(extent = {{-44, -80}, {-24, -60}})));
      Modelica.Blocks.Interfaces.RealOutput Servo_Torque "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, -80}, {120, -60}})));
      Modelica.Blocks.Sources.ExpSine TestMotorTorque(amplitude = AmplitudeTorque, f = TorqueSignalFrequency, phase = TorqueSignalPhaseAngle, damping = TorqueSignalDamping, offset = InitialTorque, startTime = TorqueStartTime) annotation(
        Placement(transformation(extent = {{-80, -20}, {-60, 0}})));
    equation
      connect(ConstantVoltage.y, Servo_motor_speed) annotation(
        Line(points = {{-59, -88}, {110, -88}}, color = {0, 0, 127}));
      connect(ForwardOrReverse.y, Forward_Reverse) annotation(
        Line(points = {{-59, 70}, {110, 70}}, color = {255, 127, 0}));
      connect(StartOrStop.y, Start_Stop) annotation(
        Line(points = {{-59, -50}, {22, -50}, {22, -50}, {110, -50}}, color = {255, 127, 0}));
      connect(Speed_Torque, SpeedOrTorque.y) annotation(
        Line(points = {{110, 30}, {-59, 30}}, color = {255, 127, 0}));
      connect(ServoTorque.y, Servo_Torque) annotation(
        Line(points = {{-23, -70}, {110, -70}}, color = {0, 0, 127}));
      connect(TestMotorTorque.y, Test_motor_torque) annotation(
        Line(points = {{-59, -10}, {110, -10}}, color = {0, 0, 127}));
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = false)),
        Diagram(coordinateSystem(preserveAspectRatio = false)),
        experiment(StopTime = 300));
    end Test_VFD_Servo3;

    model Test_VFD_Servo2
      parameter Real Motorvoltage = 2.5 "Voltage of the motor";
      parameter Integer StartTorqueRampUpTime = 10 "Startime of the step up torque in seconds";
      parameter Integer TorqueRamUpDuration = 30 "Duration of the Torque ramp up in seconds";
      parameter Real HeightTorque = 5 "Height of torque signal";
      parameter Real InitialTorque = 0.61 "The star torque level";
      parameter Real ServoStartTorque = 0.61 "Start torque for the servo motor";
      parameter Integer OnOff = 1 "Start (1) and stop (0) the motors";
      parameter Integer SpeedTorqueControl = 1 "Speed (0) or Torque (1) control";
      parameter Integer ForwardReverse = 0 "Forward (0) or Reverse (1) control";
      Modelica.Blocks.Sources.Constant ConstantVoltage(k = Motorvoltage) annotation(
        Placement(transformation(extent = {{-80, -98}, {-60, -78}})));
      Modelica.Blocks.Interfaces.IntegerOutput Start_Stop "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, -60}, {120, -40}})));
      Modelica.Blocks.Interfaces.RealOutput Servo_motor_speed "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, -98}, {120, -78}})));
      Modelica.Blocks.Interfaces.RealOutput Test_motor_torque "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, -20}, {120, 0}})));
      Modelica.Blocks.Interfaces.IntegerOutput Speed_Torque "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, 20}, {120, 40}})));
      Modelica.Blocks.Interfaces.IntegerOutput Forward_Reverse "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, 60}, {120, 80}})));
      Modelica.Blocks.Sources.IntegerConstant ForwardOrReverse(k = ForwardReverse) annotation(
        Placement(transformation(extent = {{-80, 60}, {-60, 80}})));
      Modelica.Blocks.Sources.IntegerConstant SpeedOrTorque(k = SpeedTorqueControl) annotation(
        Placement(transformation(extent = {{-80, 20}, {-60, 40}})));
      Modelica.Blocks.Sources.IntegerConstant StartOrStop(k = OnOff) annotation(
        Placement(transformation(extent = {{-80, -60}, {-60, -40}})));
      Modelica.Blocks.Sources.Ramp TestMotorTorque(height = HeightTorque, duration = TorqueRamUpDuration, offset = InitialTorque, startTime = StartTorqueRampUpTime) annotation(
        Placement(transformation(extent = {{-80, -20}, {-60, 0}})));
      Modelica.Blocks.Sources.Constant ServoTorque(k = ServoStartTorque) annotation(
        Placement(transformation(extent = {{-44, -80}, {-24, -60}})));
      Modelica.Blocks.Interfaces.RealOutput Servo_Torque "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, -80}, {120, -60}})));
    equation
      connect(ConstantVoltage.y, Servo_motor_speed) annotation(
        Line(points = {{-59, -88}, {110, -88}}, color = {0, 0, 127}));
      connect(ForwardOrReverse.y, Forward_Reverse) annotation(
        Line(points = {{-59, 70}, {110, 70}}, color = {255, 127, 0}));
      connect(StartOrStop.y, Start_Stop) annotation(
        Line(points = {{-59, -50}, {22, -50}, {22, -50}, {110, -50}}, color = {255, 127, 0}));
      connect(Speed_Torque, SpeedOrTorque.y) annotation(
        Line(points = {{110, 30}, {-59, 30}}, color = {255, 127, 0}));
      connect(TestMotorTorque.y, Test_motor_torque) annotation(
        Line(points = {{-59, -10}, {110, -10}}, color = {0, 0, 127}));
      connect(ServoTorque.y, Servo_Torque) annotation(
        Line(points = {{-23, -70}, {110, -70}}, color = {0, 0, 127}));
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = false)),
        Diagram(coordinateSystem(preserveAspectRatio = false)),
        experiment(StopTime = 300));
    end Test_VFD_Servo2;

    model Test_VFD_Servo1
      parameter Real Motorvoltage = 2.5 "Voltage of the motor";
      parameter Real StartTorqueTime = 5 "Startime of the step up torque in seconds";
      parameter Real HeightTorque = 5 "Height of torque signal";
      parameter Real InitialTorque = 0.61 "The star torque level";
      parameter Real ServoStartTorque = 0.61 "Start torque for the servo motor";
      parameter Integer OnOff = 1 "Start (1) and stop (0) the motors";
      parameter Integer SpeedTorqueControl = 1 "Speed (0) or Torque (1) control";
      parameter Integer ForwardReverse = 0 "Forward (0) or Reverse (1) control";
      Modelica.Blocks.Sources.Constant ConstantVoltage(k = Motorvoltage) annotation(
        Placement(transformation(extent = {{-80, -98}, {-60, -78}})));
      Modelica.Blocks.Interfaces.IntegerOutput Start_Stop "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, -60}, {120, -40}})));
      Modelica.Blocks.Interfaces.RealOutput Servo_motor_speed "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, -98}, {120, -78}})));
      Modelica.Blocks.Sources.Step StepUpTorque(height = HeightTorque, offset = InitialTorque, startTime = StartTorqueTime) annotation(
        Placement(transformation(extent = {{-80, -20}, {-60, 0}})));
      Modelica.Blocks.Interfaces.RealOutput Test_motor_torque "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, -20}, {120, 0}})));
      Modelica.Blocks.Interfaces.IntegerOutput Speed_Torque "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, 20}, {120, 40}})));
      Modelica.Blocks.Interfaces.IntegerOutput Forward_Reverse "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, 60}, {120, 80}})));
      Modelica.Blocks.Sources.IntegerConstant ForwardOrReverse(k = ForwardReverse) annotation(
        Placement(transformation(extent = {{-80, 60}, {-60, 80}})));
      Modelica.Blocks.Sources.IntegerConstant SpeedOrTorque(k = SpeedTorqueControl) annotation(
        Placement(transformation(extent = {{-80, 20}, {-60, 40}})));
      Modelica.Blocks.Sources.IntegerConstant StartOrStop(k = OnOff) annotation(
        Placement(transformation(extent = {{-80, -60}, {-60, -40}})));
      Modelica.Blocks.Sources.Constant ServoTorque(k = ServoStartTorque) annotation(
        Placement(transformation(extent = {{-46, -80}, {-26, -60}})));
      Modelica.Blocks.Interfaces.RealOutput Servo_Start_Torque "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{98, -80}, {118, -60}})));
    equation
      connect(ConstantVoltage.y, Servo_motor_speed) annotation(
        Line(points = {{-59, -88}, {110, -88}}, color = {0, 0, 127}));
      connect(StepUpTorque.y, Test_motor_torque) annotation(
        Line(points = {{-59, -10}, {110, -10}}, color = {0, 0, 127}));
      connect(ForwardOrReverse.y, Forward_Reverse) annotation(
        Line(points = {{-59, 70}, {110, 70}}, color = {255, 127, 0}));
      connect(StartOrStop.y, Start_Stop) annotation(
        Line(points = {{-59, -50}, {22, -50}, {22, -50}, {110, -50}}, color = {255, 127, 0}));
      connect(Speed_Torque, SpeedOrTorque.y) annotation(
        Line(points = {{110, 30}, {-59, 30}}, color = {255, 127, 0}));
      connect(ServoTorque.y, Servo_Start_Torque) annotation(
        Line(points = {{-25, -70}, {108, -70}}, color = {0, 0, 127}));
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = false)),
        Diagram(coordinateSystem(preserveAspectRatio = false)),
        experiment(StopTime = 300));
    end Test_VFD_Servo1;

    model Digital_signal_test
      parameter Real PulsAmplitude = 1 "Amplitude of the puls";
      parameter Integer DCOM = 1 "Digital input common";
      parameter Real PulsStartTime = 0 "start time off the puls";
      parameter Real PulsPeriod = 60 "Puls Duration In Seconds";
      parameter Real PulsWidth = 50 "Puls on time in";
      Modelica.Blocks.Math.RealToInteger realToInteger annotation(
        Placement(transformation(extent = {{-22, -10}, {-2, 10}})));
      Modelica.Blocks.Sources.Pulse pulse(amplitude = PulsAmplitude, width = PulsWidth, period = PulsPeriod, startTime = PulsStartTime) annotation(
        Placement(transformation(extent = {{-80, -10}, {-60, 10}})));
      Modelica.Blocks.Interfaces.IntegerOutput StartStop "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, -10}, {120, 10}})));
      Modelica.Blocks.Sources.IntegerConstant integerConstant(k = DCOM) annotation(
        Placement(transformation(extent = {{-80, 30}, {-60, 50}})));
      Modelica.Blocks.Interfaces.IntegerOutput DCOMSignal "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, 30}, {120, 50}})));
    equation
      connect(realToInteger.u, pulse.y) annotation(
        Line(points = {{-24, 0}, {-59, 0}}, color = {0, 0, 127}));
      connect(realToInteger.y, StartStop) annotation(
        Line(points = {{-1, 0}, {110, 0}}, color = {255, 127, 0}));
      connect(integerConstant.y, DCOMSignal) annotation(
        Line(points = {{-59, 40}, {110, 40}}, color = {255, 127, 0}));
      connect(DCOMSignal, DCOMSignal) annotation(
        Line(points = {{110, 40}, {110, 40}}, color = {255, 127, 0}));
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = false)),
        Diagram(coordinateSystem(preserveAspectRatio = false)),
        experiment(StopTime = 300));
    end Digital_signal_test;

    model Digital_signal_test1
      parameter Integer DCOM = 1 "Digital input common";
      Modelica.Blocks.Sources.IntegerConstant integerConstant(k = DCOM) annotation(
        Placement(transformation(extent = {{-80, 30}, {-60, 50}})));
      Modelica.Blocks.Interfaces.IntegerOutput DCOMSignal "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, 30}, {120, 50}})));
    equation
      connect(integerConstant.y, DCOMSignal) annotation(
        Line(points = {{-59, 40}, {110, 40}}, color = {255, 127, 0}));
      connect(DCOMSignal, DCOMSignal) annotation(
        Line(points = {{110, 40}, {110, 40}}, color = {255, 127, 0}));
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = false)),
        Diagram(coordinateSystem(preserveAspectRatio = false)),
        experiment(StopTime = 300));
    end Digital_signal_test1;

    model Test1_VFD
      parameter Real PulsAmplitude = 1 "Amplitude of the puls";
      parameter Integer Motorvoltage = 5 "Voltage of the motor";
      parameter Integer DCOM = 1 "Digital input common";
      parameter Real PulsStartTime = 0 "start time off the puls";
      parameter Real PulsPeriod = 60 "Puls Duration In Seconds";
      parameter Real PulsWidth = 50 "Puls on time in";
      Modelica.Blocks.Math.RealToInteger realToInteger annotation(
        Placement(transformation(extent = {{-22, -10}, {-2, 10}})));
      Modelica.Blocks.Sources.Pulse pulse(amplitude = PulsAmplitude, width = PulsWidth, period = PulsPeriod, startTime = PulsStartTime) annotation(
        Placement(transformation(extent = {{-80, -10}, {-60, 10}})));
      Modelica.Blocks.Sources.Constant const(k = Motorvoltage) annotation(
        Placement(transformation(extent = {{-80, -76}, {-60, -56}})));
      Modelica.Blocks.Math.Product product annotation(
        Placement(transformation(extent = {{-22, -70}, {-2, -50}})));
      Modelica.Blocks.Interfaces.IntegerOutput StartStop "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, -10}, {120, 10}})));
      Modelica.Blocks.Interfaces.RealOutput MotorVoltage "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, -70}, {120, -50}})));
      Modelica.Blocks.Sources.IntegerConstant integerConstant(k = DCOM) annotation(
        Placement(transformation(extent = {{-80, 30}, {-60, 50}})));
      Modelica.Blocks.Interfaces.IntegerOutput DCOMSignal "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, 30}, {120, 50}})));
    equation
      connect(pulse.y, product.u1) annotation(
        Line(points = {{-59, 0}, {-32, 0}, {-32, -54}, {-24, -54}}, color = {0, 0, 127}));
      connect(const.y, product.u2) annotation(
        Line(points = {{-59, -66}, {-24, -66}}, color = {0, 0, 127}));
      connect(realToInteger.u, pulse.y) annotation(
        Line(points = {{-24, 0}, {-59, 0}}, color = {0, 0, 127}));
      connect(realToInteger.y, StartStop) annotation(
        Line(points = {{-1, 0}, {110, 0}}, color = {255, 127, 0}));
      connect(product.y, MotorVoltage) annotation(
        Line(points = {{-1, -60}, {110, -60}}, color = {0, 0, 127}));
      connect(integerConstant.y, DCOMSignal) annotation(
        Line(points = {{-59, 40}, {110, 40}}, color = {255, 127, 0}));
      connect(DCOMSignal, DCOMSignal) annotation(
        Line(points = {{110, 40}, {110, 40}}, color = {255, 127, 0}));
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = false)),
        Diagram(coordinateSystem(preserveAspectRatio = false)),
        experiment(StopTime = 300));
    end Test1_VFD;

    model Test_VFD
      parameter Real PulsAmplitude = 1 "Amplitude of the puls";
      parameter Integer Motorvoltage = 5 "Voltage of the motor";
      parameter Real PulsStartTime = 0 "start time off the puls";
      parameter Real PulsPeriod = 60 "Puls Duration In Seconds";
      parameter Real PulsWidth = 50 "Puls on time in";
      Modelica.Blocks.Math.RealToInteger realToInteger annotation(
        Placement(transformation(extent = {{-22, -10}, {-2, 10}})));
      Modelica.Blocks.Sources.Pulse pulse(amplitude = PulsAmplitude, width = PulsWidth, period = PulsPeriod, startTime = PulsStartTime) annotation(
        Placement(transformation(extent = {{-80, -10}, {-60, 10}})));
      Modelica.Blocks.Sources.Constant const(k = Motorvoltage) annotation(
        Placement(transformation(extent = {{-80, -76}, {-60, -56}})));
      Modelica.Blocks.Math.Product product annotation(
        Placement(transformation(extent = {{-22, -70}, {-2, -50}})));
      Modelica.Blocks.Interfaces.IntegerOutput StartStop "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, -10}, {120, 10}})));
      Modelica.Blocks.Interfaces.RealOutput MotorVoltage "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, -70}, {120, -50}})));
    equation
      connect(pulse.y, product.u1) annotation(
        Line(points = {{-59, 0}, {-32, 0}, {-32, -54}, {-24, -54}}, color = {0, 0, 127}));
      connect(const.y, product.u2) annotation(
        Line(points = {{-59, -66}, {-24, -66}}, color = {0, 0, 127}));
      connect(realToInteger.u, pulse.y) annotation(
        Line(points = {{-24, 0}, {-59, 0}}, color = {0, 0, 127}));
      connect(realToInteger.y, StartStop) annotation(
        Line(points = {{-1, 0}, {110, 0}}, color = {255, 127, 0}));
      connect(product.y, MotorVoltage) annotation(
        Line(points = {{-1, -60}, {110, -60}}, color = {0, 0, 127}));
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = false)),
        Diagram(coordinateSystem(preserveAspectRatio = false)),
        experiment(StopTime = 300));
    end Test_VFD;

    model Test_VFD_Servo1_V2
      parameter Real ServoMotorvoltage = 2.5 "Voltage of the Servomotor";
      parameter Integer StopOrStartServoMotor = 1 "Stop (0) and start (1) amplitude signal for the servomotor";
      parameter Real StartTorqueTime = 5 "Startime of the step up torque in seconds";
      parameter Real HeightTorque = 5 "Height of torque signal";
      parameter Real InitialTorque = 0.61 "The star torque level";
      parameter Real ServoStartTorque = 0.61 "Start torque for the servo motor";
      parameter Integer StopOrStartTestMotor = 1 "Stop (0) and start (1) amplitude signal for the testmotors";
      parameter Integer SpeedTorqueSignal = 1 "Speed (0) or Torque (1) amplitude signal for the testmotors";
      parameter Integer ForwardOrReverseSignal = 0 "Forward (0) or Reverse (1) amplitude signal for the testmotors";
      Modelica.Blocks.Sources.Constant ConstantVoltage(k = ServoMotorvoltage) annotation(
        Placement(transformation(extent = {{-80, -98}, {-60, -78}})));
      Modelica.Blocks.Interfaces.IntegerOutput Start_Stop_Testmotor "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, 20}, {120, 40}})));
      Modelica.Blocks.Interfaces.RealOutput Servo_motor_speed "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, -98}, {120, -78}})));
      Modelica.Blocks.Sources.Step StepUpTorque(height = HeightTorque, offset = InitialTorque, startTime = StartTorqueTime) annotation(
        Placement(transformation(extent = {{-80, 40}, {-60, 60}})));
      Modelica.Blocks.Interfaces.RealOutput Test_motor_torque "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, 40}, {120, 60}})));
      Modelica.Blocks.Interfaces.IntegerOutput Speed_Torque "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, 60}, {120, 80}})));
      Modelica.Blocks.Interfaces.IntegerOutput Forward_Reverse "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, 80}, {120, 100}})));
      Modelica.Blocks.Sources.Constant ServoTorque(k = ServoStartTorque) annotation(
        Placement(transformation(extent = {{-46, -80}, {-26, -60}})));
      Modelica.Blocks.Interfaces.RealOutput Servo_Start_Torque "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, -80}, {120, -60}})));
      Modelica.Blocks.Math.RealToInteger realToInteger annotation(
        Placement(transformation(extent = {{0, 80}, {20, 100}})));
      Modelica.Blocks.Math.RealToInteger realToInteger1 annotation(
        Placement(transformation(extent = {{40, 60}, {60, 80}})));
      Modelica.Blocks.Math.RealToInteger realToInteger2 annotation(
        Placement(transformation(extent = {{40, 20}, {60, 40}})));
      Modelica.Blocks.Math.RealToInteger realToInteger3 annotation(
        Placement(transformation(extent = {{0, -60}, {20, -40}})));
      Modelica.Blocks.Interfaces.IntegerOutput Start_Stop_Servomotor "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, -60}, {120, -40}})));
      Modelica.Blocks.Sources.Constant ForwardAndReverseControlSignal(k = ForwardOrReverseSignal) annotation(
        Placement(transformation(extent = {{-80, 80}, {-60, 100}})));
      Modelica.Blocks.Sources.Constant ServoMotorStartAndStopSignal(k = StopOrStartServoMotor) annotation(
        Placement(transformation(extent = {{-80, -60}, {-60, -40}})));
      Modelica.Blocks.Sources.Constant SpeedAndTorqueControlSignal(k = SpeedTorqueSignal) annotation(
        Placement(transformation(extent = {{-40, 60}, {-20, 80}})));
      Modelica.Blocks.Sources.Constant TestMotorStartAndStopSignal(k = StopOrStartTestMotor) annotation(
        Placement(transformation(extent = {{-40, 20}, {-20, 40}})));
    equation
      connect(ConstantVoltage.y, Servo_motor_speed) annotation(
        Line(points = {{-59, -88}, {110, -88}}, color = {0, 0, 127}));
      connect(StepUpTorque.y, Test_motor_torque) annotation(
        Line(points = {{-59, 50}, {110, 50}}, color = {0, 0, 127}));
      connect(ServoTorque.y, Servo_Start_Torque) annotation(
        Line(points = {{-25, -70}, {110, -70}}, color = {0, 0, 127}));
      connect(realToInteger.y, Forward_Reverse) annotation(
        Line(points = {{21, 90}, {110, 90}}, color = {255, 127, 0}));
      connect(realToInteger1.y, Speed_Torque) annotation(
        Line(points = {{61, 70}, {110, 70}}, color = {255, 127, 0}));
      connect(realToInteger2.y, Start_Stop_Testmotor) annotation(
        Line(points = {{61, 30}, {110, 30}}, color = {255, 127, 0}));
      connect(realToInteger3.y, Start_Stop_Servomotor) annotation(
        Line(points = {{21, -50}, {110, -50}}, color = {255, 127, 0}));
      connect(realToInteger.u, ForwardAndReverseControlSignal.y) annotation(
        Line(points = {{-2, 90}, {-59, 90}}, color = {0, 0, 127}));
      connect(realToInteger1.u, SpeedAndTorqueControlSignal.y) annotation(
        Line(points = {{38, 70}, {-19, 70}}, color = {0, 0, 127}));
      connect(realToInteger2.u, TestMotorStartAndStopSignal.y) annotation(
        Line(points = {{38, 30}, {-19, 30}}, color = {0, 0, 127}));
      connect(realToInteger3.u, ServoMotorStartAndStopSignal.y) annotation(
        Line(points = {{-2, -50}, {-59, -50}}, color = {0, 0, 127}));
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = false)),
        Diagram(coordinateSystem(preserveAspectRatio = false)),
        experiment(StopTime = 300));
    end Test_VFD_Servo1_V2;

    model Test_VFD_Servo2_V2
      parameter Real ServoMotorvoltage = 2.5 "Voltage of the Servomotor";
      parameter Integer StopOrStartServoMotor = 1 "Stop (0) and start (1) amplitude signal for the servomotor";
      parameter Integer StartTorqueRampUpTime = 10 "Startime of the step up torque in seconds";
      parameter Integer TorqueRamUpDuration = 30 "Duration of the Torque ramp up in seconds";
      parameter Real HeightTorque = 5 "Height of torque signal";
      parameter Real InitialTorque = 0.61 "The star torque level";
      parameter Real ServoStartTorque = 0.61 "Start torque for the servo motor";
      parameter Integer StopOrStartTestMotor = 1 "Stop (0) and start (1) amplitude signal for the testmotors";
      parameter Integer SpeedTorqueSignal = 1 "Speed (0) or Torque (1) amplitude signal for the testmotors";
      parameter Integer ForwardOrReverseSignal = 0 "Forward (0) or Reverse (1) amplitude signal for the testmotors";
      Modelica.Blocks.Sources.Constant ConstantVoltage(k = ServoMotorvoltage) annotation(
        Placement(transformation(extent = {{-80, -98}, {-60, -78}})));
      Modelica.Blocks.Interfaces.IntegerOutput Start_Stop_Testmotor "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, 20}, {120, 40}})));
      Modelica.Blocks.Interfaces.RealOutput Servo_motor_speed "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, -98}, {120, -78}})));
      Modelica.Blocks.Interfaces.RealOutput Test_motor_torque "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, 40}, {120, 60}})));
      Modelica.Blocks.Interfaces.IntegerOutput Speed_Torque "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, 60}, {120, 80}})));
      Modelica.Blocks.Interfaces.IntegerOutput Forward_Reverse "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, 80}, {120, 100}})));
      Modelica.Blocks.Sources.Constant ServoTorque(k = ServoStartTorque) annotation(
        Placement(transformation(extent = {{-46, -80}, {-26, -60}})));
      Modelica.Blocks.Interfaces.RealOutput Servo_Start_Torque "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, -80}, {120, -60}})));
      Modelica.Blocks.Math.RealToInteger realToInteger annotation(
        Placement(transformation(extent = {{0, 80}, {20, 100}})));
      Modelica.Blocks.Math.RealToInteger realToInteger1 annotation(
        Placement(transformation(extent = {{40, 60}, {60, 80}})));
      Modelica.Blocks.Math.RealToInteger realToInteger2 annotation(
        Placement(transformation(extent = {{40, 20}, {60, 40}})));
      Modelica.Blocks.Math.RealToInteger realToInteger3 annotation(
        Placement(transformation(extent = {{0, -60}, {20, -40}})));
      Modelica.Blocks.Interfaces.IntegerOutput Start_Stop_Servomotor "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, -60}, {120, -40}})));
      Modelica.Blocks.Sources.Constant ForwardAndReverseControlSignal(k = ForwardOrReverseSignal) annotation(
        Placement(transformation(extent = {{-80, 80}, {-60, 100}})));
      Modelica.Blocks.Sources.Constant ServoMotorStartAndStopSignal(k = StopOrStartServoMotor) annotation(
        Placement(transformation(extent = {{-80, -60}, {-60, -40}})));
      Modelica.Blocks.Sources.Constant SpeedAndTorqueControlSignal(k = SpeedTorqueSignal) annotation(
        Placement(transformation(extent = {{-40, 60}, {-20, 80}})));
      Modelica.Blocks.Sources.Constant TestMotorStartAndStopSignal(k = StopOrStartTestMotor) annotation(
        Placement(transformation(extent = {{-40, 20}, {-20, 40}})));
      Modelica.Blocks.Sources.Ramp TestMotorTorque(height = HeightTorque, duration = TorqueRamUpDuration, offset = InitialTorque, startTime = StartTorqueRampUpTime) annotation(
        Placement(transformation(extent = {{-80, 40}, {-60, 60}})));
    equation
      connect(ConstantVoltage.y, Servo_motor_speed) annotation(
        Line(points = {{-59, -88}, {110, -88}}, color = {0, 0, 127}));
      connect(ServoTorque.y, Servo_Start_Torque) annotation(
        Line(points = {{-25, -70}, {110, -70}}, color = {0, 0, 127}));
      connect(realToInteger.y, Forward_Reverse) annotation(
        Line(points = {{21, 90}, {110, 90}}, color = {255, 127, 0}));
      connect(realToInteger1.y, Speed_Torque) annotation(
        Line(points = {{61, 70}, {110, 70}}, color = {255, 127, 0}));
      connect(realToInteger2.y, Start_Stop_Testmotor) annotation(
        Line(points = {{61, 30}, {110, 30}}, color = {255, 127, 0}));
      connect(realToInteger3.y, Start_Stop_Servomotor) annotation(
        Line(points = {{21, -50}, {110, -50}}, color = {255, 127, 0}));
      connect(realToInteger.u, ForwardAndReverseControlSignal.y) annotation(
        Line(points = {{-2, 90}, {-59, 90}}, color = {0, 0, 127}));
      connect(realToInteger1.u, SpeedAndTorqueControlSignal.y) annotation(
        Line(points = {{38, 70}, {-19, 70}}, color = {0, 0, 127}));
      connect(realToInteger2.u, TestMotorStartAndStopSignal.y) annotation(
        Line(points = {{38, 30}, {-19, 30}}, color = {0, 0, 127}));
      connect(realToInteger3.u, ServoMotorStartAndStopSignal.y) annotation(
        Line(points = {{-2, -50}, {-59, -50}}, color = {0, 0, 127}));
      connect(Test_motor_torque, TestMotorTorque.y) annotation(
        Line(points = {{110, 50}, {-59, 50}}, color = {0, 0, 127}));
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = false)),
        Diagram(coordinateSystem(preserveAspectRatio = false)),
        experiment(StopTime = 300));
    end Test_VFD_Servo2_V2;

    model Test_VFD_Servo3_V2
      parameter Real ServoMotorvoltage = 2.5 "Voltage of the Servomotor";
      parameter Integer StopOrStartServoMotor = 1 "Stop (0) and start (1) amplitude signal for the servomotor";
      parameter Real TorquePulsStartTime = 5 "Startime of the torque puls signal";
      parameter Real AmplitudeTorque = 5 "Height of torque puls signal";
      parameter Real TorqueSignalDamping = 0.2 "Damping of the torque signal";
      parameter Real TorqueSignalFrequency = 2 "Frequency of the torque signal";
      parameter Real TorqueSignalPhaseAngle = 30 "The phase angle of the Torque signal";
      parameter Real InitialTorque = 0.61 "The star torque level";
      parameter Real ServoStartTorque = 0.61 "Start torque for the servo motor";
      parameter Integer StopOrStartTestMotor = 1 "Stop (0) and start (1) amplitude signal for the testmotors";
      parameter Integer SpeedTorqueSignal = 1 "Speed (0) or Torque (1) amplitude signal for the testmotors";
      parameter Integer ForwardOrReverseSignal = 0 "Forward (0) or Reverse (1) amplitude signal for the testmotors";
      Modelica.Blocks.Sources.Constant ConstantVoltage(k = ServoMotorvoltage) annotation(
        Placement(transformation(extent = {{-80, -98}, {-60, -78}})));
      Modelica.Blocks.Interfaces.IntegerOutput Start_Stop_Testmotor "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, 20}, {120, 40}})));
      Modelica.Blocks.Interfaces.RealOutput Servo_motor_speed "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, -98}, {120, -78}})));
      Modelica.Blocks.Interfaces.RealOutput Test_motor_torque "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, 40}, {120, 60}})));
      Modelica.Blocks.Interfaces.IntegerOutput Speed_Torque "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, 60}, {120, 80}})));
      Modelica.Blocks.Interfaces.IntegerOutput Forward_Reverse "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, 80}, {120, 100}})));
      Modelica.Blocks.Sources.Constant ServoTorque(k = ServoStartTorque) annotation(
        Placement(transformation(extent = {{-46, -80}, {-26, -60}})));
      Modelica.Blocks.Interfaces.RealOutput Servo_Start_Torque "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, -80}, {120, -60}})));
      Modelica.Blocks.Math.RealToInteger realToInteger annotation(
        Placement(transformation(extent = {{0, 80}, {20, 100}})));
      Modelica.Blocks.Math.RealToInteger realToInteger1 annotation(
        Placement(transformation(extent = {{40, 60}, {60, 80}})));
      Modelica.Blocks.Math.RealToInteger realToInteger2 annotation(
        Placement(transformation(extent = {{40, 20}, {60, 40}})));
      Modelica.Blocks.Math.RealToInteger realToInteger3 annotation(
        Placement(transformation(extent = {{0, -60}, {20, -40}})));
      Modelica.Blocks.Interfaces.IntegerOutput Start_Stop_Servomotor "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, -60}, {120, -40}})));
      Modelica.Blocks.Sources.Constant ForwardAndReverseControlSignal(k = ForwardOrReverseSignal) annotation(
        Placement(transformation(extent = {{-80, 80}, {-60, 100}})));
      Modelica.Blocks.Sources.Constant ServoMotorStartAndStopSignal(k = StopOrStartServoMotor) annotation(
        Placement(transformation(extent = {{-80, -60}, {-60, -40}})));
      Modelica.Blocks.Sources.Constant SpeedAndTorqueControlSignal(k = SpeedTorqueSignal) annotation(
        Placement(transformation(extent = {{-40, 60}, {-20, 80}})));
      Modelica.Blocks.Sources.Constant TestMotorStartAndStopSignal(k = StopOrStartTestMotor) annotation(
        Placement(transformation(extent = {{-40, 20}, {-20, 40}})));
      Modelica.Blocks.Sources.ExpSine TestMotorTorque(amplitude = AmplitudeTorque, f = TorqueSignalFrequency, phase = TorqueSignalPhaseAngle, damping = TorqueSignalDamping, offset = InitialTorque, startTime = TorquePulsStartTime) annotation(
        Placement(transformation(extent = {{-80, 40}, {-60, 60}})));
    equation
      connect(ConstantVoltage.y, Servo_motor_speed) annotation(
        Line(points = {{-59, -88}, {110, -88}}, color = {0, 0, 127}));
      connect(ServoTorque.y, Servo_Start_Torque) annotation(
        Line(points = {{-25, -70}, {110, -70}}, color = {0, 0, 127}));
      connect(realToInteger.y, Forward_Reverse) annotation(
        Line(points = {{21, 90}, {110, 90}}, color = {255, 127, 0}));
      connect(realToInteger1.y, Speed_Torque) annotation(
        Line(points = {{61, 70}, {110, 70}}, color = {255, 127, 0}));
      connect(realToInteger2.y, Start_Stop_Testmotor) annotation(
        Line(points = {{61, 30}, {110, 30}}, color = {255, 127, 0}));
      connect(realToInteger3.y, Start_Stop_Servomotor) annotation(
        Line(points = {{21, -50}, {110, -50}}, color = {255, 127, 0}));
      connect(realToInteger.u, ForwardAndReverseControlSignal.y) annotation(
        Line(points = {{-2, 90}, {-59, 90}}, color = {0, 0, 127}));
      connect(realToInteger1.u, SpeedAndTorqueControlSignal.y) annotation(
        Line(points = {{38, 70}, {-19, 70}}, color = {0, 0, 127}));
      connect(realToInteger2.u, TestMotorStartAndStopSignal.y) annotation(
        Line(points = {{38, 30}, {-19, 30}}, color = {0, 0, 127}));
      connect(realToInteger3.u, ServoMotorStartAndStopSignal.y) annotation(
        Line(points = {{-2, -50}, {-59, -50}}, color = {0, 0, 127}));
      connect(Test_motor_torque, TestMotorTorque.y) annotation(
        Line(points = {{110, 50}, {-59, 50}}, color = {0, 0, 127}));
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = false)),
        Diagram(coordinateSystem(preserveAspectRatio = false)),
        experiment(StopTime = 300));
    end Test_VFD_Servo3_V2;

    model Test_VFD_Servo1_V1
      parameter Real ServoMotorvoltage = 2.5 "Voltage of the Servomotor";
      parameter Integer StopOrStartPulsAmplitudeServoMotor = 1 "Stop (0) and start (1) amplitude signal for the servomotor";
      parameter Integer StopOrStartPulsStartTimeServoMotor = 0 "Time delay for the stop/start signal";
      parameter Integer StopOrStartPulsPeriodServoMotor = 60 "Puls Duration In Seconds for the stop/start signal";
      parameter Integer StopOrStartPulsWidthServoMotor = 100 "Puls width for the stop/start signal";
      parameter Integer StopOrStartPulsOffsetServoMotor = 0 "Initial start value for the stop/start signal";
      parameter Real StartTorqueTime = 5 "Startime of the step up torque in seconds";
      parameter Real HeightTorque = 5 "Height of torque signal";
      parameter Real InitialTorque = 0.61 "The star torque level";
      parameter Real ServoStartTorque = 0.61 "Start torque for the servo motor";
      parameter Integer StopOrStartPulsAmplitudeTestMotor = 1 "Stop (0) and start (1) amplitude signal for the testmotors";
      parameter Integer StopOrStartPulsStartTimeTestMotor = 0 "Time delay for the stop/start signal";
      parameter Integer StopOrStartPulsPeriodTestMotor = 60 "Puls Duration In Seconds for the stop/start signal";
      parameter Integer StopOrStartPulsWidthTestMotor = 100 "Puls width for the stop/start signal";
      parameter Integer StopOrStartPulsOffsetTestMotor = 0 "Initial start value for the stop/start signal";
      parameter Integer SpeedTorquePulsAmplitude = 1 "Speed (0) or Torque (1) amplitude signal for the testmotors";
      parameter Integer SpeedTorquePulsStartTime = 0 "Time delay for the speed/torque signal";
      parameter Integer SpeedTorquePulsPeriod = 60 "Puls Duration In Seconds for the speed/torque signal";
      parameter Integer SpeedTorquePulsWidth = 100 "Puls width for the speed/torque signal";
      parameter Integer SpeedTorquePulsOffset = 0 "Initial start value for the speed/torque signal";
      parameter Integer ForwardOrReversePulsAmplitude = 0 "Forward (0) or Reverse (1) amplitude signal for the testmotors";
      parameter Integer ForwardOrReversePulsStartTime = 0 "Time delay for the forward/reverse signal";
      parameter Integer ForwardOrReversePulsPeriod = 60 "Puls Duration In Seconds for the forward/reverse signal";
      parameter Integer ForwardOrReversePulsWidth = 50 "Puls width for the forward/reverse signal";
      parameter Integer ForwardOrReversePulsOffset = 0 "Initial start value for the forward/reverse signal";
      Modelica.Blocks.Sources.Constant ConstantVoltage(k = ServoMotorvoltage) annotation(
        Placement(transformation(extent = {{-80, -98}, {-60, -78}})));
      Modelica.Blocks.Interfaces.IntegerOutput Start_Stop_Testmotor "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, 20}, {120, 40}})));
      Modelica.Blocks.Interfaces.RealOutput Servo_motor_speed "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, -98}, {120, -78}})));
      Modelica.Blocks.Sources.Step StepUpTorque(height = HeightTorque, offset = InitialTorque, startTime = StartTorqueTime) annotation(
        Placement(transformation(extent = {{-80, 40}, {-60, 60}})));
      Modelica.Blocks.Interfaces.RealOutput Test_motor_torque "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, 40}, {120, 60}})));
      Modelica.Blocks.Interfaces.IntegerOutput Speed_Torque "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, 60}, {120, 80}})));
      Modelica.Blocks.Interfaces.IntegerOutput Forward_Reverse "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, 80}, {120, 100}})));
      Modelica.Blocks.Sources.Constant ServoTorque(k = ServoStartTorque) annotation(
        Placement(transformation(extent = {{-46, -80}, {-26, -60}})));
      Modelica.Blocks.Interfaces.RealOutput Servo_Start_Torque "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, -80}, {120, -60}})));
      Modelica.Blocks.Math.RealToInteger realToInteger annotation(
        Placement(transformation(extent = {{0, 80}, {20, 100}})));
      Modelica.Blocks.Sources.Pulse ForwardOrReversePuls(amplitude = ForwardOrReversePulsAmplitude, width = ForwardOrReversePulsWidth, period = ForwardOrReversePulsPeriod, offset = ForwardOrReversePulsOffset, startTime = ForwardOrReversePulsStartTime) annotation(
        Placement(transformation(extent = {{-80, 80}, {-60, 100}})));
      Modelica.Blocks.Math.RealToInteger realToInteger1 annotation(
        Placement(transformation(extent = {{40, 60}, {60, 80}})));
      Modelica.Blocks.Sources.Pulse SpeedOrTorquePuls(amplitude = SpeedTorquePulsAmplitude, width = SpeedTorquePulsWidth, period = SpeedTorquePulsPeriod, offset = SpeedTorquePulsOffset, startTime = SpeedTorquePulsStartTime) annotation(
        Placement(transformation(extent = {{-40, 60}, {-20, 80}})));
      Modelica.Blocks.Math.RealToInteger realToInteger2 annotation(
        Placement(transformation(extent = {{40, 20}, {60, 40}})));
      Modelica.Blocks.Sources.Pulse StopOrStartPuls(amplitude = StopOrStartPulsAmplitudeTestMotor, width = StopOrStartPulsWidthTestMotor, period = StopOrStartPulsPeriodTestMotor, offset = StopOrStartPulsOffsetTestMotor, startTime = StopOrStartPulsStartTimeTestMotor) annotation(
        Placement(transformation(extent = {{-40, 20}, {-20, 40}})));
      Modelica.Blocks.Math.RealToInteger realToInteger3 annotation(
        Placement(transformation(extent = {{0, -60}, {20, -40}})));
      Modelica.Blocks.Sources.Pulse StopOrStartPuls1(amplitude = StopOrStartPulsAmplitudeServoMotor, width = StopOrStartPulsWidthServoMotor, period = StopOrStartPulsPeriodServoMotor, offset = StopOrStartPulsOffsetServoMotor, startTime = StopOrStartPulsStartTimeServoMotor) annotation(
        Placement(transformation(extent = {{-80, -60}, {-60, -40}})));
      Modelica.Blocks.Interfaces.IntegerOutput Start_Stop_Servomotor "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, -60}, {120, -40}})));
    equation
      connect(ConstantVoltage.y, Servo_motor_speed) annotation(
        Line(points = {{-59, -88}, {110, -88}}, color = {0, 0, 127}));
      connect(StepUpTorque.y, Test_motor_torque) annotation(
        Line(points = {{-59, 50}, {110, 50}}, color = {0, 0, 127}));
      connect(ServoTorque.y, Servo_Start_Torque) annotation(
        Line(points = {{-25, -70}, {110, -70}}, color = {0, 0, 127}));
      connect(realToInteger.y, Forward_Reverse) annotation(
        Line(points = {{21, 90}, {110, 90}}, color = {255, 127, 0}));
      connect(realToInteger.u, ForwardOrReversePuls.y) annotation(
        Line(points = {{-2, 90}, {-59, 90}}, color = {0, 0, 127}));
      connect(realToInteger1.u, SpeedOrTorquePuls.y) annotation(
        Line(points = {{38, 70}, {-19, 70}}, color = {0, 0, 127}));
      connect(realToInteger1.y, Speed_Torque) annotation(
        Line(points = {{61, 70}, {110, 70}}, color = {255, 127, 0}));
      connect(realToInteger2.y, Start_Stop_Testmotor) annotation(
        Line(points = {{61, 30}, {110, 30}}, color = {255, 127, 0}));
      connect(realToInteger2.u, StopOrStartPuls.y) annotation(
        Line(points = {{38, 30}, {-19, 30}}, color = {0, 0, 127}));
      connect(realToInteger3.y, Start_Stop_Servomotor) annotation(
        Line(points = {{21, -50}, {110, -50}}, color = {255, 127, 0}));
      connect(realToInteger3.u, StopOrStartPuls1.y) annotation(
        Line(points = {{-2, -50}, {-59, -50}}, color = {0, 0, 127}));
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = false)),
        Diagram(coordinateSystem(preserveAspectRatio = false)),
        experiment(StopTime = 300));
    end Test_VFD_Servo1_V1;

    model Test_VFD_Servo2_V1
      parameter Real ServoMotorvoltage = 2.5 "Voltage of the Servomotor";
      parameter Integer StopOrStartPulsAmplitudeServoMotor = 1 "Stop (0) and start (1) amplitude signal for the servomotor";
      parameter Integer StopOrStartPulsStartTimeServoMotor = 0 "Time delay for the stop/start signal";
      parameter Integer StopOrStartPulsPeriodServoMotor = 60 "Puls Duration In Seconds for the stop/start signal";
      parameter Integer StopOrStartPulsWidthServoMotor = 100 "Puls width for the stop/start signal";
      parameter Integer StopOrStartPulsOffsetServoMotor = 0 "Initial start value for the stop/start signal";
      parameter Integer StartTorqueRampUpTime = 10 "Startime of the step up torque in seconds";
      parameter Integer TorqueRamUpDuration = 30 "Duration of the Torque ramp up in seconds";
      parameter Real HeightTorque = 5 "Height of torque signal";
      parameter Real InitialTorque = 0.61 "The star torque level";
      parameter Real ServoStartTorque = 0.61 "Start torque for the servo motor";
      parameter Integer StopOrStartPulsAmplitudeTestMotor = 1 "Stop (0) and start (1) amplitude signal for the testmotors";
      parameter Integer StopOrStartPulsStartTimeTestMotor = 0 "Time delay for the stop/start signal";
      parameter Integer StopOrStartPulsPeriodTestMotor = 60 "Puls Duration In Seconds for the stop/start signal";
      parameter Integer StopOrStartPulsWidthTestMotor = 100 "Puls width for the stop/start signal";
      parameter Integer StopOrStartPulsOffsetTestMotor = 0 "Initial start value for the stop/start signal";
      parameter Integer SpeedTorquePulsAmplitude = 1 "Speed (0) or Torque (1) amplitude signal for the testmotors";
      parameter Integer SpeedTorquePulsStartTime = 0 "Time delay for the speed/torque signal";
      parameter Integer SpeedTorquePulsPeriod = 60 "Puls Duration In Seconds for the speed/torque signal";
      parameter Integer SpeedTorquePulsWidth = 100 "Puls width for the speed/torque signal";
      parameter Integer SpeedTorquePulsOffset = 0 "Initial start value for the speed/torque signal";
      parameter Integer ForwardOrReversePulsAmplitude = 0 "Forward (0) or Reverse (1) amplitude signal for the testmotors";
      parameter Integer ForwardOrReversePulsStartTime = 0 "Time delay for the forward/reverse signal";
      parameter Integer ForwardOrReversePulsPeriod = 60 "Puls Duration In Seconds for the forward/reverse signal";
      parameter Integer ForwardOrReversePulsWidth = 50 "Puls width for the forward/reverse signal";
      parameter Integer ForwardOrReversePulsOffset = 0 "Initial start value for the forward/reverse signal";
      Modelica.Blocks.Sources.Constant ConstantVoltage(k = ServoMotorvoltage) annotation(
        Placement(transformation(extent = {{-80, -98}, {-60, -78}})));
      Modelica.Blocks.Interfaces.IntegerOutput Start_Stop_Testmotor "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, 20}, {120, 40}})));
      Modelica.Blocks.Interfaces.RealOutput Servo_motor_speed "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, -98}, {120, -78}})));
      Modelica.Blocks.Interfaces.RealOutput Test_motor_torque "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, 40}, {120, 60}})));
      Modelica.Blocks.Interfaces.IntegerOutput Speed_Torque "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, 60}, {120, 80}})));
      Modelica.Blocks.Interfaces.IntegerOutput Forward_Reverse "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, 80}, {120, 100}})));
      Modelica.Blocks.Sources.Constant ServoTorque(k = ServoStartTorque) annotation(
        Placement(transformation(extent = {{-46, -80}, {-26, -60}})));
      Modelica.Blocks.Interfaces.RealOutput Servo_Start_Torque "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, -80}, {120, -60}})));
      Modelica.Blocks.Math.RealToInteger realToInteger annotation(
        Placement(transformation(extent = {{0, 80}, {20, 100}})));
      Modelica.Blocks.Sources.Pulse ForwardOrReversePuls(amplitude = ForwardOrReversePulsAmplitude, width = ForwardOrReversePulsWidth, period = ForwardOrReversePulsPeriod, offset = ForwardOrReversePulsOffset, startTime = ForwardOrReversePulsStartTime) annotation(
        Placement(transformation(extent = {{-80, 80}, {-60, 100}})));
      Modelica.Blocks.Math.RealToInteger realToInteger1 annotation(
        Placement(transformation(extent = {{40, 60}, {60, 80}})));
      Modelica.Blocks.Sources.Pulse SpeedOrTorquePuls(amplitude = SpeedTorquePulsAmplitude, width = SpeedTorquePulsWidth, period = SpeedTorquePulsPeriod, offset = SpeedTorquePulsOffset, startTime = SpeedTorquePulsStartTime) annotation(
        Placement(transformation(extent = {{-40, 60}, {-20, 80}})));
      Modelica.Blocks.Math.RealToInteger realToInteger2 annotation(
        Placement(transformation(extent = {{40, 20}, {60, 40}})));
      Modelica.Blocks.Sources.Pulse StopOrStartPuls(amplitude = StopOrStartPulsAmplitudeTestMotor, width = StopOrStartPulsWidthTestMotor, period = StopOrStartPulsPeriodTestMotor, offset = StopOrStartPulsOffsetTestMotor, startTime = StopOrStartPulsStartTimeTestMotor) annotation(
        Placement(transformation(extent = {{-40, 20}, {-20, 40}})));
      Modelica.Blocks.Math.RealToInteger realToInteger3 annotation(
        Placement(transformation(extent = {{0, -60}, {20, -40}})));
      Modelica.Blocks.Sources.Pulse StopOrStartPuls1(amplitude = StopOrStartPulsAmplitudeServoMotor, width = StopOrStartPulsWidthServoMotor, period = StopOrStartPulsPeriodServoMotor, offset = StopOrStartPulsOffsetServoMotor, startTime = StopOrStartPulsStartTimeServoMotor) annotation(
        Placement(transformation(extent = {{-80, -60}, {-60, -40}})));
      Modelica.Blocks.Interfaces.IntegerOutput Start_Stop_Servomotor "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, -60}, {120, -40}})));
      Modelica.Blocks.Sources.Ramp TestMotorTorque(height = HeightTorque, duration = TorqueRamUpDuration, offset = InitialTorque, startTime = StartTorqueRampUpTime) annotation(
        Placement(transformation(extent = {{-80, 40}, {-60, 60}})));
    equation
      connect(ConstantVoltage.y, Servo_motor_speed) annotation(
        Line(points = {{-59, -88}, {110, -88}}, color = {0, 0, 127}));
      connect(ServoTorque.y, Servo_Start_Torque) annotation(
        Line(points = {{-25, -70}, {110, -70}}, color = {0, 0, 127}));
      connect(realToInteger.y, Forward_Reverse) annotation(
        Line(points = {{21, 90}, {110, 90}}, color = {255, 127, 0}));
      connect(realToInteger.u, ForwardOrReversePuls.y) annotation(
        Line(points = {{-2, 90}, {-59, 90}}, color = {0, 0, 127}));
      connect(realToInteger1.u, SpeedOrTorquePuls.y) annotation(
        Line(points = {{38, 70}, {-19, 70}}, color = {0, 0, 127}));
      connect(realToInteger1.y, Speed_Torque) annotation(
        Line(points = {{61, 70}, {110, 70}}, color = {255, 127, 0}));
      connect(realToInteger2.y, Start_Stop_Testmotor) annotation(
        Line(points = {{61, 30}, {110, 30}}, color = {255, 127, 0}));
      connect(realToInteger2.u, StopOrStartPuls.y) annotation(
        Line(points = {{38, 30}, {-19, 30}}, color = {0, 0, 127}));
      connect(realToInteger3.y, Start_Stop_Servomotor) annotation(
        Line(points = {{21, -50}, {110, -50}}, color = {255, 127, 0}));
      connect(realToInteger3.u, StopOrStartPuls1.y) annotation(
        Line(points = {{-2, -50}, {-59, -50}}, color = {0, 0, 127}));
      connect(TestMotorTorque.y, Test_motor_torque) annotation(
        Line(points = {{-59, 50}, {110, 50}}, color = {0, 0, 127}));
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = false)),
        Diagram(coordinateSystem(preserveAspectRatio = false)),
        experiment(StopTime = 300));
    end Test_VFD_Servo2_V1;

    model Test_VFD_Servo3_V1
      parameter Real ServoMotorvoltage = 2.5 "Voltage of the Servomotor";
      parameter Integer StopOrStartPulsAmplitudeServoMotor = 1 "Stop (0) and start (1) amplitude signal for the servomotor";
      parameter Integer StopOrStartPulsStartTimeServoMotor = 0 "Time delay for the stop/start signal";
      parameter Integer StopOrStartPulsPeriodServoMotor = 60 "Puls Duration In Seconds for the stop/start signal";
      parameter Integer StopOrStartPulsWidthServoMotor = 100 "Puls width for the stop/start signal";
      parameter Integer StopOrStartPulsOffsetServoMotor = 0 "Initial start value for the stop/start signal";
      parameter Real TorquePulsStartTime = 5 "Startime of the torque puls signal";
      parameter Real AmplitudeTorque = 5 "Height of torque puls signal";
      parameter Real TorqueSignalDamping = 0.2 "Damping of the torque signal";
      parameter Real TorqueSignalFrequency = 2 "Frequency of the torque signal";
      parameter Real TorqueSignalPhaseAngle = 30 "The phase angle of the Torque signal";
      parameter Real InitialTorque = 0.61 "The star torque level";
      parameter Real ServoStartTorque = 0.61 "Start torque for the servo motor";
      parameter Integer StopOrStartPulsAmplitudeTestMotor = 1 "Stop (0) and start (1) amplitude signal for the testmotors";
      parameter Integer StopOrStartPulsStartTimeTestMotor = 0 "Time delay for the stop/start signal";
      parameter Integer StopOrStartPulsPeriodTestMotor = 60 "Puls Duration In Seconds for the stop/start signal";
      parameter Integer StopOrStartPulsWidthTestMotor = 100 "Puls width for the stop/start signal";
      parameter Integer StopOrStartPulsOffsetTestMotor = 0 "Initial start value for the stop/start signal";
      parameter Integer SpeedTorquePulsAmplitude = 1 "Speed (0) or Torque (1) amplitude signal for the testmotors";
      parameter Integer SpeedTorquePulsStartTime = 0 "Time delay for the speed/torque signal";
      parameter Integer SpeedTorquePulsPeriod = 60 "Puls Duration In Seconds for the speed/torque signal";
      parameter Integer SpeedTorquePulsWidth = 100 "Puls width for the speed/torque signal";
      parameter Integer SpeedTorquePulsOffset = 0 "Initial start value for the speed/torque signal";
      parameter Integer ForwardOrReversePulsAmplitude = 0 "Forward (0) or Reverse (1) amplitude signal for the testmotors";
      parameter Integer ForwardOrReversePulsStartTime = 0 "Time delay for the forward/reverse signal";
      parameter Integer ForwardOrReversePulsPeriod = 60 "Puls Duration In Seconds for the forward/reverse signal";
      parameter Integer ForwardOrReversePulsWidth = 50 "Puls width for the forward/reverse signal";
      parameter Integer ForwardOrReversePulsOffset = 0 "Initial start value for the forward/reverse signal";
      Modelica.Blocks.Sources.Constant ConstantVoltage(k = ServoMotorvoltage) annotation(
        Placement(transformation(extent = {{-80, -98}, {-60, -78}})));
      Modelica.Blocks.Interfaces.IntegerOutput Start_Stop_Testmotor "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, 20}, {120, 40}})));
      Modelica.Blocks.Interfaces.RealOutput Servo_motor_speed "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, -98}, {120, -78}})));
      Modelica.Blocks.Interfaces.RealOutput Test_motor_torque "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, 40}, {120, 60}})));
      Modelica.Blocks.Interfaces.IntegerOutput Speed_Torque "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, 60}, {120, 80}})));
      Modelica.Blocks.Interfaces.IntegerOutput Forward_Reverse "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, 80}, {120, 100}})));
      Modelica.Blocks.Sources.Constant ServoTorque(k = ServoStartTorque) annotation(
        Placement(transformation(extent = {{-46, -80}, {-26, -60}})));
      Modelica.Blocks.Interfaces.RealOutput Servo_Start_Torque "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, -80}, {120, -60}})));
      Modelica.Blocks.Math.RealToInteger realToInteger annotation(
        Placement(transformation(extent = {{0, 80}, {20, 100}})));
      Modelica.Blocks.Sources.Pulse ForwardOrReversePuls(amplitude = ForwardOrReversePulsAmplitude, width = ForwardOrReversePulsWidth, period = ForwardOrReversePulsPeriod, offset = ForwardOrReversePulsOffset, startTime = ForwardOrReversePulsStartTime) annotation(
        Placement(transformation(extent = {{-80, 80}, {-60, 100}})));
      Modelica.Blocks.Math.RealToInteger realToInteger1 annotation(
        Placement(transformation(extent = {{40, 60}, {60, 80}})));
      Modelica.Blocks.Sources.Pulse SpeedOrTorquePuls(amplitude = SpeedTorquePulsAmplitude, width = SpeedTorquePulsWidth, period = SpeedTorquePulsPeriod, offset = SpeedTorquePulsOffset, startTime = SpeedTorquePulsStartTime) annotation(
        Placement(transformation(extent = {{-40, 60}, {-20, 80}})));
      Modelica.Blocks.Math.RealToInteger realToInteger2 annotation(
        Placement(transformation(extent = {{40, 20}, {60, 40}})));
      Modelica.Blocks.Sources.Pulse StopOrStartPuls(amplitude = StopOrStartPulsAmplitudeTestMotor, width = StopOrStartPulsWidthTestMotor, period = StopOrStartPulsPeriodTestMotor, offset = StopOrStartPulsOffsetTestMotor, startTime = StopOrStartPulsStartTimeTestMotor) annotation(
        Placement(transformation(extent = {{-40, 20}, {-20, 40}})));
      Modelica.Blocks.Math.RealToInteger realToInteger3 annotation(
        Placement(transformation(extent = {{0, -60}, {20, -40}})));
      Modelica.Blocks.Sources.Pulse StopOrStartPuls1(amplitude = StopOrStartPulsAmplitudeServoMotor, width = StopOrStartPulsWidthServoMotor, period = StopOrStartPulsPeriodServoMotor, offset = StopOrStartPulsOffsetServoMotor, startTime = StopOrStartPulsStartTimeServoMotor) annotation(
        Placement(transformation(extent = {{-80, -60}, {-60, -40}})));
      Modelica.Blocks.Interfaces.IntegerOutput Start_Stop_Servomotor "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, -60}, {120, -40}})));
      Modelica.Blocks.Sources.ExpSine TestMotorTorque(amplitude = AmplitudeTorque, f = TorqueSignalFrequency, phase = TorqueSignalPhaseAngle, damping = TorqueSignalDamping, offset = InitialTorque, startTime = TorquePulsStartTime) annotation(
        Placement(transformation(extent = {{-80, 40}, {-60, 60}})));
    equation
      connect(ConstantVoltage.y, Servo_motor_speed) annotation(
        Line(points = {{-59, -88}, {110, -88}}, color = {0, 0, 127}));
      connect(ServoTorque.y, Servo_Start_Torque) annotation(
        Line(points = {{-25, -70}, {110, -70}}, color = {0, 0, 127}));
      connect(realToInteger.y, Forward_Reverse) annotation(
        Line(points = {{21, 90}, {110, 90}}, color = {255, 127, 0}));
      connect(realToInteger.u, ForwardOrReversePuls.y) annotation(
        Line(points = {{-2, 90}, {-59, 90}}, color = {0, 0, 127}));
      connect(realToInteger1.u, SpeedOrTorquePuls.y) annotation(
        Line(points = {{38, 70}, {-19, 70}}, color = {0, 0, 127}));
      connect(realToInteger1.y, Speed_Torque) annotation(
        Line(points = {{61, 70}, {110, 70}}, color = {255, 127, 0}));
      connect(realToInteger2.y, Start_Stop_Testmotor) annotation(
        Line(points = {{61, 30}, {110, 30}}, color = {255, 127, 0}));
      connect(realToInteger2.u, StopOrStartPuls.y) annotation(
        Line(points = {{38, 30}, {-19, 30}}, color = {0, 0, 127}));
      connect(realToInteger3.y, Start_Stop_Servomotor) annotation(
        Line(points = {{21, -50}, {110, -50}}, color = {255, 127, 0}));
      connect(realToInteger3.u, StopOrStartPuls1.y) annotation(
        Line(points = {{-2, -50}, {-59, -50}}, color = {0, 0, 127}));
      connect(TestMotorTorque.y, Test_motor_torque) annotation(
        Line(points = {{-59, 50}, {110, 50}}, color = {0, 0, 127}));
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = false)),
        Diagram(coordinateSystem(preserveAspectRatio = false)),
        experiment(StopTime = 300));
    end Test_VFD_Servo3_V1;

    model VFDAndServoControl1
      parameter Real ServoMotorvoltage = 2.5 "Voltage of the Servomotor";
      parameter Real ServoStartTorque = 0.02 "Start torque for the servo motor";
      parameter Real TestMotorTorque = 5 "Torque signal sendt to the test motor";
      Modelica.Blocks.Sources.Constant ConstantVoltage(k = ServoMotorvoltage) annotation(
        Placement(transformation(extent = {{-80, -98}, {-60, -78}})));
      Modelica.Blocks.Interfaces.IntegerOutput Start_Stop_Testmotor "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, 20}, {120, 40}})));
      Modelica.Blocks.Interfaces.RealOutput Servo_motor_speed "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, -98}, {120, -78}})));
      Modelica.Blocks.Interfaces.RealOutput Test_motor_torque "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, 40}, {120, 60}})));
      Modelica.Blocks.Sources.Constant ServoTorque(k = ServoStartTorque) annotation(
        Placement(transformation(extent = {{-46, -80}, {-26, -60}})));
      Modelica.Blocks.Interfaces.RealOutput Servo_Start_Torque "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, -80}, {120, -60}})));
      Modelica.Blocks.Interfaces.IntegerOutput Start_Stop_Servomotor "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, -60}, {120, -40}})));
      Modelica.Blocks.Logical.GreaterThreshold greaterThreshold annotation(
        Placement(transformation(extent = {{-14, -64}, {6, -44}})));
      Modelica.Blocks.Math.BooleanToInteger booleanToInteger annotation(
        Placement(transformation(extent = {{60, -60}, {80, -40}})));
      Modelica.Blocks.Logical.And and1 annotation(
        Placement(transformation(extent = {{28, -60}, {48, -40}})));
      Modelica.Blocks.Logical.GreaterThreshold greaterThreshold1 annotation(
        Placement(transformation(extent = {{-14, -34}, {6, -14}})));
      Modelica.Blocks.Logical.GreaterThreshold greaterThreshold2 annotation(
        Placement(transformation(extent = {{26, 20}, {46, 40}})));
      Modelica.Blocks.Math.BooleanToInteger booleanToInteger1 annotation(
        Placement(transformation(extent = {{66, 20}, {86, 40}})));
      Modelica.Blocks.Sources.Constant TestMotortorque(k = TestMotorTorque) annotation(
        Placement(transformation(extent = {{-80, 40}, {-60, 60}})));
    equation
      connect(ConstantVoltage.y, Servo_motor_speed) annotation(
        Line(points = {{-59, -88}, {110, -88}}, color = {0, 0, 127}));
      connect(ServoTorque.y, Servo_Start_Torque) annotation(
        Line(points = {{-25, -70}, {110, -70}}, color = {0, 0, 127}));
      connect(booleanToInteger.u, and1.y) annotation(
        Line(points = {{58, -50}, {49, -50}}, color = {255, 0, 255}));
      connect(booleanToInteger.y, Start_Stop_Servomotor) annotation(
        Line(points = {{81, -50}, {110, -50}}, color = {255, 127, 0}));
      connect(and1.u1, greaterThreshold1.y) annotation(
        Line(points = {{26, -50}, {14, -50}, {14, -24}, {7, -24}}, color = {255, 0, 255}));
      connect(and1.u2, greaterThreshold.y) annotation(
        Line(points = {{26, -58}, {14, -58}, {14, -54}, {7, -54}}, color = {255, 0, 255}));
      connect(ServoTorque.y, greaterThreshold.u) annotation(
        Line(points = {{-25, -70}, {-24, -70}, {-24, -54}, {-16, -54}}, color = {0, 0, 127}));
      connect(ConstantVoltage.y, greaterThreshold1.u) annotation(
        Line(points = {{-59, -88}, {-59, -24}, {-16, -24}}, color = {0, 0, 127}));
      connect(greaterThreshold2.y, booleanToInteger1.u) annotation(
        Line(points = {{47, 30}, {64, 30}}, color = {255, 0, 255}));
      connect(Start_Stop_Testmotor, booleanToInteger1.y) annotation(
        Line(points = {{110, 30}, {87, 30}}, color = {255, 127, 0}));
      connect(TestMotortorque.y, Test_motor_torque) annotation(
        Line(points = {{-59, 50}, {110, 50}}, color = {0, 0, 127}));
      connect(greaterThreshold2.u, Test_motor_torque) annotation(
        Line(points = {{24, 30}, {-40, 30}, {-40, 50}, {110, 50}}, color = {0, 0, 127}));
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = false)),
        Diagram(coordinateSystem(preserveAspectRatio = false)),
        experiment(StopTime = 300));
    end VFDAndServoControl1;

    model VFDAndServoControl3
      extends ActiveWork.MotorTest.VFDAndServoControl2;
      Modelica.Blocks.Math.BooleanToInteger booleanToInteger2 annotation(
        Placement(transformation(extent = {{68, -18}, {88, 2}})));
      Modelica.Blocks.Interfaces.IntegerOutput Forward_Reverse_Testmotor "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, -18}, {120, 2}})));
      Modelica.Blocks.Logical.LessEqualThreshold lessEqualThreshold annotation(
        Placement(transformation(extent = {{34, -18}, {54, 2}})));
    equation
      connect(booleanToInteger2.y, Forward_Reverse_Testmotor) annotation(
        Line(points = {{89, -8}, {110, -8}}, color = {255, 127, 0}));
      connect(booleanToInteger2.u, lessEqualThreshold.y) annotation(
        Line(points = {{66, -8}, {55, -8}}, color = {255, 0, 255}));
      connect(lessEqualThreshold.u, Test_motor_torque) annotation(
        Line(points = {{32, -8}, {10, -8}, {10, -4}, {-18, -4}, {-18, 30}, {-40, 30}, {-40, 50}, {110, 50}}, color = {0, 0, 127}));
      annotation(
        experiment(StartTime = 0, StopTime = 300, Tolerance = 1e-6, Interval = 0.6));
    end VFDAndServoControl3;

    model ConfigTestVDF
      parameter Real TestMotorTorqueAmplitude = 5 "Torque signal sendt to the test motor";
      parameter Real TestMotorTorqueStartTime = 10 "Ramp up start time";
      parameter Real TestMotorTorqueDuration = 10 "Duration of the ramp up";
      parameter Real TestMotorTorqueOffset = 0 "Initial torque value sendt to the test motot";
      Modelica.Blocks.Sources.Ramp TestMotortorque(duration = TestMotorTorqueDuration, height = TestMotorTorqueAmplitude, offset = TestMotorTorqueOffset, startTime = TestMotorTorqueStartTime) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-80, 40}, {-60, 60}}, rotation = 0)));
      Modelica.Blocks.Logical.GreaterThreshold greaterThreshold2 annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{26, 20}, {46, 40}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput Test_motor_torque annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{100, 40}, {120, 60}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{100, 40}, {120, 60}}, rotation = 0)));
      Modelica.Blocks.Math.BooleanToInteger booleanToInteger1 annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{66, 20}, {86, 40}}, rotation = 0)));
      Modelica.Blocks.Interfaces.IntegerOutput Start_Stop_Testmotor annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{100, 20}, {120, 40}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{100, 20}, {120, 40}}, rotation = 0)));
    equation
      connect(greaterThreshold2.y, booleanToInteger1.u) annotation(
        Line(points = {{47, 30}, {64, 30}}, color = {255, 0, 255}));
      connect(Start_Stop_Testmotor, booleanToInteger1.y) annotation(
        Line(points = {{110, 30}, {87, 30}}, color = {255, 127, 0}));
      connect(greaterThreshold2.u, Test_motor_torque) annotation(
        Line(points = {{24, 30}, {-40, 30}, {-40, 50}, {110, 50}}, color = {0, 0, 127}));
      connect(TestMotortorque.y, Test_motor_torque) annotation(
        Line(points = {{-59, 50}, {110, 50}}, color = {0, 0, 127}));
      annotation(
        experiment(StartTime = 0, StopTime = 300, Tolerance = 1e-6, Interval = 0.6));
    end ConfigTestVDF;

    model ConfigTestVDF1
      parameter Real TestMotorTorqueAmplitude = 5 "Torque signal sendt to the test motor";
      parameter Integer TestMotorTorqueStartTime = 10 "Ramp up start time";
      parameter Integer TestMotorTorqueHeight = 1 "Duration of the ramp up";
      parameter Integer TestMotorTorqueOffset = 0 "Initial torque value sendt to the test motot";
      Modelica.Blocks.Interfaces.RealOutput Test_motor_torque annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{100, 40}, {120, 60}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{100, 40}, {120, 60}}, rotation = 0)));
      Modelica.Blocks.Interfaces.IntegerOutput Start_Stop_Testmotor annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{100, 20}, {120, 40}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{100, 20}, {120, 40}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant const(k = TestMotorTorqueAmplitude) annotation(
        Placement(visible = true, transformation(origin = {-44, 52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.IntegerStep integerStep(height = TestMotorTorqueHeight, offset = TestMotorTorqueOffset, startTime = TestMotorTorqueStartTime) annotation(
        Placement(visible = true, transformation(origin = {-42, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(const.y, Test_motor_torque) annotation(
        Line(points = {{-33, 52}, {110, 52}, {110, 50}}, color = {0, 0, 127}));
      connect(integerStep.y, Start_Stop_Testmotor) annotation(
        Line(points = {{-31, 12}, {86, 12}, {86, 30}, {110, 30}}, color = {255, 127, 0}));
      annotation(
        __OpenModelica_simulationFlags(lv = "stdout,assert,LOG_STATS", s = "dassl", variableFilter = ".*"),
        experiment(StartTime = 0, StopTime = 300, Tolerance = 1e-6, Interval = 0.6));
    end ConfigTestVDF1;

    model ConfigTestVDF2
      parameter Real SinHeight = 2 "Height of the sin wave";
      parameter Real SinOffset = 5 "Offset of the sin wave";
      parameter Real SinFrequency = 0.1 "Frequency of the sin wave";
      parameter Integer TestMotorTorqueStartTime = 1 "Ramp up start time";
      parameter Integer TestMotorTorqueHeight = 1 "Duration of the ramp up";
      parameter Integer TestMotorTorqueOffset = 0 "Initial torque value sendt to the test motot";
      Modelica.Blocks.Interfaces.RealOutput Test_motor_torque annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{100, 40}, {120, 60}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{100, 40}, {120, 60}}, rotation = 0)));
      Modelica.Blocks.Interfaces.IntegerOutput Start_Stop_Testmotor annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{100, 20}, {120, 40}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{100, 20}, {120, 40}}, rotation = 0)));
      Modelica.Blocks.Sources.IntegerStep integerStep(height = TestMotorTorqueHeight, offset = TestMotorTorqueOffset, startTime = TestMotorTorqueStartTime) annotation(
        Placement(visible = true, transformation(origin = {-42, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Sine sine(amplitude = SinHeight, f = SinFrequency, offset = SinOffset) annotation(
        Placement(visible = true, transformation(origin = {-46, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(integerStep.y, Start_Stop_Testmotor) annotation(
        Line(points = {{-31, 12}, {86, 12}, {86, 30}, {110, 30}}, color = {255, 127, 0}));
      connect(sine.y, Test_motor_torque) annotation(
        Line(points = {{-35, 54}, {110, 54}, {110, 50}}, color = {0, 0, 127}));
      annotation(
        __OpenModelica_simulationFlags(lv = "stdout,assert,LOG_STATS", s = "dassl", variableFilter = ".*"),
        experiment(StartTime = 0, StopTime = 300, Tolerance = 1e-06, Interval = 0.6));
    end ConfigTestVDF2;

    model ConfigTestVDF12
      parameter Real TestMotorTorqueAmplitude = 5 "Torque signal sendt to the test motor";
      parameter Integer TestMotorTorqueStartTime = 10 "Ramp up start time";
      parameter Integer TestMotorTorqueHeight = 1 "Duration of the ramp up";
      parameter Integer TestMotorTorqueOffset = 0 "Initial torque value sendt to the test motot";
      Modelica.Blocks.Interfaces.RealOutput Test_motor_torque annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{100, 40}, {120, 60}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{100, 40}, {120, 60}}, rotation = 0)));
      Modelica.Blocks.Interfaces.IntegerOutput Start_Stop_Testmotor annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{100, 20}, {120, 40}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{100, 20}, {120, 40}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant const(k = TestMotorTorqueAmplitude) annotation(
        Placement(visible = true, transformation(origin = {-44, 52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.IntegerStep integerStep(height = TestMotorTorqueHeight, offset = TestMotorTorqueOffset, startTime = TestMotorTorqueStartTime) annotation(
        Placement(visible = true, transformation(origin = {-42, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput InputVoltage annotation(
        Placement(visible = true, transformation(origin = {-120, -26}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-102, -30}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Math.Gain gain(k = 1) annotation(
        Placement(visible = true, transformation(origin = {-34, -26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(const.y, Test_motor_torque) annotation(
        Line(points = {{-33, 52}, {110, 52}, {110, 50}}, color = {0, 0, 127}));
      connect(integerStep.y, Start_Stop_Testmotor) annotation(
        Line(points = {{-31, 12}, {86, 12}, {86, 30}, {110, 30}}, color = {255, 127, 0}));
      connect(gain.u, InputVoltage) annotation(
        Line(points = {{-46, -26}, {-120, -26}}, color = {0, 0, 127}));
      annotation(
        __OpenModelica_simulationFlags(lv = "stdout,assert,LOG_STATS", s = "dassl", variableFilter = ".*"),
        experiment(StartTime = 0, StopTime = 300, Tolerance = 1e-6, Interval = 0.6));
    end ConfigTestVDF12;

    model Test2
      parameter Real ServoMotorvoltage = 2.5 "Voltage of the Servomotor";
      parameter Real ServoMotorTorqueOffset = 0.05 "Start torque for the servo motor";
      parameter Real ServoMotorTorqueStartTime = 2 "Start time of ramping down the start torque";
      parameter Real ServoMotorTorqueDuration = 1 "Duration of the torque ramp down";
      parameter Real ServoMotorTorqueAmplitude = 0.15 "Amplitude of ramping down";
      parameter Real TestMotorTorqueAmplitude = 10 "Torque signal sendt to the test motor";
      parameter Real TestMotorTorqueStartTime = 5 "Ramp up start time";
      parameter Real TestMotorTorqueDuration = 1 "Duration of the ramp up";
      parameter Real TestMotorTorqueOffset = -10 "Initial torque value sendt to the test motot";
      Modelica.Blocks.Sources.Constant ConstantVoltage(k = ServoMotorvoltage) annotation(
        Placement(transformation(extent = {{-80, -98}, {-60, -78}})));
      Modelica.Blocks.Interfaces.IntegerOutput Start_Stop_Testmotor "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, 20}, {120, 40}})));
      Modelica.Blocks.Interfaces.RealOutput Servo_motor_speed "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, -98}, {120, -78}})));
      Modelica.Blocks.Interfaces.RealOutput Test_motor_torque "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, 40}, {120, 60}})));
      Modelica.Blocks.Interfaces.RealOutput Servo_Start_Torque "Connector of Real output signal" annotation(
        Placement(transformation(extent = {{100, -80}, {120, -60}})));
      Modelica.Blocks.Interfaces.IntegerOutput Start_Stop_Servomotor "Connector of Integer output signal" annotation(
        Placement(transformation(extent = {{100, -60}, {120, -40}})));
      Modelica.Blocks.Math.BooleanToInteger booleanToInteger annotation(
        Placement(transformation(extent = {{60, -60}, {80, -40}})));
      Modelica.Blocks.Logical.GreaterThreshold greaterThreshold1 annotation(
        Placement(visible = true, transformation(origin = {24, -26}, extent = {{-14, -34}, {6, -14}}, rotation = 0)));
      Modelica.Blocks.Math.BooleanToInteger booleanToInteger1 annotation(
        Placement(transformation(extent = {{66, 20}, {86, 40}})));
      Modelica.Blocks.Sources.Ramp TestMotortorque(height = TestMotorTorqueAmplitude, duration = TestMotorTorqueDuration, offset = TestMotorTorqueOffset, startTime = TestMotorTorqueStartTime) annotation(
        Placement(transformation(extent = {{-80, 40}, {-60, 60}})));
      Modelica.Blocks.Sources.Ramp rampDown(duration = ServoMotorTorqueDuration, height = ServoMotorTorqueAmplitude, offset = ServoMotorTorqueOffset, startTime = ServoMotorTorqueStartTime) annotation(
        Placement(visible = true, transformation(origin = {30, -120}, extent = {{-80, 40}, {-60, 60}}, rotation = 0)));
      Modelica.Blocks.Logical.Less less annotation(
        Placement(visible = true, transformation(origin = {26, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant constant1(k = 0) annotation(
        Placement(visible = true, transformation(origin = {28, 104}, extent = {{-80, -98}, {-60, -78}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput y annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {122, -8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Ramp ramp(duration = 1, height = 0.2, offset = 0, startTime = 20) annotation(
        Placement(visible = true, transformation(origin = {64, -50}, extent = {{-80, 40}, {-60, 60}}, rotation = 0)));
    equation
      connect(ConstantVoltage.y, Servo_motor_speed) annotation(
        Line(points = {{-59, -88}, {110, -88}}, color = {0, 0, 127}));
      connect(booleanToInteger.y, Start_Stop_Servomotor) annotation(
        Line(points = {{81, -50}, {110, -50}}, color = {255, 127, 0}));
      connect(ConstantVoltage.y, greaterThreshold1.u) annotation(
        Line(points = {{-59, -88}, {-59, -50}, {8, -50}}, color = {0, 0, 127}));
      connect(Start_Stop_Testmotor, booleanToInteger1.y) annotation(
        Line(points = {{110, 30}, {87, 30}}, color = {255, 127, 0}));
      connect(TestMotortorque.y, Test_motor_torque) annotation(
        Line(points = {{-59, 50}, {110, 50}}, color = {0, 0, 127}));
      connect(booleanToInteger.u, greaterThreshold1.y) annotation(
        Line(points = {{58, -50}, {31, -50}}, color = {255, 0, 255}));
      connect(rampDown.y, Servo_Start_Torque) annotation(
        Line(points = {{-29, -70}, {110, -70}}, color = {0, 0, 127}));
      connect(booleanToInteger1.u, less.y) annotation(
        Line(points = {{64, 30}, {37, 30}}, color = {255, 0, 255}));
      connect(constant1.y, less.u2) annotation(
        Line(points = {{-31, 16}, {-31, 22}, {14, 22}}, color = {0, 0, 127}));
      connect(less.u1, TestMotortorque.y) annotation(
        Line(points = {{14, 30}, {-16, 30}, {-16, 50}, {-59, 50}}, color = {0, 0, 127}));
      connect(ramp.y, y) annotation(
        Line(points = {{5, 0}, {110, 0}}, color = {0, 0, 127}));
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = false)),
        Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})),
        experiment(StopTime = 300, StartTime = 0, Tolerance = 1e-06, Interval = 0.6));
    end Test2;
  end Old_Models;

  package ActiveWork
    package MotorTest
      model VFDAndServoControl2
        parameter Real ServoMotorvoltage = 2.5 "Voltage of the Servomotor";
        parameter Real ServoStartTorque = 0.05 "Start torque for the servo motor";
        parameter Real TestMotorTorqueAmplitude = 5 "Torque signal sendt to the test motor";
        parameter Real TestMotorTorqueStartTime = 10 "Ramp up start time";
        parameter Real TestMotorTorqueDuration = 10 "Duration of the ramp up";
        parameter Real TestMotorTorqueOffset = 0 "Initial torque value sendt to the test motot";
        Modelica.Blocks.Sources.Constant ConstantVoltage(k = ServoMotorvoltage) annotation(
          Placement(transformation(extent = {{-80, -98}, {-60, -78}})));
        Modelica.Blocks.Interfaces.IntegerOutput Start_Stop_Testmotor "Connector of Integer output signal" annotation(
          Placement(transformation(extent = {{100, 20}, {120, 40}})));
        Modelica.Blocks.Interfaces.RealOutput Servo_motor_speed "Connector of Real output signal" annotation(
          Placement(transformation(extent = {{100, -98}, {120, -78}})));
        Modelica.Blocks.Interfaces.RealOutput Test_motor_torque "Connector of Real output signal" annotation(
          Placement(transformation(extent = {{100, 40}, {120, 60}})));
        Modelica.Blocks.Sources.Constant ServoTorque(k = ServoStartTorque) annotation(
          Placement(transformation(extent = {{-46, -80}, {-26, -60}})));
        Modelica.Blocks.Interfaces.RealOutput Servo_Start_Torque "Connector of Real output signal" annotation(
          Placement(transformation(extent = {{100, -80}, {120, -60}})));
        Modelica.Blocks.Interfaces.IntegerOutput Start_Stop_Servomotor "Connector of Integer output signal" annotation(
          Placement(transformation(extent = {{100, -60}, {120, -40}})));
        Modelica.Blocks.Logical.GreaterThreshold greaterThreshold annotation(
          Placement(transformation(extent = {{-14, -64}, {6, -44}})));
        Modelica.Blocks.Math.BooleanToInteger booleanToInteger annotation(
          Placement(transformation(extent = {{60, -60}, {80, -40}})));
        Modelica.Blocks.Logical.And and1 annotation(
          Placement(transformation(extent = {{28, -60}, {48, -40}})));
        Modelica.Blocks.Logical.GreaterThreshold greaterThreshold1 annotation(
          Placement(transformation(extent = {{-14, -34}, {6, -14}})));
        Modelica.Blocks.Logical.GreaterThreshold greaterThreshold2 annotation(
          Placement(transformation(extent = {{26, 20}, {46, 40}})));
        Modelica.Blocks.Math.BooleanToInteger booleanToInteger1 annotation(
          Placement(transformation(extent = {{66, 20}, {86, 40}})));
        Modelica.Blocks.Sources.Ramp TestMotortorque(height = TestMotorTorqueAmplitude, duration = TestMotorTorqueDuration, offset = TestMotorTorqueOffset, startTime = TestMotorTorqueStartTime) annotation(
          Placement(transformation(extent = {{-80, 40}, {-60, 60}})));
  Modelica.Blocks.Sources.Step step annotation(
          Placement(visible = true, transformation(origin = {-70, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(ConstantVoltage.y, Servo_motor_speed) annotation(
          Line(points = {{-59, -88}, {110, -88}}, color = {0, 0, 127}));
        connect(ServoTorque.y, Servo_Start_Torque) annotation(
          Line(points = {{-25, -70}, {110, -70}}, color = {0, 0, 127}));
        connect(booleanToInteger.u, and1.y) annotation(
          Line(points = {{58, -50}, {49, -50}}, color = {255, 0, 255}));
        connect(booleanToInteger.y, Start_Stop_Servomotor) annotation(
          Line(points = {{81, -50}, {110, -50}}, color = {255, 127, 0}));
        connect(and1.u1, greaterThreshold1.y) annotation(
          Line(points = {{26, -50}, {14, -50}, {14, -24}, {7, -24}}, color = {255, 0, 255}));
        connect(and1.u2, greaterThreshold.y) annotation(
          Line(points = {{26, -58}, {14, -58}, {14, -54}, {7, -54}}, color = {255, 0, 255}));
        connect(ServoTorque.y, greaterThreshold.u) annotation(
          Line(points = {{-25, -70}, {-24, -70}, {-24, -54}, {-16, -54}}, color = {0, 0, 127}));
        connect(ConstantVoltage.y, greaterThreshold1.u) annotation(
          Line(points = {{-59, -88}, {-59, -24}, {-16, -24}}, color = {0, 0, 127}));
        connect(greaterThreshold2.y, booleanToInteger1.u) annotation(
          Line(points = {{47, 30}, {64, 30}}, color = {255, 0, 255}));
        connect(Start_Stop_Testmotor, booleanToInteger1.y) annotation(
          Line(points = {{110, 30}, {87, 30}}, color = {255, 127, 0}));
        connect(greaterThreshold2.u, Test_motor_torque) annotation(
          Line(points = {{24, 30}, {-40, 30}, {-40, 50}, {110, 50}}, color = {0, 0, 127}));
        connect(TestMotortorque.y, Test_motor_torque) annotation(
          Line(points = {{-59, 50}, {110, 50}}, color = {0, 0, 127}));
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false)),
          Diagram(coordinateSystem(preserveAspectRatio = false)),
          experiment(StopTime = 300, StartTime = 0, Tolerance = 1e-06, Interval = 0.6));
      end VFDAndServoControl2;

      model VFDAndServoControl4
        parameter Real ServoMotorvoltage = 2.5 "Voltage of the Servomotor";
        parameter Real ServoStartTorque = 0.02 "Start torque for the servo motor";
        parameter Real TestMotorTorqueAmplitude = 5 "Torque signal sendt to the test motor";
        parameter Real TestMotorTorqueStartTime = 10 "Ramp up start time";
        parameter Real TestMotorTorqueDuration = 10 "Duration of the ramp up";
        parameter Real TestMotorTorqueOffset = 0 "Initial torque value sendt to the test motot";
        Modelica.Blocks.Sources.Constant ConstantVoltage(k = ServoMotorvoltage) annotation(
          Placement(transformation(extent = {{-80, -98}, {-60, -78}})));
        Modelica.Blocks.Interfaces.IntegerOutput Start_Stop_Testmotor "Connector of Integer output signal" annotation(
          Placement(transformation(extent = {{100, 20}, {120, 40}})));
        Modelica.Blocks.Interfaces.RealOutput Servo_motor_speed "Connector of Real output signal" annotation(
          Placement(transformation(extent = {{100, -98}, {120, -78}})));
        Modelica.Blocks.Interfaces.RealOutput Test_motor_torque "Connector of Real output signal" annotation(
          Placement(transformation(extent = {{100, 40}, {120, 60}})));
        Modelica.Blocks.Sources.Constant ServoTorque(k = ServoStartTorque) annotation(
          Placement(transformation(extent = {{-46, -80}, {-26, -60}})));
        Modelica.Blocks.Interfaces.RealOutput Servo_Start_Torque "Connector of Real output signal" annotation(
          Placement(transformation(extent = {{100, -80}, {120, -60}})));
        Modelica.Blocks.Interfaces.IntegerOutput Start_Stop_Servomotor "Connector of Integer output signal" annotation(
          Placement(transformation(extent = {{100, -60}, {120, -40}})));
        Modelica.Blocks.Logical.GreaterThreshold greaterThreshold annotation(
          Placement(transformation(extent = {{-14, -64}, {6, -44}})));
        Modelica.Blocks.Math.BooleanToInteger booleanToInteger annotation(
          Placement(transformation(extent = {{60, -60}, {80, -40}})));
        Modelica.Blocks.Logical.And and1 annotation(
          Placement(transformation(extent = {{28, -60}, {48, -40}})));
        Modelica.Blocks.Logical.GreaterThreshold greaterThreshold1 annotation(
          Placement(transformation(extent = {{-14, -34}, {6, -14}})));
        Modelica.Blocks.Logical.GreaterThreshold greaterThreshold2 annotation(
          Placement(transformation(extent = {{26, 20}, {46, 40}})));
        Modelica.Blocks.Math.BooleanToInteger booleanToInteger1 annotation(
          Placement(transformation(extent = {{66, 20}, {86, 40}})));
        Modelica.Blocks.Sources.Sinc TestMotorTorqueSignal(amplitude = 5, f = 2, offset = 2, startTime = 20) "Do not change this parameters" annotation(
          Placement(transformation(extent = {{-60, 40}, {-40, 60}})));
      equation
        connect(ConstantVoltage.y, Servo_motor_speed) annotation(
          Line(points = {{-59, -88}, {110, -88}}, color = {0, 0, 127}));
        connect(ServoTorque.y, Servo_Start_Torque) annotation(
          Line(points = {{-25, -70}, {110, -70}}, color = {0, 0, 127}));
        connect(booleanToInteger.u, and1.y) annotation(
          Line(points = {{58, -50}, {49, -50}}, color = {255, 0, 255}));
        connect(booleanToInteger.y, Start_Stop_Servomotor) annotation(
          Line(points = {{81, -50}, {110, -50}}, color = {255, 127, 0}));
        connect(and1.u1, greaterThreshold1.y) annotation(
          Line(points = {{26, -50}, {14, -50}, {14, -24}, {7, -24}}, color = {255, 0, 255}));
        connect(and1.u2, greaterThreshold.y) annotation(
          Line(points = {{26, -58}, {14, -58}, {14, -54}, {7, -54}}, color = {255, 0, 255}));
        connect(ServoTorque.y, greaterThreshold.u) annotation(
          Line(points = {{-25, -70}, {-24, -70}, {-24, -54}, {-16, -54}}, color = {0, 0, 127}));
        connect(ConstantVoltage.y, greaterThreshold1.u) annotation(
          Line(points = {{-59, -88}, {-59, -24}, {-16, -24}}, color = {0, 0, 127}));
        connect(greaterThreshold2.y, booleanToInteger1.u) annotation(
          Line(points = {{47, 30}, {64, 30}}, color = {255, 0, 255}));
        connect(Start_Stop_Testmotor, booleanToInteger1.y) annotation(
          Line(points = {{110, 30}, {87, 30}}, color = {255, 127, 0}));
        connect(greaterThreshold2.u, Test_motor_torque) annotation(
          Line(points = {{24, 30}, {8, 30}, {8, 50}, {110, 50}}, color = {0, 0, 127}));
        connect(TestMotorTorqueSignal.y, Test_motor_torque) annotation(
          Line(points = {{-39, 50}, {110, 50}}, color = {0, 0, 127}));
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false)),
          Diagram(coordinateSystem(preserveAspectRatio = false)),
          experiment(StopTime = 300));
      end VFDAndServoControl4;

      model ConfigTestVDF22
        parameter Real SinHeight = 2 "Height of the sin wave";
        parameter Real SinOffset = 5 "Offset of the sin wave";
        parameter Real SinFrequency = 0.1 "Frequency of the sin wave";
        parameter Integer TestMotorTorqueStartTime = 1 "Ramp up start time";
        parameter Integer TestMotorTorqueHeight = 1 "Duration of the ramp up";
        parameter Integer TestMotorTorqueOffset = 0 "Initial torque value sendt to the test motot";
        Modelica.Blocks.Interfaces.RealOutput Test_motor_torque annotation(
          Placement(visible = true, transformation(origin = {0, 0}, extent = {{100, 40}, {120, 60}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{100, 40}, {120, 60}}, rotation = 0)));
        Modelica.Blocks.Interfaces.IntegerOutput Start_Stop_Testmotor annotation(
          Placement(visible = true, transformation(origin = {0, 0}, extent = {{100, 20}, {120, 40}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{100, 20}, {120, 40}}, rotation = 0)));
        Modelica.Blocks.Sources.IntegerStep integerStep(height = TestMotorTorqueHeight, offset = TestMotorTorqueOffset, startTime = TestMotorTorqueStartTime) annotation(
          Placement(visible = true, transformation(origin = {-42, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Sine sine(amplitude = SinHeight, f = SinFrequency, offset = SinOffset) annotation(
          Placement(visible = true, transformation(origin = {-46, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput InputVoltage annotation(
          Placement(visible = true, transformation(origin = {-120, -26}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-102, -30}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant const(k = 5) annotation(
          Placement(visible = true, transformation(origin = {-90, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Product P1 annotation(
          Placement(visible = true, transformation(origin = {-28, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Product P2 annotation(
          Placement(visible = true, transformation(origin = {40, -48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant constant1(k = 455) annotation(
          Placement(visible = true, transformation(origin = {-34, -74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(integerStep.y, Start_Stop_Testmotor) annotation(
          Line(points = {{-31, 12}, {86, 12}, {86, 30}, {110, 30}}, color = {255, 127, 0}));
        connect(sine.y, Test_motor_torque) annotation(
          Line(points = {{-35, 54}, {110, 54}, {110, 50}}, color = {0, 0, 127}));
        connect(const.y, P1.u2) annotation(
          Line(points = {{-79, -60}, {-59, -60}, {-59, -38}, {-40, -38}}, color = {0, 0, 127}));
        connect(P1.u1, InputVoltage) annotation(
          Line(points = {{-40, -26}, {-120, -26}}, color = {0, 0, 127}));
        connect(P2.u1, InputVoltage) annotation(
          Line(points = {{28, -42}, {-120, -42}, {-120, -26}}, color = {0, 0, 127}));
        connect(constant1.y, P2.u2) annotation(
          Line(points = {{-23, -74}, {28, -74}, {28, -54}}, color = {0, 0, 127}));
        annotation(
          __OpenModelica_simulationFlags(lv = "stdout,assert,LOG_STATS", s = "dassl", variableFilter = ".*"),
          experiment(StartTime = 0, StopTime = 300, Tolerance = 1e-06, Interval = 0.6));
      end ConfigTestVDF22;

      model Test1
        parameter Real TestMotorTorqueAmplitude = 9 "Torque signal sendt to the test motor";
        parameter Real TestMotorTorqueStartTime = 5 "Ramp up start time";
        parameter Real TestMotorTorqueDuration = 5 "Duration of the ramp up";
        parameter Real TestMotorTorqueOffset = 0 "Initial torque value sendt to the test motot";
        Modelica.Blocks.Interfaces.IntegerOutput Start_Stop_Testmotor "Connector of Integer output signal" annotation(
          Placement(transformation(extent = {{100, 20}, {120, 40}})));
        Modelica.Blocks.Interfaces.RealOutput Test_motor_torque "Connector of Real output signal" annotation(
          Placement(transformation(extent = {{100, 40}, {120, 60}})));
        Modelica.Blocks.Math.BooleanToInteger booleanToInteger1 annotation(
          Placement(transformation(extent = {{66, 20}, {86, 40}})));
        Modelica.Blocks.Sources.Ramp TestMotortorque(height = TestMotorTorqueAmplitude, duration = TestMotorTorqueDuration, offset = TestMotorTorqueOffset, startTime = TestMotorTorqueStartTime) annotation(
          Placement(transformation(extent = {{-80, 40}, {-60, 60}})));
        Modelica.Blocks.Logical.GreaterThreshold greaterThreshold annotation(
          Placement(visible = true, transformation(origin = {10, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(Start_Stop_Testmotor, booleanToInteger1.y) annotation(
          Line(points = {{110, 30}, {87, 30}}, color = {255, 127, 0}));
        connect(TestMotortorque.y, Test_motor_torque) annotation(
          Line(points = {{-59, 50}, {110, 50}}, color = {0, 0, 127}));
        connect(booleanToInteger1.u, greaterThreshold.y) annotation(
          Line(points = {{64, 30}, {21, 30}, {21, 32}}, color = {255, 0, 255}));
        connect(greaterThreshold.u, TestMotortorque.y) annotation(
          Line(points = {{-2, 32}, {-59, 32}, {-59, 50}}, color = {0, 0, 127}));
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false)),
          Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})),
          experiment(StopTime = 300, StartTime = 0, Tolerance = 1e-06, Interval = 0.6));
      end Test1;

      model ConfigTest
  Modelica.Blocks.Math.Add add annotation(
          Placement(visible = true, transformation(origin = {-44, 24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput y annotation(
          Placement(visible = true, transformation(origin = {110, 24}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {150, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput u annotation(
          Placement(visible = true, transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-122, 42}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput u1 annotation(
          Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-106, 16}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      equation
        connect(add.y, y) annotation(
          Line(points = {{-32, 24}, {110, 24}}, color = {0, 0, 127}));
        connect(add.u1, u) annotation(
          Line(points = {{-56, 30}, {-88, 30}, {-88, 60}, {-120, 60}}, color = {0, 0, 127}));
        connect(add.u2, u1) annotation(
          Line(points = {{-56, 18}, {-88, 18}, {-88, 0}, {-120, 0}}, color = {0, 0, 127}));
      end ConfigTest;
      
      model TotalControl
        parameter Real TestMotorTorqueAmplitude = 5 "Torque signal sendt to the test motor";
        parameter Real TestMotorTorqueStartTime = 11 "Ramp up start time";
        parameter Real TestMotorTorqueDuration = 5 "Duration of the ramp up";
        parameter Real TestMotorTorqueOffset = 0 "Initial torque value of the test motot";
        
        parameter Real ServoMotorVoltageAmplitude = 2.5 "Voltage signal sendt to the Servomotor";
        parameter Real ServoMotorVoltageStartTime = 1 "Ramp up start time";
        parameter Real ServoMotorVoltageDuration = 0 "Duration of the ram up";
        parameter Real ServoMotorVoltageOffset = 0 "Initial voltage value of the servo motor";
        
        parameter Real ServoMotorTorqueAmplitude = 0.05 "Start torque for the servo motor";
        parameter Real ServoMotorTorqueStartTime = 1 "Ramp up start time";
        parameter Real ServoMotorTorqueDuration = 0 "Duration of the ramp up";
        parameter Real ServoMotorTorqueOffset = 0 "Initial torque value of the servo motor";
        
        
        Modelica.Blocks.Interfaces.IntegerOutput Start_Stop_Testmotor "Connector of Integer output signal" annotation(
          Placement(visible = true, transformation(origin = {0, 36}, extent = {{100, 20}, {120, 40}}, rotation = 0), iconTransformation(origin = {0, 40}, extent = {{100, 20}, {120, 40}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput Test_motor_torque "Connector of Real output signal" annotation(
          Placement(visible = true, transformation(origin = {0, 36}, extent = {{100, 40}, {120, 60}}, rotation = 0), iconTransformation(origin = {0, 40}, extent = {{100, 40}, {120, 60}}, rotation = 0)));
        Modelica.Blocks.Math.BooleanToInteger booleanToInteger1 annotation(
          Placement(visible = true, transformation(origin = {0, 36}, extent = {{66, 20}, {86, 40}}, rotation = 0)));
        Modelica.Blocks.Sources.Ramp TestMotortorque(height = TestMotorTorqueAmplitude, duration = TestMotorTorqueDuration, offset = TestMotorTorqueOffset, startTime = TestMotorTorqueStartTime) annotation(
          Placement(visible = true, transformation(origin = {0, 36}, extent = {{-80, 40}, {-60, 60}}, rotation = 0)));
        Modelica.Blocks.Logical.GreaterThreshold greaterThreshold annotation(
          Placement(visible = true, transformation(origin = {10, 68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Logical.And and1 annotation(
          Placement(visible = true, transformation(origin = {0, 2}, extent = {{28, -60}, {48, -40}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput Servo_motor_speed annotation(
          Placement(visible = true, transformation(origin = {0, 2}, extent = {{100, -98}, {120, -78}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{100, -98}, {120, -78}}, rotation = 0)));
  Modelica.Blocks.Math.BooleanToInteger booleanToInteger annotation(
          Placement(visible = true, transformation(origin = {0, 2}, extent = {{60, -60}, {80, -40}}, rotation = 0)));
  Modelica.Blocks.Logical.GreaterThreshold greaterThreshold1 annotation(
          Placement(visible = true, transformation(origin = {0, 2}, extent = {{-14, -34}, {6, -14}}, rotation = 0)));
  Modelica.Blocks.Logical.GreaterThreshold greaterThreshold2 annotation(
          Placement(visible = true, transformation(origin = {0, 2}, extent = {{-14, -64}, {6, -44}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput Servo_Start_Torque annotation(
          Placement(visible = true, transformation(origin = {0, 2}, extent = {{100, -80}, {120, -60}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{100, -80}, {120, -60}}, rotation = 0)));
  Modelica.Blocks.Interfaces.IntegerOutput Start_Stop_Servomotor annotation(
          Placement(visible = true, transformation(origin = {0, 2}, extent = {{100, -60}, {120, -40}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{100, -60}, {120, -40}}, rotation = 0)));
  Modelica.Blocks.Interfaces.IntegerOutput Speed_Torque annotation(
          Placement(visible = true, transformation(origin = {110, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Ramp ServoMotorTorque(duration = ServoMotorTorqueDuration, height = ServoMotorTorqueAmplitude, offset = ServoMotorTorqueOffset, startTime = ServoMotorTorqueStartTime) annotation(
          Placement(visible = true, transformation(origin = {32, -118}, extent = {{-80, 40}, {-60, 60}}, rotation = 0)));
  Modelica.Blocks.Sources.Ramp ServoMotorVoltage(duration = ServoMotorVoltageDuration, height = ServoMotorVoltageAmplitude, offset = ServoMotorVoltageOffset, startTime = ServoMotorVoltageStartTime) annotation(
          Placement(visible = true, transformation(origin = {0, -134}, extent = {{-80, 40}, {-60, 60}}, rotation = 0)));
      equation
        connect(Start_Stop_Testmotor, booleanToInteger1.y) annotation(
          Line(points = {{110, 66}, {87, 66}}, color = {255, 127, 0}));
        connect(TestMotortorque.y, Test_motor_torque) annotation(
          Line(points = {{-59, 86}, {110, 86}}, color = {0, 0, 127}));
        connect(booleanToInteger1.u, greaterThreshold.y) annotation(
          Line(points = {{64, 66}, {21, 66}, {21, 68}}, color = {255, 0, 255}));
        connect(greaterThreshold.u, TestMotortorque.y) annotation(
          Line(points = {{-2, 68}, {-59, 68}, {-59, 86}}, color = {0, 0, 127}));
        connect(and1.u1, greaterThreshold1.y) annotation(
          Line(points = {{26, -48}, {14, -48}, {14, -22}, {7, -22}}, color = {255, 0, 255}));
        connect(booleanToInteger.u, and1.y) annotation(
          Line(points = {{58, -48}, {49, -48}}, color = {255, 0, 255}));
        connect(and1.u2, greaterThreshold2.y) annotation(
          Line(points = {{26, -56}, {14, -56}, {14, -52}, {7, -52}}, color = {255, 0, 255}));
        connect(booleanToInteger.y, Start_Stop_Servomotor) annotation(
          Line(points = {{81, -48}, {110, -48}}, color = {255, 127, 0}));
        connect(booleanToInteger1.y, Speed_Torque) annotation(
          Line(points = {{87, 66}, {87, 42}, {110, 42}}, color = {255, 127, 0}));
        connect(ServoMotorTorque.y, Servo_Start_Torque) annotation(
          Line(points = {{-27, -68}, {110, -68}}, color = {0, 0, 127}));
        connect(greaterThreshold2.u, ServoMotorTorque.y) annotation(
          Line(points = {{-16, -52}, {-26, -52}, {-26, -68}}, color = {0, 0, 127}));
  connect(ServoMotorVoltage.y, Servo_motor_speed) annotation(
          Line(points = {{-58, -84}, {110, -84}, {110, -86}}, color = {0, 0, 127}));
  connect(ServoMotorVoltage.y, greaterThreshold1.u) annotation(
          Line(points = {{-58, -84}, {-58, -50}, {-30, -50}, {-30, -22}, {-16, -22}}, color = {0, 0, 127}));
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false)),
          Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})),
          experiment(StopTime = 300, StartTime = 0, Tolerance = 1e-06, Interval = 0.6));
      end TotalControl;
      
      model TotalControl2
        parameter Real TestMotorTorqueAmplitude = 5 "Torque signal sendt to the test motor";
        parameter Real TestMotorTorqueStartTime = 11 "Ramp up start time";
        parameter Real TestMotorTorqueDuration = 5 "Duration of the ramp up";
        parameter Real TestMotorTorqueOffset = 0 "Initial torque value of the test motot";
        
        parameter Real ServoMotorVoltageAmplitude = 2.5 "Voltage signal sendt to the Servomotor";
        parameter Real ServoMotorVoltageStartTime = 1 "Ramp up start time";
        parameter Real ServoMotorVoltageDuration = 0 "Duration of the ram up";
        parameter Real ServoMotorVoltageOffset = 0 "Initial voltage value of the servo motor";
        
        parameter Real ServoMotorTorqueAmplitude = 0.05 "Start torque for the servo motor";
        parameter Real ServoMotorTorqueStartTime = 1 "Ramp up start time";
        parameter Real ServoMotorTorqueDuration = 0 "Duration of the ramp up";
        parameter Real ServoMotorTorqueOffset = 0 "Initial torque value of the servo motor";
        
        
        Modelica.Blocks.Interfaces.IntegerOutput Start_Stop_Testmotor "Connector of Integer output signal" annotation(
          Placement(visible = true, transformation(origin = {0, 36}, extent = {{100, 20}, {120, 40}}, rotation = 0), iconTransformation(origin = {0, 40}, extent = {{100, 20}, {120, 40}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput Test_motor_torque "Connector of Real output signal" annotation(
          Placement(visible = true, transformation(origin = {0, 36}, extent = {{100, 40}, {120, 60}}, rotation = 0), iconTransformation(origin = {0, 40}, extent = {{100, 40}, {120, 60}}, rotation = 0)));
        Modelica.Blocks.Math.BooleanToInteger booleanToInteger1 annotation(
          Placement(visible = true, transformation(origin = {0, 36}, extent = {{66, 20}, {86, 40}}, rotation = 0)));
        Modelica.Blocks.Sources.Ramp TestMotortorque(height = TestMotorTorqueAmplitude, duration = TestMotorTorqueDuration, offset = TestMotorTorqueOffset, startTime = TestMotorTorqueStartTime) annotation(
          Placement(visible = true, transformation(origin = {84, 36}, extent = {{-80, 40}, {-60, 60}}, rotation = 0)));
        Modelica.Blocks.Logical.GreaterThreshold greaterThreshold annotation(
          Placement(visible = true, transformation(origin = {46, 66}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Logical.And and1 annotation(
          Placement(visible = true, transformation(origin = {10, 2}, extent = {{28, -60}, {48, -40}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput Servo_motor_speed annotation(
          Placement(visible = true, transformation(origin = {0, 2}, extent = {{100, -98}, {120, -78}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{100, -98}, {120, -78}}, rotation = 0)));
      Modelica.Blocks.Math.BooleanToInteger booleanToInteger annotation(
          Placement(visible = true, transformation(origin = {10, 2}, extent = {{60, -60}, {80, -40}}, rotation = 0)));
      Modelica.Blocks.Logical.GreaterThreshold greaterThreshold1 annotation(
          Placement(visible = true, transformation(origin = {28, 8}, extent = {{-14, -34}, {6, -14}}, rotation = 0)));
      Modelica.Blocks.Logical.GreaterThreshold greaterThreshold2 annotation(
          Placement(visible = true, transformation(origin = {20, 4}, extent = {{-14, -64}, {6, -44}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput Servo_Start_Torque annotation(
          Placement(visible = true, transformation(origin = {0, 2}, extent = {{100, -80}, {120, -60}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{100, -80}, {120, -60}}, rotation = 0)));
      Modelica.Blocks.Interfaces.IntegerOutput Start_Stop_Servomotor annotation(
          Placement(visible = true, transformation(origin = {0, 2}, extent = {{100, -60}, {120, -40}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{100, -60}, {120, -40}}, rotation = 0)));
      Modelica.Blocks.Interfaces.IntegerOutput Speed_Torque annotation(
          Placement(visible = true, transformation(origin = {110, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Ramp ServoMotorTorque(duration = ServoMotorTorqueDuration, height = ServoMotorTorqueAmplitude, offset = ServoMotorTorqueOffset, startTime = ServoMotorTorqueStartTime) annotation(
          Placement(visible = true, transformation(origin = {60, -120}, extent = {{-80, 40}, {-60, 60}}, rotation = 0)));
      Modelica.Blocks.Sources.Ramp ServoMotorVoltage(duration = ServoMotorVoltageDuration, height = ServoMotorVoltageAmplitude, offset = ServoMotorVoltageOffset, startTime = ServoMotorVoltageStartTime) annotation(
          Placement(visible = true, transformation(origin = {32, -134}, extent = {{-80, 40}, {-60, 60}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput TM_M_Speed annotation(
          Placement(visible = true, transformation(origin = {-120, 88}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant1(k = 455) annotation(
          Placement(visible = true, transformation(origin = {-58, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Product P1 annotation(
          Placement(visible = true, transformation(origin = {-56, 82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Product P2 annotation(
          Placement(visible = true, transformation(origin = {-14, 68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 5) annotation(
          Placement(visible = true, transformation(origin = {-90, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain Ao1(k = 1)  annotation(
          Placement(visible = true, transformation(origin = {-70, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain Ao2(k = 1)  annotation(
          Placement(visible = true, transformation(origin = {-70, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput AO1 annotation(
          Placement(visible = true, transformation(origin = {-120, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput AO2 annotation(
          Placement(visible = true, transformation(origin = {-120, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      equation
        connect(Start_Stop_Testmotor, booleanToInteger1.y) annotation(
          Line(points = {{110, 66}, {87, 66}}, color = {255, 127, 0}));
        connect(TestMotortorque.y, Test_motor_torque) annotation(
          Line(points = {{25, 86}, {110, 86}}, color = {0, 0, 127}));
        connect(booleanToInteger1.u, greaterThreshold.y) annotation(
          Line(points = {{64, 66}, {57, 66}}, color = {255, 0, 255}));
        connect(greaterThreshold.u, TestMotortorque.y) annotation(
          Line(points = {{34, 66}, {25, 66}, {25, 86}}, color = {0, 0, 127}));
        connect(and1.u1, greaterThreshold1.y) annotation(
          Line(points = {{36, -48}, {36, -16}, {35, -16}}, color = {255, 0, 255}));
        connect(booleanToInteger.u, and1.y) annotation(
          Line(points = {{68, -48}, {59, -48}}, color = {255, 0, 255}));
        connect(and1.u2, greaterThreshold2.y) annotation(
          Line(points = {{36, -56}, {31.5, -56}, {31.5, -50}, {27, -50}}, color = {255, 0, 255}));
        connect(booleanToInteger.y, Start_Stop_Servomotor) annotation(
          Line(points = {{91, -48}, {110, -48}}, color = {255, 127, 0}));
        connect(booleanToInteger1.y, Speed_Torque) annotation(
          Line(points = {{87, 66}, {87, 42}, {110, 42}}, color = {255, 127, 0}));
        connect(ServoMotorTorque.y, Servo_Start_Torque) annotation(
          Line(points = {{1, -70}, {47.5, -70}, {47.5, -68}, {110, -68}}, color = {0, 0, 127}));
        connect(greaterThreshold2.u, ServoMotorTorque.y) annotation(
          Line(points = {{4, -50}, {2, -50}, {2, -70}, {1, -70}}, color = {0, 0, 127}));
        connect(ServoMotorVoltage.y, Servo_motor_speed) annotation(
          Line(points = {{-27, -84}, {41.5, -84}, {41.5, -86}, {110, -86}}, color = {0, 0, 127}));
        connect(ServoMotorVoltage.y, greaterThreshold1.u) annotation(
          Line(points = {{-27, -84}, {-27, -50}, {-6, -50}, {-6, -16}, {12, -16}}, color = {0, 0, 127}));
        connect(P1.u1, TM_M_Speed) annotation(
          Line(points = {{-68, 88}, {-120, 88}}, color = {0, 0, 127}));
        connect(constant1.y, P2.u2) annotation(
          Line(points = {{-47, 54}, {-38.5, 54}, {-38.5, 62}, {-26, 62}}, color = {0, 0, 127}));
        connect(P2.u1, TM_M_Speed) annotation(
          Line(points = {{-26, 74}, {-75, 74}, {-75, 88}, {-120, 88}}, color = {0, 0, 127}));
        connect(const.y, P1.u2) annotation(
          Line(points = {{-79, 54}, {-75, 54}, {-75, 76}, {-68, 76}}, color = {0, 0, 127}));
        connect(Ao1.u, AO1) annotation(
          Line(points = {{-82, 20}, {-120, 20}}, color = {0, 0, 127}));
        connect(Ao2.u, AO2) annotation(
          Line(points = {{-82, -20}, {-120, -20}}, color = {0, 0, 127}));
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false)),
          Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})),
          experiment(StopTime = 300, StartTime = 0, Tolerance = 1e-06, Interval = 0.6));
      end TotalControl2;
      
      model TotalControl3
  TestPackage.ActiveWork.MotorTest.TotalControl totalControl annotation(
          Placement(visible = true, transformation(origin = {10, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput y annotation(
          Placement(visible = true, transformation(origin = {114, 86}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {114, 86}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.IntegerOutput y1 annotation(
          Placement(visible = true, transformation(origin = {114, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {114, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.IntegerOutput y2 annotation(
          Placement(visible = true, transformation(origin = {124, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {124, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.IntegerOutput y3 annotation(
          Placement(visible = true, transformation(origin = {134, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {134, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput y4 annotation(
          Placement(visible = true, transformation(origin = {142, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {142, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput y5 annotation(
          Placement(visible = true, transformation(origin = {126, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {126, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput u annotation(
          Placement(visible = true, transformation(origin = {-124, 78}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-124, 78}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput u1 annotation(
          Placement(visible = true, transformation(origin = {-124, 44}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-102, 42}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput u2 annotation(
          Placement(visible = true, transformation(origin = {-124, -10}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-108, -12}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  TestPackage.ActiveWork.MotorTest.OnlyIn onlyIn annotation(
          Placement(visible = true, transformation(origin = {-50, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        equation
        connect(totalControl.Test_motor_torque, y) annotation(
          Line(points = {{22, 60}, {67, 60}, {67, 86}, {114, 86}, {114, 86}}, color = {0, 0, 127}));
        connect(totalControl.Start_Stop_Testmotor, y1) annotation(
          Line(points = {{22, 58}, {82, 58}, {82, 70}, {114, 70}}, color = {255, 127, 0}));
        connect(totalControl.Speed_Torque, y2) annotation(
          Line(points = {{22, 56}, {124, 56}, {124, 50}}, color = {255, 127, 0}));
        connect(totalControl.Start_Stop_Servomotor, y3) annotation(
          Line(points = {{22, 46}, {94, 46}, {94, 10}, {134, 10}}, color = {255, 127, 0}));
        connect(totalControl.Servo_Start_Torque, y4) annotation(
          Line(points = {{22, 44}, {86, 44}, {86, -10}, {142, -10}}, color = {0, 0, 127}));
        connect(totalControl.Servo_motor_speed, y5) annotation(
          Line(points = {{22, 42}, {80, 42}, {80, -32}, {126, -32}}, color = {0, 0, 127}));
  connect(onlyIn.TM_M_Speed, u) annotation(
          Line(points = {{-62, 58}, {-68, 58}, {-68, 78}, {-124, 78}}, color = {0, 0, 127}));
  connect(onlyIn.AO1, u1) annotation(
          Line(points = {{-62, 50}, {-92, 50}, {-92, 44}, {-124, 44}}, color = {0, 0, 127}));
  connect(onlyIn.AO2, u2) annotation(
          Line(points = {{-62, 42}, {-62, -10}, {-124, -10}}, color = {0, 0, 127}));
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false)),
          Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})),
          experiment(StopTime = 300, StartTime = 0, Tolerance = 1e-06, Interval = 0.6));
      end TotalControl3;
      
      model OnlyIn
      Modelica.Blocks.Interfaces.RealInput TM_M_Speed annotation(
          Placement(visible = true, transformation(origin = {-120, 88}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant constant1(k = 455) annotation(
          Placement(visible = true, transformation(origin = {-58, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Product P1 annotation(
          Placement(visible = true, transformation(origin = {-56, 82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Product P2 annotation(
          Placement(visible = true, transformation(origin = {-14, 68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant const(k = 5) annotation(
          Placement(visible = true, transformation(origin = {-90, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Gain Ao1(k = 1)  annotation(
          Placement(visible = true, transformation(origin = {-66, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Gain Ao2(k = 1)  annotation(
          Placement(visible = true, transformation(origin = {-70, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput AO1 annotation(
          Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-118, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput AO2 annotation(
          Placement(visible = true, transformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Math.Product Power annotation(
          Placement(visible = true, transformation(origin = {4, -38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(P1.u1, TM_M_Speed) annotation(
          Line(points = {{-68, 88}, {-120, 88}}, color = {0, 0, 127}));
        connect(constant1.y, P2.u2) annotation(
          Line(points = {{-47, 54}, {-38.5, 54}, {-38.5, 62}, {-26, 62}}, color = {0, 0, 127}));
        connect(P2.u1, TM_M_Speed) annotation(
          Line(points = {{-26, 74}, {-75, 74}, {-75, 88}, {-120, 88}}, color = {0, 0, 127}));
        connect(const.y, P1.u2) annotation(
          Line(points = {{-79, 54}, {-75, 54}, {-75, 76}, {-68, 76}}, color = {0, 0, 127}));
        connect(Ao1.u, AO1) annotation(
          Line(points = {{-78, 0}, {-120, 0}}, color = {0, 0, 127}));
        connect(Ao2.u, AO2) annotation(
          Line(points = {{-82, -80}, {-120, -80}}, color = {0, 0, 127}));
  connect(Ao2.y, Power.u2) annotation(
          Line(points = {{-58, -80}, {-20, -80}, {-20, -44}, {-8, -44}}, color = {0, 0, 127}));
  connect(Ao1.y, Power.u1) annotation(
          Line(points = {{-54, 0}, {-32, 0}, {-32, -32}, {-8, -32}}, color = {0, 0, 127}));
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false)),
          Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})),
          experiment(StopTime = 300, StartTime = 0, Tolerance = 1e-06, Interval = 0.6));
      end OnlyIn;
    end MotorTest;

    package WaterModel
      model TestSimpleHPL "Model of a hydropower system with a simple turbine turbine"
        extends Modelica.Icons.Example;
        Modelica.Blocks.Sources.Ramp control(duration = 30, height = -0.04615, offset = 0.7493, startTime = 50) annotation(
          Placement(visible = true, transformation(origin = {-80, 62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        OpenHPL.Waterway.Pipe intake(H = 23, Vdot(fixed = true)) annotation(
          Placement(visible = true, transformation(origin = {4, 0}, extent = {{-70, 20}, {-50, 40}}, rotation = 0)));
        OpenHPL.Waterway.Pipe discharge(H = 0.5, L = 600) annotation(
          Placement(visible = true, transformation(origin = {-12, 0}, extent = {{50, -10}, {70, 10}}, rotation = 0)));
        OpenHPL.Waterway.Reservoir tail(h_0 = 5, useLevel = true) annotation(
          Placement(visible = true, transformation(origin = {74, 0}, extent = {{-10, 10}, {10, -10}}, rotation = 180)));
        replaceable OpenHPL.Waterway.Pipe penstock(D_i = 3, D_o = 3, H = 428.5, L = 600, vertical = true) annotation(
          Placement(visible = true, transformation(origin = {-4, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0))) constrainedby Interfaces.TwoContact annotation(
           Placement(transformation(origin = {0, 30}, extent = {{-10, -10}, {10, 10}})));
        inner OpenHPL.Data data annotation(
          Placement(transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}})));
        OpenHPL.ElectroMech.Turbines.Turbine turbine1(enable_w = true) annotation(
          Placement(visible = true, transformation(origin = {22, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        OpenHPL.Waterway.Reservoir reservoir(h_0 = 48, useInflow = false, useLevel = true) annotation(
          Placement(visible = true, transformation(origin = {-80, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant const(k = 48) annotation(
          Placement(visible = true, transformation(origin = {-80, -12}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        OpenHPL.Waterway.SurgeTank surgeTank annotation(
          Placement(visible = true, transformation(origin = {-30, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant constant1(k = 5) annotation(
          Placement(visible = true, transformation(origin = {80, -48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        OpenHPL.ElectroMech.Generators.SimpleGen simpleGen(enable_w_in = false) annotation(
          Placement(visible = true, transformation(origin = {28, 68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor annotation(
          Placement(visible = true, transformation(origin = {28, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealOutput Torque_Out annotation(
          Placement(visible = true, transformation(origin = {110, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {140, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(discharge.o, tail.o) annotation(
          Line(points = {{58, 0}, {64, 0}}, color = {28, 108, 200}));
        connect(turbine1.o, discharge.i) annotation(
          Line(points = {{32, 0}, {38, 0}}, color = {0, 128, 255}));
        connect(penstock.o, turbine1.i) annotation(
          Line(points = {{6, 30}, {10, 30}, {10, -1}, {12, -1}, {12, 0}}, color = {0, 128, 255}));
        connect(turbine1.u_t, control.y) annotation(
          Line(points = {{14, 12}, {14, 42}, {-14.5, 42}, {-14.5, 62}, {-69, 62}}, color = {0, 0, 127}));
        connect(reservoir.o, intake.i) annotation(
          Line(points = {{-70, 30}, {-66, 30}}, color = {0, 128, 255}));
        connect(const.y, reservoir.level) annotation(
          Line(points = {{-91, -12}, {-98, -12}, {-98, 36}, {-92, 36}}, color = {0, 0, 127}));
        connect(intake.o, surgeTank.i) annotation(
          Line(points = {{-46, 30}, {-40, 30}}, color = {0, 128, 255}));
        connect(surgeTank.o, penstock.i) annotation(
          Line(points = {{-20, 30}, {-14, 30}}, color = {0, 128, 255}));
        connect(constant1.y, tail.level) annotation(
          Line(points = {{91, -48}, {96, -48}, {96, 6}, {86, 6}}, color = {0, 0, 127}));
        connect(turbine1.flange, torqueSensor.flange_a) annotation(
          Line(points = {{22, 0}, {22, 28}, {28, 28}}));
        connect(torqueSensor.flange_b, simpleGen.flange) annotation(
          Line(points = {{28, 48}, {28, 68}}));
        connect(torqueSensor.tau, Torque_Out) annotation(
          Line(points = {{40, 30}, {75, 30}, {75, 84}, {110, 84}}, color = {0, 0, 127}));
  connect(turbine1.w, simpleGen.Pload) annotation(
          Line(points = {{34, 4}, {36, 4}, {36, 20}, {50, 20}, {50, 90}, {28, 90}, {28, 80}}, color = {0, 0, 127}));
        annotation(
          experiment(StopTime = 1000, StartTime = 0, Tolerance = 1e-06, Interval = 2),
          Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}})));
      end TestSimpleHPL;
      
      model TestSimpleHPL1 "Model of a hydropower system with a simple turbine turbine"
        extends Modelica.Icons.Example;
        Modelica.Blocks.Sources.Ramp control(duration = 30, height = -0.04615, offset = 0.7493, startTime = 50) annotation(
          Placement(visible = true, transformation(origin = {-80, 62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        OpenHPL.Waterway.Pipe intake(H = 23, Vdot(fixed = true)) annotation(
          Placement(visible = true, transformation(origin = {4, 0}, extent = {{-70, 20}, {-50, 40}}, rotation = 0)));
        OpenHPL.Waterway.Pipe discharge(H = 0.5, L = 600) annotation(
          Placement(visible = true, transformation(origin = {-12, 0}, extent = {{50, -10}, {70, 10}}, rotation = 0)));
        OpenHPL.Waterway.Reservoir tail(h_0 = 5, useLevel = true) annotation(
          Placement(visible = true, transformation(origin = {74, 0}, extent = {{-10, 10}, {10, -10}}, rotation = 180)));
        replaceable OpenHPL.Waterway.Pipe penstock(D_i = 3, D_o = 3, H = 428.5, L = 600, vertical = true) annotation(
          Placement(visible = true, transformation(origin = {-4, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0))) constrainedby Interfaces.TwoContact annotation(
           Placement(transformation(origin = {0, 30}, extent = {{-10, -10}, {10, 10}})));
        inner OpenHPL.Data data annotation(
          Placement(transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}})));
        OpenHPL.ElectroMech.Turbines.Turbine turbine1(enable_w = true) annotation(
          Placement(visible = true, transformation(origin = {22, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        OpenHPL.Waterway.Reservoir reservoir(h_0 = 48, useInflow = false, useLevel = true) annotation(
          Placement(visible = true, transformation(origin = {-80, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput P_Out annotation(
          Placement(visible = true, transformation(origin = {110, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {136, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant const(k = 48) annotation(
          Placement(visible = true, transformation(origin = {-80, -12}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        OpenHPL.Waterway.SurgeTank surgeTank annotation(
          Placement(visible = true, transformation(origin = {-30, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant constant1(k = 5) annotation(
          Placement(visible = true, transformation(origin = {80, -48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        OpenHPL.ElectroMech.Generators.SimpleGen simpleGen(enable_w_in = false) annotation(
          Placement(visible = true, transformation(origin = {28, 68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor annotation(
          Placement(visible = true, transformation(origin = {28, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Blocks.Interfaces.RealOutput Torque_Out annotation(
          Placement(visible = true, transformation(origin = {110, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {140, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Power_In annotation(
          Placement(visible = true, transformation(origin = {-120, 78}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-96, 70}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      equation
        connect(discharge.o, tail.o) annotation(
          Line(points = {{58, 0}, {64, 0}}, color = {28, 108, 200}));
        connect(turbine1.o, discharge.i) annotation(
          Line(points = {{32, 0}, {38, 0}}, color = {0, 128, 255}));
        connect(penstock.o, turbine1.i) annotation(
          Line(points = {{6, 30}, {10, 30}, {10, -1}, {12, -1}, {12, 0}}, color = {0, 128, 255}));
        connect(turbine1.u_t, control.y) annotation(
          Line(points = {{14, 12}, {14, 42}, {-14.5, 42}, {-14.5, 62}, {-69, 62}}, color = {0, 0, 127}));
        connect(reservoir.o, intake.i) annotation(
          Line(points = {{-70, 30}, {-66, 30}}, color = {0, 128, 255}));
        connect(turbine1.w, P_Out) annotation(
          Line(points = {{33, 4}, {34, 4}, {34, 12}, {110, 12}}, color = {0, 0, 127}));
        connect(const.y, reservoir.level) annotation(
          Line(points = {{-91, -12}, {-98, -12}, {-98, 36}, {-92, 36}}, color = {0, 0, 127}));
        connect(intake.o, surgeTank.i) annotation(
          Line(points = {{-46, 30}, {-40, 30}}, color = {0, 128, 255}));
        connect(surgeTank.o, penstock.i) annotation(
          Line(points = {{-20, 30}, {-14, 30}}, color = {0, 128, 255}));
        connect(constant1.y, tail.level) annotation(
          Line(points = {{91, -48}, {96, -48}, {96, 6}, {86, 6}}, color = {0, 0, 127}));
        connect(turbine1.flange, torqueSensor.flange_a) annotation(
          Line(points = {{22, 0}, {22, 28}, {28, 28}}));
        connect(torqueSensor.flange_b, simpleGen.flange) annotation(
          Line(points = {{28, 48}, {28, 68}}));
        connect(torqueSensor.tau, Torque_Out) annotation(
          Line(points = {{40, 30}, {75, 30}, {75, 84}, {110, 84}}, color = {0, 0, 127}));
        connect(simpleGen.Pload, Power_In) annotation(
          Line(points = {{28, 80}, {28, 86}, {-58, 86}, {-58, 78}, {-120, 78}}, color = {0, 0, 127}));
        annotation(
          experiment(StopTime = 1000, StartTime = 0, Tolerance = 1e-06, Interval = 2),
          Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}})));
      end TestSimpleHPL1;

      model converter
        Modelica.Blocks.Logical.LessEqualThreshold lessEqualThreshold(threshold = 10) annotation(
          Placement(visible = true, transformation(origin = {-66, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput u annotation(
          Placement(visible = true, transformation(origin = {-120, 70}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-96, 64}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        Modelica.Blocks.Logical.And and1 annotation(
          Placement(visible = true, transformation(origin = {-24, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Logical.GreaterEqual greaterEqual annotation(
          Placement(visible = true, transformation(origin = {-52, 44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant const(k = -10) annotation(
          Placement(visible = true, transformation(origin = {-88, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.BooleanToReal booleanToReal annotation(
          Placement(visible = true, transformation(origin = {16, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.BooleanToInteger booleanToInteger annotation(
          Placement(visible = true, transformation(origin = {14, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.IntegerOutput y annotation(
          Placement(visible = true, transformation(origin = {110, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {122, 24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Product product annotation(
          Placement(visible = true, transformation(origin = {56, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput y1 annotation(
          Placement(visible = true, transformation(origin = {110, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {122, 74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(lessEqualThreshold.u, u) annotation(
          Line(points = {{-78, 70}, {-120, 70}}, color = {0, 0, 127}));
        connect(lessEqualThreshold.y, and1.u1) annotation(
          Line(points = {{-55, 70}, {-36, 70}}, color = {255, 0, 255}));
        connect(greaterEqual.u1, u) annotation(
          Line(points = {{-64, 44}, {-82, 44}, {-82, 70}, {-120, 70}}, color = {0, 0, 127}));
        connect(const.y, greaterEqual.u2) annotation(
          Line(points = {{-77, 30}, {-75.5, 30}, {-75.5, 36}, {-64, 36}, {-64, 36}}, color = {0, 0, 127}));
        connect(greaterEqual.y, and1.u2) annotation(
          Line(points = {{-41, 44}, {-36, 44}, {-36, 62}}, color = {255, 0, 255}));
        connect(and1.y, booleanToReal.u) annotation(
          Line(points = {{-12, 70}, {4, 70}}, color = {255, 0, 255}));
        connect(booleanToInteger.u, and1.y) annotation(
          Line(points = {{2, 30}, {-4, 30}, {-4, 70}, {-12, 70}}, color = {255, 0, 255}));
        connect(booleanToInteger.y, y) annotation(
          Line(points = {{26, 30}, {110, 30}}, color = {255, 127, 0}));
        connect(booleanToReal.y, product.u2) annotation(
          Line(points = {{28, 70}, {44, 70}}, color = {0, 0, 127}));
        connect(product.u1, u) annotation(
          Line(points = {{44, 82}, {32, 82}, {32, 94}, {-86, 94}, {-86, 70}, {-120, 70}}, color = {0, 0, 127}));
        connect(product.y, y1) annotation(
          Line(points = {{68, 76}, {110, 76}}, color = {0, 0, 127}));
      end converter;

      model test
  TestPackage.ActiveWork.WaterModel.TestSimpleHPL testSimpleHPL annotation(
          Placement(visible = true, transformation(origin = {-90, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 10000) annotation(
          Placement(visible = true, transformation(origin = {-88, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Division division annotation(
          Placement(visible = true, transformation(origin = {-46, 78}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant1(k = -10) annotation(
          Placement(visible = true, transformation(origin = {-68, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Product product annotation(
          Placement(visible = true, transformation(origin = {76, 36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Logical.LessEqualThreshold lessEqualThreshold(threshold = 10) annotation(
          Placement(visible = true, transformation(origin = {-46, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.BooleanToInteger booleanToInteger annotation(
          Placement(visible = true, transformation(origin = {34, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Logical.And and1 annotation(
          Placement(visible = true, transformation(origin = {-4, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.BooleanToReal booleanToReal annotation(
          Placement(visible = true, transformation(origin = {36, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Logical.GreaterEqual greaterEqual annotation(
          Placement(visible = true, transformation(origin = {-32, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput y annotation(
          Placement(visible = true, transformation(origin = {128, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {128, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.IntegerOutput y1 annotation(
          Placement(visible = true, transformation(origin = {128, -14}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {128, -14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(const.y, division.u2) annotation(
          Line(points = {{-77, 54}, {-61, 54}, {-61, 72}, {-58, 72}}, color = {0, 0, 127}));
        connect(testSimpleHPL.Torque_Out, division.u1) annotation(
          Line(points = {{-76, 84}, {-58, 84}}, color = {0, 0, 127}));
        connect(and1.y, booleanToReal.u) annotation(
          Line(points = {{7, 30}, {23, 30}}, color = {255, 0, 255}));
        connect(booleanToReal.y, product.u2) annotation(
          Line(points = {{47, 30}, {63, 30}}, color = {0, 0, 127}));
        connect(constant1.y, greaterEqual.u2) annotation(
          Line(points = {{-57, -10}, {-55.5, -10}, {-55.5, -4}, {-44, -4}, {-44, -4}}, color = {0, 0, 127}));
        connect(greaterEqual.y, and1.u2) annotation(
          Line(points = {{-21, 4}, {-16, 4}, {-16, 22}}, color = {255, 0, 255}));
        connect(lessEqualThreshold.y, and1.u1) annotation(
          Line(points = {{-35, 30}, {-16, 30}}, color = {255, 0, 255}));
        connect(booleanToInteger.u, and1.y) annotation(
          Line(points = {{22, -10}, {16, -10}, {16, 30}, {8, 30}}, color = {255, 0, 255}));
        connect(division.y, lessEqualThreshold.u) annotation(
          Line(points = {{-34, 78}, {-24, 78}, {-24, 46}, {-68, 46}, {-68, 30}, {-58, 30}}, color = {0, 0, 127}));
        connect(product.u1, division.y) annotation(
          Line(points = {{64, 42}, {52, 42}, {52, 78}, {-34, 78}}, color = {0, 0, 127}));
        connect(product.y, y) annotation(
          Line(points = {{88, 36}, {128, 36}, {128, 32}}, color = {0, 0, 127}));
        connect(booleanToInteger.y, y1) annotation(
          Line(points = {{46, -10}, {79, -10}, {79, -14}, {128, -14}}, color = {255, 127, 0}));
  connect(greaterEqual.u1, division.y) annotation(
          Line(points = {{-44, 4}, {-56, 4}, {-56, 8}, {-68, 8}, {-68, 46}, {-24, 46}, {-24, 78}, {-34, 78}}, color = {0, 0, 127}));
      annotation(
          experiment(StartTime = 0, StopTime = 1000, Tolerance = 1e-6, Interval = 0.2));
end test;
    end WaterModel;
  end ActiveWork;
  annotation(
    uses(Modelica(version = "4.0.0"), OpenHPL(version = "2.0.1")));
end TestPackage;
