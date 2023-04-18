package TestPackage
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

  model VFDAndServoControl3
    extends VFDAndServoControl2;
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
      Line(points = {{-32, 52}, {110, 52}, {110, 50}}, color = {0, 0, 127}));
    connect(integerStep.y, Start_Stop_Testmotor) annotation(
      Line(points = {{-30, 12}, {86, 12}, {86, 30}, {110, 30}}, color = {255, 127, 0}));
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
      Line(points = {{-30, 12}, {86, 12}, {86, 30}, {110, 30}}, color = {255, 127, 0}));
    connect(sine.y, Test_motor_torque) annotation(
      Line(points = {{-34, 54}, {110, 54}, {110, 50}}, color = {0, 0, 127}));
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
  Modelica.Blocks.Math.Gain gain(k = 1)  annotation(
      Placement(visible = true, transformation(origin = {-34, -26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(const.y, Test_motor_torque) annotation(
      Line(points = {{-32, 52}, {110, 52}, {110, 50}}, color = {0, 0, 127}));
    connect(integerStep.y, Start_Stop_Testmotor) annotation(
      Line(points = {{-30, 12}, {86, 12}, {86, 30}, {110, 30}}, color = {255, 127, 0}));
  connect(gain.u, InputVoltage) annotation(
      Line(points = {{-46, -26}, {-120, -26}}, color = {0, 0, 127}));
    annotation(
      __OpenModelica_simulationFlags(lv = "stdout,assert,LOG_STATS", s = "dassl", variableFilter = ".*"),
      experiment(StartTime = 0, StopTime = 300, Tolerance = 1e-6, Interval = 0.6));
  end ConfigTestVDF12;
  
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
  Modelica.Blocks.Sources.Constant const(k = 5)  annotation(
      Placement(visible = true, transformation(origin = {-90, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Product P1 annotation(
      Placement(visible = true, transformation(origin = {-28, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Product P2 annotation(
      Placement(visible = true, transformation(origin = {40, -48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant1(k = 455) annotation(
      Placement(visible = true, transformation(origin = {-34, -74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(integerStep.y, Start_Stop_Testmotor) annotation(
      Line(points = {{-30, 12}, {86, 12}, {86, 30}, {110, 30}}, color = {255, 127, 0}));
    connect(sine.y, Test_motor_torque) annotation(
      Line(points = {{-34, 54}, {110, 54}, {110, 50}}, color = {0, 0, 127}));
    connect(const.y, P1.u2) annotation(
      Line(points = {{-78, -60}, {-59, -60}, {-59, -38}, {-40, -38}}, color = {0, 0, 127}));
    connect(P1.u1, InputVoltage) annotation(
      Line(points = {{-40, -26}, {-120, -26}}, color = {0, 0, 127}));
  connect(P2.u1, InputVoltage) annotation(
      Line(points = {{28, -42}, {-120, -42}, {-120, -26}}, color = {0, 0, 127}));
  connect(constant1.y, P2.u2) annotation(
      Line(points = {{-22, -74}, {28, -74}, {28, -54}}, color = {0, 0, 127}));
    annotation(
      __OpenModelica_simulationFlags(lv = "stdout,assert,LOG_STATS", s = "dassl", variableFilter = ".*"),
      experiment(StartTime = 0, StopTime = 300, Tolerance = 1e-06, Interval = 0.6));
  end ConfigTestVDF22;
  
  model SimpleWaterWayTest "Model of a hydropower system with a simple turbine turbine"
    extends Modelica.Icons.Example;
    OpenHPL.Waterway.Reservoir reservoir(h_0 = 48) annotation(
      Placement(transformation(origin = {-90, 30}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Ramp control(duration = 30, height = -0.04615, offset = 0.7493, startTime = 500) annotation(
      Placement(transformation(origin = {-10, 70}, extent = {{-10, -10}, {10, 10}})));
    OpenHPL.Waterway.Pipe intake(H = 23, Vdot(fixed = true)) annotation(
      Placement(transformation(extent = {{-70, 20}, {-50, 40}})));
    OpenHPL.Waterway.Pipe discharge(H = 0.5, L = 600) annotation(
      Placement(transformation(extent = {{50, -10}, {70, 10}})));
    OpenHPL.Waterway.Reservoir tail(h_0 = 5) annotation(
      Placement(transformation(origin = {90, 0}, extent = {{-10, 10}, {10, -10}}, rotation = 180)));
    replaceable OpenHPL.Waterway.Pipe penstock(D_i = 3, D_o = 3, H = 428.5, L = 600, vertical = true) constrainedby Interfaces.TwoContact annotation(
       Placement(transformation(origin = {0, 30}, extent = {{-10, -10}, {10, 10}})));
    OpenHPL.Waterway.SurgeTank surgeTank(h_0 = 69.9) annotation(
      Placement(transformation(origin = {-30, 30}, extent = {{-10, -10}, {10, 10}})));
    inner OpenHPL.Data data annotation(
      Placement(transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}})));
  OpenHPL.ElectroMech.Turbines.Turbine turbine annotation(
      Placement(visible = true, transformation(origin = {32, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b annotation(
      Placement(visible = true, transformation(origin = {-104, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {120, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpenHPL.ElectroMech.PowerSystem.Grid grid annotation(
      Placement(visible = true, transformation(origin = {42, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(reservoir.o, intake.i) annotation(
      Line(points = {{-80, 30}, {-70, 30}}, color = {28, 108, 200}));
    connect(intake.o, surgeTank.i) annotation(
      Line(points = {{-50, 30}, {-40, 30}}, color = {28, 108, 200}));
    connect(surgeTank.o, penstock.i) annotation(
      Line(points = {{-20, 30}, {-10, 30}}, color = {28, 108, 200}));
    connect(discharge.o, tail.o) annotation(
      Line(points = {{70, 0}, {80, 0}}, color = {28, 108, 200}));
    connect(control.y, turbine.u_t) annotation(
      Line(points = {{2, 70}, {24, 70}, {24, 28}}, color = {0, 0, 127}));
    connect(discharge.i, turbine.o) annotation(
      Line(points = {{50, 0}, {48, 0}, {48, 16}, {42, 16}}, color = {0, 128, 255}));
    connect(turbine.i, penstock.o) annotation(
      Line(points = {{22, 16}, {14, 16}, {14, 30}, {10, 30}}, color = {0, 128, 255}));
    connect(turbine.flange, flange_b) annotation(
      Line(points = {{32, 16}, {32, 56}, {-104, 56}}));
    connect(turbine.P_out, grid.w_in) annotation(
      Line(points = {{36, 28}, {34, 28}, {34, 64}}, color = {0, 0, 127}));
    annotation(
      experiment(StopTime = 1000));
  end SimpleWaterWayTest;

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
  end Old_Models;
  
  model Test1
    parameter Real ServoMotorvoltage = 2.5 "Voltage of the Servomotor";
    parameter Real ServoMotorTorqueOffset = 0.05 "Start torque for the servo motor";
    parameter Real ServoMotorTorqueStartTime = 2 "Start time of ramping down the start torque";
    parameter Real ServoMotorTorqueDuration = 1 "Duration of the torque ramp down";
    parameter Real ServoMotorTorqueAmplitude = 1.95 "Amplitude of ramping down";
    parameter Real TestMotorTorqueAmplitude = 5 "Torque signal sendt to the test motor";
    parameter Real TestMotorTorqueStartTime = 20 "Ramp up start time";
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
    Modelica.Blocks.Interfaces.RealOutput Servo_Start_Torque "Connector of Real output signal" annotation(
      Placement(transformation(extent = {{100, -80}, {120, -60}})));
    Modelica.Blocks.Interfaces.IntegerOutput Start_Stop_Servomotor "Connector of Integer output signal" annotation(
      Placement(transformation(extent = {{100, -60}, {120, -40}})));
    Modelica.Blocks.Math.BooleanToInteger booleanToInteger annotation(
      Placement(transformation(extent = {{60, -60}, {80, -40}})));
    Modelica.Blocks.Logical.GreaterThreshold greaterThreshold1 annotation(
      Placement(visible = true, transformation(origin = {24, -26}, extent = {{-14, -34}, {6, -14}}, rotation = 0)));
    Modelica.Blocks.Logical.GreaterThreshold greaterThreshold2 annotation(
      Placement(transformation(extent = {{26, 20}, {46, 40}})));
    Modelica.Blocks.Math.BooleanToInteger booleanToInteger1 annotation(
      Placement(transformation(extent = {{66, 20}, {86, 40}})));
    Modelica.Blocks.Sources.Ramp TestMotortorque(height = TestMotorTorqueAmplitude, duration = TestMotorTorqueDuration, offset = TestMotorTorqueOffset, startTime = TestMotorTorqueStartTime) annotation(
      Placement(transformation(extent = {{-80, 40}, {-60, 60}})));
  Modelica.Blocks.Sources.Ramp rampDown(duration = ServoMotorTorqueDuration, height = ServoMotorTorqueAmplitude, offset = ServoMotorTorqueOffset, startTime = ServoMotorTorqueStartTime) annotation(
      Placement(visible = true, transformation(origin = {30, -120}, extent = {{-80, 40}, {-60, 60}}, rotation = 0)));
  equation
    connect(ConstantVoltage.y, Servo_motor_speed) annotation(
      Line(points = {{-59, -88}, {110, -88}}, color = {0, 0, 127}));
    connect(booleanToInteger.y, Start_Stop_Servomotor) annotation(
      Line(points = {{81, -50}, {110, -50}}, color = {255, 127, 0}));
    connect(ConstantVoltage.y, greaterThreshold1.u) annotation(
      Line(points = {{-59, -88}, {-59, -50}, {8, -50}}, color = {0, 0, 127}));
    connect(greaterThreshold2.y, booleanToInteger1.u) annotation(
      Line(points = {{47, 30}, {64, 30}}, color = {255, 0, 255}));
    connect(Start_Stop_Testmotor, booleanToInteger1.y) annotation(
      Line(points = {{110, 30}, {87, 30}}, color = {255, 127, 0}));
    connect(greaterThreshold2.u, Test_motor_torque) annotation(
      Line(points = {{24, 30}, {-40, 30}, {-40, 50}, {110, 50}}, color = {0, 0, 127}));
    connect(TestMotortorque.y, Test_motor_torque) annotation(
      Line(points = {{-59, 50}, {110, 50}}, color = {0, 0, 127}));
    connect(booleanToInteger.u, greaterThreshold1.y) annotation(
      Line(points = {{58, -50}, {31, -50}}, color = {255, 0, 255}));
  connect(rampDown.y, Servo_Start_Torque) annotation(
      Line(points = {{-28, -70}, {110, -70}}, color = {0, 0, 127}));
    annotation(
      Icon(coordinateSystem(preserveAspectRatio = false)),
      Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})),
      experiment(StopTime = 300, StartTime = 0, Tolerance = 1e-06, Interval = 0.6));
  end Test1;
  annotation(
    uses(Modelica(version = "4.0.0"), OpenHPL(version = "2.0.1")));
end TestPackage;
