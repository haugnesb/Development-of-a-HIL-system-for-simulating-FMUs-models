package TestPackage
  model SimpleTest
    parameter Real TestMotorTorqueAmplitude = -4 "Torque signal sendt to the test motor";
    parameter Real TestMotorTorqueStartTime = 20 "Ramp up start time";
    parameter Real TestMotorTorqueDuration = 3 "Duration of the ramp up";
    parameter Real TestMotorTorqueOffset = 8 "Initial torque value of the test motot";
    parameter Real ServoMotorVoltageAmplitude = 2.5 "Voltage signal sendt to the Servomotor";
    parameter Real ServoMotorVoltageStartTime = 1 "Ramp up start time";
    parameter Real ServoMotorVoltageDuration = 0 "Duration of the ram up";
    parameter Real ServoMotorVoltageOffset = 0 "Initial voltage value of the servo motor";
    parameter Real ServoMotorTorqueAmplitude = 0.05 "Start torque for the servo motor";
    parameter Real ServoMotorTorqueStartTime = 1 "Ramp up start time";
    parameter Real ServoMotorTorqueDuration = 0 "Duration of the ramp up";
    parameter Real ServoMotorTorqueOffset = 0 "Initial torque value of the servo motor";
    Modelica.Blocks.Interfaces.IntegerOutput TM_Start_Stop_Out "Connector of Integer output signal" annotation(
      Placement(visible = true, transformation(origin = {0, 30}, extent = {{100, 20}, {120, 40}}, rotation = 0), iconTransformation(origin = {0, 40}, extent = {{100, 20}, {120, 40}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput TM_Torque_Out "Connector of Real output signal" annotation(
      Placement(visible = true, transformation(origin = {0, 30}, extent = {{100, 40}, {120, 60}}, rotation = 0), iconTransformation(origin = {0, 40}, extent = {{100, 40}, {120, 60}}, rotation = 0)));
    Modelica.Blocks.Math.BooleanToInteger booleanToInteger1 annotation(
      Placement(visible = true, transformation(origin = {76, 60}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Sources.Ramp TestMotortorque(duration = TestMotorTorqueDuration, height = TestMotorTorqueAmplitude, offset = 0, startTime = TestMotorTorqueStartTime) annotation(
      Placement(visible = true, transformation(origin = {66, -2}, extent = {{-80, 40}, {-60, 60}}, rotation = 0)));
    Modelica.Blocks.Logical.And and1 annotation(
      Placement(visible = true, transformation(origin = {60, -8}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput SM_Speed_Out annotation(
      Placement(visible = true, transformation(origin = {0, 34}, extent = {{100, -98}, {120, -78}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{100, -98}, {120, -78}}, rotation = 0)));
    Modelica.Blocks.Math.BooleanToInteger booleanToInteger annotation(
      Placement(visible = true, transformation(origin = {78, -8}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Logical.GreaterThreshold greaterThreshold1 annotation(
      Placement(visible = true, transformation(origin = {40, -8}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Logical.GreaterThreshold greaterThreshold2 annotation(
      Placement(visible = true, transformation(origin = {40, -24}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput SM_Start_Torque_Out annotation(
      Placement(visible = true, transformation(origin = {0, 38}, extent = {{100, -80}, {120, -60}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{100, -80}, {120, -60}}, rotation = 0)));
    Modelica.Blocks.Interfaces.IntegerOutput SM_Start_Stop_Out annotation(
      Placement(visible = true, transformation(origin = {0, 42}, extent = {{100, -60}, {120, -40}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{100, -60}, {120, -40}}, rotation = 0)));
    Modelica.Blocks.Interfaces.IntegerOutput TM_Speed_Torque_Out annotation(
      Placement(visible = true, transformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Ramp ServoMotorTorque(duration = ServoMotorTorqueDuration, height = ServoMotorTorqueAmplitude, offset = ServoMotorTorqueOffset, startTime = ServoMotorTorqueStartTime) annotation(
      Placement(visible = true, transformation(origin = {16, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Ramp ServoMotorVoltage(duration = ServoMotorVoltageDuration, height = ServoMotorVoltageAmplitude, offset = ServoMotorVoltageOffset, startTime = ServoMotorVoltageStartTime) annotation(
      Placement(visible = true, transformation(origin = {48, -104}, extent = {{-80, 40}, {-60, 60}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput TM_M_Speed_In annotation(
      Placement(visible = true, transformation(origin = {-120, 88}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Math.Product TM_Voltage annotation(
      Placement(visible = true, transformation(origin = {1, 83}, extent = {{-9, -9}, {9, 9}}, rotation = 0)));
    Modelica.Blocks.Math.Product TM_Speed annotation(
      Placement(visible = true, transformation(origin = {-57, 63}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant Voltage_Conversion(k = 5) annotation(
      Placement(visible = true, transformation(origin = {-33, 71}, extent = {{-9, -9}, {9, 9}}, rotation = 0)));
    Modelica.Blocks.Math.Gain SM_Voltage_Conversion(k = 405) annotation(
      Placement(visible = true, transformation(origin = {-52, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Gain SM_Torque_Conversion(k = -17.21) annotation(
      Placement(visible = true, transformation(origin = {-34, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput SM_M_Speed_In annotation(
      Placement(visible = true, transformation(origin = {-120, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput SM_M_Torque_In annotation(
      Placement(visible = true, transformation(origin = {-120, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.IntegerOutput TM_Forward_Reverse_Out annotation(
      Placement(visible = true, transformation(origin = {110, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant Speed_Conversion(k = 455) annotation(
      Placement(visible = true, transformation(origin = {-87, 59}, extent = {{-9, -9}, {9, 9}}, rotation = 0)));
    Modelica.Blocks.Logical.GreaterThreshold greaterThreshold annotation(
      Placement(visible = true, transformation(origin = {60, 60}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Sources.Ramp ramp(duration = TestMotorTorqueDuration, height = TestMotorTorqueOffset, offset = 0, startTime = 5) annotation(
      Placement(visible = true, transformation(origin = {68, -34}, extent = {{-80, 40}, {-60, 60}}, rotation = 0)));
    Modelica.Blocks.Math.Add add annotation(
      Placement(visible = true, transformation(origin = {38, 60}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
    Modelica.Blocks.Continuous.Filter filter(f_cut = 1) annotation(
      Placement(visible = true, transformation(origin = {-64, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Continuous.Filter filter1(f_cut = 1) annotation(
      Placement(visible = true, transformation(origin = {-86, 88}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Continuous.Filter filter2(f_cut = 1) annotation(
      Placement(visible = true, transformation(origin = {-82, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(and1.u1, greaterThreshold1.y) annotation(
      Line(points = {{55.2, -8}, {44.2, -8}}, color = {255, 0, 255}));
    connect(booleanToInteger.u, and1.y) annotation(
      Line(points = {{73.2, -8}, {64.2, -8}}, color = {255, 0, 255}));
    connect(and1.u2, greaterThreshold2.y) annotation(
      Line(points = {{55.2, -11.2}, {48.2, -11.2}, {48.2, -24.2}, {44.2, -24.2}}, color = {255, 0, 255}));
    connect(booleanToInteger.y, SM_Start_Stop_Out) annotation(
      Line(points = {{82.4, -8}, {110.4, -8}}, color = {255, 127, 0}));
    connect(booleanToInteger1.y, TM_Speed_Torque_Out) annotation(
      Line(points = {{80.4, 60}, {86.25, 60}, {86.25, 40}, {110.1, 40}}, color = {255, 127, 0}));
    connect(ServoMotorTorque.y, SM_Start_Torque_Out) annotation(
      Line(points = {{27, -32}, {110, -32}}, color = {0, 0, 127}));
    connect(greaterThreshold2.u, ServoMotorTorque.y) annotation(
      Line(points = {{35.2, -24}, {30.2, -24}, {30.2, -32}, {27, -32}}, color = {0, 0, 127}));
    connect(ServoMotorVoltage.y, SM_Speed_Out) annotation(
      Line(points = {{-11, -54}, {110, -54}}, color = {0, 0, 127}));
    connect(ServoMotorVoltage.y, greaterThreshold1.u) annotation(
      Line(points = {{-11, -54}, {-3, -54}, {-3, -8}, {35, -8}}, color = {0, 0, 127}));
    connect(Voltage_Conversion.y, TM_Voltage.u2) annotation(
      Line(points = {{-23, 71}, {-17.5, 71}, {-17.5, 78}, {-10, 78}}, color = {0, 0, 127}));
    connect(booleanToInteger1.y, TM_Forward_Reverse_Out) annotation(
      Line(points = {{80.4, 60}, {86.1, 60}, {86.1, 20}, {110.1, 20}}, color = {255, 127, 0}));
    connect(Speed_Conversion.y, TM_Speed.u2) annotation(
      Line(points = {{-77, 59}, {-65, 59}}, color = {0, 0, 127}));
    connect(booleanToInteger1.u, greaterThreshold.y) annotation(
      Line(points = {{71.2, 60}, {64.2, 60}}, color = {255, 0, 255}));
    connect(add.y, greaterThreshold.u) annotation(
      Line(points = {{45, 60}, {45, 60.5}, {37.6, 60.5}, {37.6, 60}, {54.6, 60}}, color = {0, 0, 127}));
    connect(add.y, TM_Torque_Out) annotation(
      Line(points = {{45, 60}, {47.6, 60}, {47.6, 80}, {110, 80}}, color = {0, 0, 127}));
    connect(filter.y, SM_Torque_Conversion.u) annotation(
      Line(points = {{-53, -20}, {-46, -20}}, color = {0, 0, 127}));
    connect(TM_M_Speed_In, filter1.u) annotation(
      Line(points = {{-120, 88}, {-98, 88}}, color = {0, 0, 127}));
    connect(filter1.y, TM_Voltage.u1) annotation(
      Line(points = {{-75, 88}, {-10, 88}}, color = {0, 0, 127}));
    connect(filter1.y, TM_Speed.u1) annotation(
      Line(points = {{-75, 88}, {-70, 88}, {-70, 67}, {-65, 67}}, color = {0, 0, 127}));
    connect(filter2.u, SM_M_Speed_In) annotation(
      Line(points = {{-94, 20}, {-120, 20}}, color = {0, 0, 127}));
    connect(filter2.y, SM_Voltage_Conversion.u) annotation(
      Line(points = {{-71, 20}, {-64, 20}}, color = {0, 0, 127}));
    connect(TestMotortorque.y, add.u1) annotation(
      Line(points = {{8, 48}, {14, 48}, {14, 64}, {30, 64}}, color = {0, 0, 127}));
    connect(ramp.y, add.u2) annotation(
      Line(points = {{10, 16}, {20, 16}, {20, 56}, {30, 56}}, color = {0, 0, 127}));
    connect(TM_Start_Stop_Out, booleanToInteger1.y) annotation(
      Line(points = {{110, 60}, {80, 60}}, color = {255, 127, 0}));
  connect(filter.u, SM_M_Torque_In) annotation(
      Line(points = {{-76, -20}, {-120, -20}}, color = {0, 0, 127}));
    annotation(
      Icon(coordinateSystem(preserveAspectRatio = false)),
      Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-140, 120}, {160, -80}}), graphics = {Text(extent = {{120, 88}, {120, 88}}, textString = "text")}),
      experiment(StopTime = 300, StartTime = 0, Tolerance = 1e-06, Interval = 0.6));
  end SimpleTest;

  model SignalChecker
    Modelica.Blocks.Logical.LessEqualThreshold lessEqualThreshold(threshold = 10) annotation(
      Placement(visible = true, transformation(origin = {-50, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput Torque_In annotation(
      Placement(visible = true, transformation(origin = {-120, 70}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Logical.And and1 annotation(
      Placement(visible = true, transformation(origin = {-10, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Logical.GreaterEqual greaterEqual annotation(
      Placement(visible = true, transformation(origin = {-50, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant const(k = -10) annotation(
      Placement(visible = true, transformation(origin = {-90, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.BooleanToReal booleanToReal annotation(
      Placement(visible = true, transformation(origin = {30, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.BooleanToInteger booleanToInteger annotation(
      Placement(visible = true, transformation(origin = {30, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.IntegerOutput TM_stop_start annotation(
      Placement(visible = true, transformation(origin = {110, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Math.Product product annotation(
      Placement(visible = true, transformation(origin = {70, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput Test_motor_torque_out annotation(
      Placement(visible = true, transformation(origin = {110, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.IntegerOutput TM_forward_reverse annotation(
      Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.IntegerOutput TM_speed_torque annotation(
      Placement(visible = true, transformation(origin = {110, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {119, -39}, extent = {{-19, -19}, {19, 19}}, rotation = 0)));
  equation
    connect(lessEqualThreshold.u, Torque_In) annotation(
      Line(points = {{-62, 70}, {-120, 70}}, color = {0, 0, 127}));
    connect(lessEqualThreshold.y, and1.u1) annotation(
      Line(points = {{-39, 70}, {-22, 70}}, color = {255, 0, 255}));
    connect(greaterEqual.u1, Torque_In) annotation(
      Line(points = {{-62, 40}, {-70, 40}, {-70, 70}, {-120, 70}}, color = {0, 0, 127}));
    connect(const.y, greaterEqual.u2) annotation(
      Line(points = {{-79, 32}, {-62, 32}}, color = {0, 0, 127}));
    connect(greaterEqual.y, and1.u2) annotation(
      Line(points = {{-39, 40}, {-30, 40}, {-30, 61}, {-24, 61}, {-24, 61.5}, {-22, 61.5}, {-22, 62}}, color = {255, 0, 255}));
    connect(and1.y, booleanToReal.u) annotation(
      Line(points = {{1, 70}, {18, 70}}, color = {255, 0, 255}));
    connect(booleanToInteger.u, and1.y) annotation(
      Line(points = {{18, 20}, {10, 20}, {10, 70}, {1, 70}}, color = {255, 0, 255}));
    connect(booleanToReal.y, product.u2) annotation(
      Line(points = {{41, 70}, {50.5, 70}, {50.5, 74}, {58, 74}}, color = {0, 0, 127}));
    connect(product.u1, Torque_In) annotation(
      Line(points = {{58, 86}, {-70, 86}, {-70, 70}, {-120, 70}}, color = {0, 0, 127}));
    connect(product.y, Test_motor_torque_out) annotation(
      Line(points = {{81, 80}, {110, 80}}, color = {0, 0, 127}));
    connect(booleanToInteger.y, TM_stop_start) annotation(
      Line(points = {{42, 20}, {110, 20}}, color = {255, 127, 0}));
    connect(TM_forward_reverse, booleanToInteger.y) annotation(
      Line(points = {{110, 0}, {80, 0}, {80, 20}, {42, 20}}, color = {255, 127, 0}));
    connect(TM_speed_torque, booleanToInteger.y) annotation(
      Line(points = {{110, -20}, {60, -20}, {60, 20}, {42, 20}}, color = {255, 127, 0}));
    annotation(
      experiment(StartTime = 0, StopTime = 300, Tolerance = 1e-06, Interval = 0.6),
      Diagram(coordinateSystem(extent = {{-140, 100}, {140, -40}})));
  end SignalChecker;

  model SMModel
    parameter Real ServoMotorVoltageAmplitude = 2.5 "Voltage signal sendt to the Servomotor";
    parameter Real ServoMotorVoltageStartTime = 0.1 "Ramp up start time";
    parameter Real ServoMotorVoltageDuration = 0 "Duration of the ram up";
    parameter Real ServoMotorVoltageOffset = 0 "Initial voltage value of the servo motor";
    parameter Real ServoMotorTorqueAmplitude = 0.05 "Start torque for the servo motor";
    parameter Real ServoMotorTorqueStartTime = 0.1 "Ramp up start time";
    parameter Real ServoMotorTorqueDuration = 0 "Duration of the ramp up";
    parameter Real ServoMotorTorqueOffset = 0 "Initial torque value of the servo motor";
    parameter Real TorqueConversion = 17.2 "Converting the torque measured from the SM from voltage to torque";
    parameter Real SpeedConversion = 405*2*3.14159265/60 "Converting the measured speed of the SM from voltage to angular velosity";
    parameter Real DelayStartTime = 0.1 "Delay for first signal sendt in seconds";
    Modelica.Blocks.Logical.And and1 annotation(
      Placement(visible = true, transformation(origin = {6, 90}, extent = {{28, -60}, {48, -40}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput SM_speed_out annotation(
      Placement(visible = true, transformation(origin = {0, 48}, extent = {{100, -98}, {120, -78}}, rotation = 0), iconTransformation(origin = {120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Math.BooleanToInteger booleanToInteger annotation(
      Placement(visible = true, transformation(origin = {2, 90}, extent = {{60, -60}, {80, -40}}, rotation = 0)));
    Modelica.Blocks.Logical.GreaterThreshold greaterThreshold1 annotation(
      Placement(visible = true, transformation(origin = {14, 64}, extent = {{-14, -34}, {6, -14}}, rotation = 0)));
    Modelica.Blocks.Logical.GreaterThreshold greaterThreshold2 annotation(
      Placement(visible = true, transformation(origin = {14, 64}, extent = {{-14, -64}, {6, -44}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput SM_start_torque_out annotation(
      Placement(visible = true, transformation(origin = {0, 50}, extent = {{100, -80}, {120, -60}}, rotation = 0), iconTransformation(origin = {120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.IntegerOutput SM_start_stop_out annotation(
      Placement(visible = true, transformation(origin = {0, 90}, extent = {{100, -60}, {120, -40}}, rotation = 0), iconTransformation(origin = {120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Sources.Ramp ServoMotorTorque(duration = ServoMotorTorqueDuration, height = ServoMotorTorqueAmplitude, offset = ServoMotorTorqueOffset, startTime = ServoMotorTorqueStartTime) annotation(
      Placement(visible = true, transformation(origin = {46, -70}, extent = {{-80, 40}, {-60, 60}}, rotation = 0)));
    Modelica.Blocks.Sources.Ramp ServoMotorVoltage(duration = ServoMotorVoltageDuration, height = ServoMotorVoltageAmplitude, offset = ServoMotorVoltageOffset, startTime = ServoMotorVoltageStartTime) annotation(
      Placement(visible = true, transformation(origin = {14, -90}, extent = {{-80, 40}, {-60, 60}}, rotation = 0)));
    Modelica.Blocks.Math.Gain SpeedConv(k = SpeedConversion) annotation(
      Placement(visible = true, transformation(origin = {-50, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput AO1 annotation(
      Placement(visible = true, transformation(origin = {-118, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput SM_w_out annotation(
      Placement(visible = true, transformation(origin = {108, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Math.Gain TorqueConv(k = TorqueConversion) annotation(
      Placement(visible = true, transformation(origin = {10, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput AO2 annotation(
      Placement(visible = true, transformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Math.Product Power annotation(
      Placement(visible = true, transformation(origin = {56, -74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Nonlinear.Limiter div0protect(uMax = Modelica.Constants.inf, uMin = Modelica.Constants.small) annotation(
      Placement(visible = true, transformation(origin = {-76, -80}, extent = {{6, 6}, {-6, -6}}, rotation = -180)));
    Modelica.Blocks.Continuous.Filter filter(f_cut = 1) annotation(
      Placement(visible = true, transformation(origin = {-42, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Product product4 annotation(
      Placement(visible = true, transformation(origin = {62, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Step Delay(height = 1, offset = 0, startTime = DelayStartTime)  annotation(
      Placement(visible = true, transformation(origin = {16, 72}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
  equation
    connect(and1.u1, greaterThreshold1.y) annotation(
      Line(points = {{32, 40}, {21, 40}}, color = {255, 0, 255}));
    connect(booleanToInteger.u, and1.y) annotation(
      Line(points = {{60, 40}, {55, 40}}, color = {255, 0, 255}));
    connect(and1.u2, greaterThreshold2.y) annotation(
      Line(points = {{32, 32}, {25.5, 32}, {25.5, 10}, {21, 10}}, color = {255, 0, 255}));
    connect(booleanToInteger.y, SM_start_stop_out) annotation(
      Line(points = {{83, 40}, {110, 40}}, color = {255, 127, 0}));
    connect(ServoMotorTorque.y, SM_start_torque_out) annotation(
      Line(points = {{-13, -20}, {110, -20}}, color = {0, 0, 127}));
    connect(greaterThreshold2.u, ServoMotorTorque.y) annotation(
      Line(points = {{-2, 10}, {-10, 10}, {-10, -20}, {-13, -20}}, color = {0, 0, 127}));
    connect(ServoMotorVoltage.y, SM_speed_out) annotation(
      Line(points = {{-45, -40}, {110, -40}}, color = {0, 0, 127}));
    connect(ServoMotorVoltage.y, greaterThreshold1.u) annotation(
      Line(points = {{-45, -40}, {-43, -40}, {-43, 40}, {-2, 40}}, color = {0, 0, 127}));
    connect(TorqueConv.y, Power.u2) annotation(
      Line(points = {{21, -80}, {44, -80}}, color = {0, 0, 127}));
    connect(filter.u, div0protect.y) annotation(
      Line(points = {{-54, -80}, {-70, -80}}, color = {0, 0, 127}));
    connect(TorqueConv.u, filter.y) annotation(
      Line(points = {{-2, -80}, {-30, -80}}, color = {0, 0, 127}));
    connect(div0protect.u, AO2) annotation(
      Line(points = {{-84, -80}, {-120, -80}}, color = {0, 0, 127}));
    connect(SpeedConv.u, AO1) annotation(
      Line(points = {{-62, 80}, {-118, 80}}, color = {0, 0, 127}));
    connect(Power.u1, SpeedConv.y) annotation(
      Line(points = {{44, -68}, {28, -68}, {28, -60}, {-80, -60}, {-80, 60}, {-20, 60}, {-20, 80}, {-39, 80}}, color = {0, 0, 127}));
  connect(product4.y, SM_w_out) annotation(
      Line(points = {{74, 80}, {108, 80}}, color = {0, 0, 127}));
  connect(product4.u1, SpeedConv.y) annotation(
      Line(points = {{50, 86}, {-20, 86}, {-20, 80}, {-38, 80}}, color = {0, 0, 127}));
  connect(Delay.y, product4.u2) annotation(
      Line(points = {{24, 72}, {38, 72}, {38, 74}, {50, 74}}, color = {0, 0, 127}));
    annotation(
      Icon(coordinateSystem(preserveAspectRatio = false)),
      Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-140, 100}, {160, -100}})),
      experiment(StopTime = 300, StartTime = 0, Tolerance = 1e-06, Interval = 0.6));
  end SMModel;

  model HPM "Model of a hydropower system with a simple turbine turbine"
    extends Modelica.Icons.Example;
    parameter Real TorqueScaling = 100000 "Scale down of the torque signal out";
    parameter Real GuideVaneOpeningOffset = 0.7493 "Start possition of theguide vane";
    parameter Real GuideVaneOpeningChange = -0.04615 "Change in guide vane possition";
    parameter Real GuideVaneOpeningDuration = 30 "Duration of the change in seconds";
    parameter Real GuideVaneOpeningStartTime = 50 "Start time for change happening in seconds";
    OpenHPL.Waterway.Reservoir reservoir(useLevel = true, h_0 = 48) annotation(
      Placement(transformation(origin = {-90, 30}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Ramp control(duration = GuideVaneOpeningDuration, height = GuideVaneOpeningChange, offset = GuideVaneOpeningOffset, startTime = GuideVaneOpeningStartTime) annotation(
      Placement(transformation(origin = {-10, 70}, extent = {{-10, -10}, {10, 10}})));
    OpenHPL.Waterway.Pipe intake(H = 23, Vdot(fixed = true)) annotation(
      Placement(visible = true, transformation(origin = {-6, 0}, extent = {{-70, 20}, {-50, 40}}, rotation = 0)));
    OpenHPL.Waterway.Pipe discharge(H = 0.5, L = 600) annotation(
      Placement(transformation(extent = {{50, -10}, {70, 10}})));
    OpenHPL.Waterway.Reservoir tail(useLevel = true, h_0 = 5) annotation(
      Placement(transformation(origin = {90, 0}, extent = {{-10, 10}, {10, -10}}, rotation = 180)));
    replaceable OpenHPL.Waterway.Pipe penstock(D_i = 3, H = 428.5, L = 600, vertical = true) annotation(
      Placement(visible = true, transformation(origin = {-8, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0))) constrainedby OpenHPL.Interfaces.TwoContact annotation(
       Placement(transformation(origin = {0, 30}, extent = {{-10, -10}, {10, 10}})));
    OpenHPL.Waterway.SurgeTank surgeTank(h_0 = 69.9) annotation(
      Placement(visible = true, transformation(origin = {-40, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    OpenHPL.ElectroMech.Turbines.Turbine turbine(C_v = 3.7, ConstEfficiency = true, enable_nomSpeed = false, enable_f = false, enable_P_out = true, enable_w_in = false) annotation(
      Placement(visible = true, transformation(origin = {24, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    inner OpenHPL.Data data annotation(
      Placement(visible = true, transformation(origin = {-90, 74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant const(k = 48) annotation(
      Placement(visible = true, transformation(origin = {-90, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant const1(k = 5) annotation(
      Placement(visible = true, transformation(origin = {90, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput Tourque_out "Mechanical Output power" annotation(
      Placement(visible = true, transformation(origin = {-2, 12}, extent = {{102, 60}, {122, 80}}, rotation = 0), iconTransformation(origin = {-2, -70}, extent = {{102, 60}, {122, 80}}, rotation = 0)));
    Modelica.Blocks.Math.Division division1 annotation(
      Placement(visible = true, transformation(origin = {86, 82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Division division annotation(
      Placement(visible = true, transformation(origin = {34, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Blocks.Nonlinear.Limiter div0protect(uMax = Modelica.Constants.inf, uMin = Modelica.Constants.small) annotation(
      Placement(visible = true, transformation(origin = {40, 34}, extent = {{6, -6}, {-6, 6}}, rotation = -90)));
    Modelica.Blocks.Sources.Constant constant2(k = TorqueScaling) annotation(
      Placement(visible = true, transformation(origin = {82, 36}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Blocks.Interfaces.RealInput W_in annotation(
      Placement(visible = true, transformation(origin = {-120, -24}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  equation
    connect(turbine.o, discharge.i) annotation(
      Line(points = {{34, 10}, {38, 10}, {38, 0}, {50, 0}}, color = {28, 108, 200}));
    connect(control.y, turbine.u_t) annotation(
      Line(points = {{1, 70}, {16, 70}, {16, 22}}, color = {0, 0, 127}));
    connect(penstock.o, turbine.i) annotation(
      Line(points = {{2, 30}, {2.95, 30}, {2.95, 10}, {14, 10}}, color = {28, 108, 200}));
    connect(reservoir.o, intake.i) annotation(
      Line(points = {{-80, 30}, {-76, 30}}, color = {28, 108, 200}));
    connect(intake.o, surgeTank.i) annotation(
      Line(points = {{-56, 30}, {-50, 30}}, color = {28, 108, 200}));
    connect(discharge.o, tail.o) annotation(
      Line(points = {{70, 0}, {80, 0}}, color = {28, 108, 200}));
    connect(const.y, reservoir.level) annotation(
      Line(points = {{-101, 0}, {-118, 0}, {-118, 36}, {-102, 36}}, color = {0, 0, 127}));
    connect(const1.y, tail.level) annotation(
      Line(points = {{101, -30}, {114, -30}, {114, 6}, {102, 6}}, color = {0, 0, 127}));
    connect(constant2.y, division1.u2) annotation(
      Line(points = {{82, 47}, {82, 62}, {66, 62}, {66, 76}, {74, 76}}, color = {0, 0, 127}));
    connect(division.y, division1.u1) annotation(
      Line(points = {{34, 71}, {34, 88}, {74, 88}}, color = {0, 0, 127}));
    connect(division.u2, div0protect.y) annotation(
      Line(points = {{40, 48}, {40, 41}}, color = {0, 0, 127}));
    connect(division1.y, Tourque_out) annotation(
      Line(points = {{98, 82}, {110, 82}}, color = {0, 0, 127}));
    connect(division.u1, turbine.P_out) annotation(
      Line(points = {{28, 48}, {28, 21}}, color = {0, 0, 127}));
    connect(W_in, div0protect.u) annotation(
      Line(points = {{-120, -24}, {40, -24}, {40, 27}}, color = {0, 0, 127}));
    connect(penstock.i, surgeTank.o) annotation(
      Line(points = {{-18, 30}, {-30, 30}}, color = {0, 128, 255}));
    annotation(
      experiment(StopTime = 300, StartTime = 0, Tolerance = 1e-06, Interval = 0.6),
      __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian --fmuCMakeBuild=true --fmuRuntimeDepends=none ",
      Diagram(coordinateSystem(extent = {{-140, 100}, {120, -100}})));
  end HPM;

  model HydroM
    Modelica.Blocks.Interfaces.RealOutput SM_Start_Torque_out annotation(
      Placement(visible = true, transformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {146, -58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.IntegerOutput SM_Start_Stop_Out annotation(
      Placement(visible = true, transformation(origin = {110, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {126, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.IntegerOutput TM_Speed_Torque_Out annotation(
      Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {140, -54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.IntegerOutput TM_Start_Stop_Out annotation(
      Placement(visible = true, transformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput TM_Torque_Out annotation(
      Placement(visible = true, transformation(origin = {110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {124, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.IntegerOutput TM_Forward_Reverse_Out annotation(
      Placement(visible = true, transformation(origin = {110, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {128, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput SM_Speed_Out annotation(
      Placement(visible = true, transformation(origin = {110, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {122, -26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput SM_torque_in annotation(
      Placement(visible = true, transformation(origin = {-120, 2}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-102, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput SM_speed_in annotation(
      Placement(visible = true, transformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-102, 78}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    TestPackage.SMModel SMM annotation(
      Placement(visible = true, transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    TestPackage.HPM hpm annotation(
      Placement(visible = true, transformation(origin = {0, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    TestPackage.SignalChecker SC annotation(
      Placement(visible = true, transformation(origin = {50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(SMM.AO2, SM_torque_in) annotation(
      Line(points = {{-62, 2}, {-120, 2}}, color = {0, 0, 127}));
    connect(SMM.AO1, SM_speed_in) annotation(
      Line(points = {{-62, 18}, {-80, 18}, {-80, 80}, {-120, 80}}, color = {0, 0, 127}));
    connect(SMM.SM_start_stop_out, SM_Start_Stop_Out) annotation(
      Line(points = {{-38, 18}, {-20, 18}, {-20, 80}, {110, 80}}, color = {255, 127, 0}));
    connect(SMM.SM_speed_out, SM_Speed_Out) annotation(
      Line(points = {{-38, 6}, {-24, 6}, {-24, -20}, {110, -20}}, color = {0, 0, 127}));
    connect(SMM.SM_start_torque_out, SM_Start_Torque_out) annotation(
      Line(points = {{-38, 2}, {-30, 2}, {-30, -40}, {110, -40}}, color = {0, 0, 127}));
    connect(SMM.SM_w_out, hpm.W_in) annotation(
      Line(points = {{-38, 10}, {-12, 10}}, color = {0, 0, 127}));
    connect(hpm.Tourque_out, SC.Torque_In) annotation(
      Line(points = {{12, 10}, {38, 10}}, color = {0, 0, 127}));
    connect(SC.Test_motor_torque_out, TM_Torque_Out) annotation(
      Line(points = {{62, 18}, {68, 18}, {68, 60}, {110, 60}}, color = {0, 0, 127}));
    connect(SC.TM_stop_start, TM_Start_Stop_Out) annotation(
      Line(points = {{62, 14}, {74, 14}, {74, 40}, {110, 40}}, color = {255, 127, 0}));
    connect(SC.TM_forward_reverse, TM_Forward_Reverse_Out) annotation(
      Line(points = {{62, 10}, {80, 10}, {80, 20}, {110, 20}}, color = {255, 127, 0}));
    connect(SC.TM_speed_torque, TM_Speed_Torque_Out) annotation(
      Line(points = {{62, 6}, {80, 6}, {80, 0}, {110, 0}}, color = {255, 127, 0}));
    annotation(
      experiment(StartTime = 0, StopTime = 300, Tolerance = 1e-06, Interval = 0.6),
      __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian --fmuCMakeBuild=true --fmuRuntimeDepends=none ",
      Diagram(coordinateSystem(extent = {{-140, 100}, {140, -60}})));
  end HydroM;
  annotation(
    uses(Modelica(version = "4.0.0"), OpenHPL(version = "2.0.1")));
end TestPackage;
