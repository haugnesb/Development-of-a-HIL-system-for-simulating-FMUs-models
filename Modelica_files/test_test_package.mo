package TestPackage
  package ActiveWork
    package MotorTest
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
        Modelica.Blocks.Math.Gain Ao1(k = 450) annotation(
          Placement(visible = true, transformation(origin = {-66, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Gain Ao2(k = 1) annotation(
          Placement(visible = true, transformation(origin = {-70, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput AO1 annotation(
          Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-118, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput AO2 annotation(
          Placement(visible = true, transformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
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
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false)),
          Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})),
          experiment(StopTime = 300, StartTime = 0, Tolerance = 1e-06, Interval = 0.6));
      end OnlyIn;

      model TVFD
        parameter Real TestMotorTorqueAmplitude = 5 "Torque signal sendt to the test motor";
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
        Modelica.Blocks.Interfaces.IntegerOutput Speed_Torque annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.IntegerOutput Forward_Reverse annotation(
          Placement(visible = true, transformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(Start_Stop_Testmotor, booleanToInteger1.y) annotation(
          Line(points = {{110, 30}, {87, 30}}, color = {255, 127, 0}));
        connect(TestMotortorque.y, Test_motor_torque) annotation(
          Line(points = {{-59, 50}, {110, 50}}, color = {0, 0, 127}));
        connect(booleanToInteger1.u, greaterThreshold.y) annotation(
          Line(points = {{64, 30}, {21, 30}, {21, 32}}, color = {255, 0, 255}));
        connect(greaterThreshold.u, TestMotortorque.y) annotation(
          Line(points = {{-2, 32}, {-59, 32}, {-59, 50}}, color = {0, 0, 127}));
        connect(booleanToInteger1.y, Speed_Torque) annotation(
          Line(points = {{88, 30}, {88, 0}, {110, 0}}, color = {255, 127, 0}));
        connect(booleanToInteger1.y, Forward_Reverse) annotation(
          Line(points = {{88, 30}, {88, -40}, {110, -40}}, color = {255, 127, 0}));
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false)),
          Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})),
          experiment(StopTime = 300, StartTime = 0, Tolerance = 1e-06, Interval = 0.6));
      end TVFD;

      model SMModel
        parameter Real ServoMotorVoltageAmplitude = 2.5 "Voltage signal sendt to the Servomotor";
        parameter Real ServoMotorVoltageStartTime = 1 "Ramp up start time";
        parameter Real ServoMotorVoltageDuration = 0 "Duration of the ram up";
        parameter Real ServoMotorVoltageOffset = 0 "Initial voltage value of the servo motor";
        parameter Real ServoMotorTorqueAmplitude = 0.05 "Start torque for the servo motor";
        parameter Real ServoMotorTorqueStartTime = 1 "Ramp up start time";
        parameter Real ServoMotorTorqueDuration = 0 "Duration of the ramp up";
        parameter Real ServoMotorTorqueOffset = 0 "Initial torque value of the servo motor";
        parameter Real TorqueConversion = 17.2 "Converting the torque measured from the SM from voltage to torque";
        parameter Real SpeedConversion = 405*2*3.14159265/60 "Converting the measured speed of the SM from voltage to angular velosity";
        parameter Real DelayStartTime = 1 "Delay for first signal sendt in seconds";
        Modelica.Blocks.Logical.And and1 annotation(
          Placement(visible = true, transformation(origin = {-20, 88}, extent = {{28, -60}, {48, -40}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput SM_speed_out annotation(
          Placement(visible = true, transformation(origin = {0, 46}, extent = {{100, -98}, {120, -78}}, rotation = 0), iconTransformation(origin = {119, 1}, extent = {{-19, -19}, {19, 19}}, rotation = 0)));
        Modelica.Blocks.Math.BooleanToInteger booleanToInteger annotation(
          Placement(visible = true, transformation(origin = {-18, 88}, extent = {{60, -60}, {80, -40}}, rotation = 0)));
        Modelica.Blocks.Logical.GreaterThreshold greaterThreshold1 annotation(
          Placement(visible = true, transformation(origin = {-34, 62}, extent = {{-14, -34}, {6, -14}}, rotation = 0)));
        Modelica.Blocks.Logical.GreaterThreshold greaterThreshold2 annotation(
          Placement(visible = true, transformation(origin = {-18, 58}, extent = {{-14, -64}, {6, -44}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput SM_start_torque_out annotation(
          Placement(visible = true, transformation(origin = {2, 70}, extent = {{100, -80}, {120, -60}}, rotation = 0), iconTransformation(origin = {121, -39}, extent = {{-21, -21}, {21, 21}}, rotation = 0)));
        Modelica.Blocks.Interfaces.IntegerOutput SM_start_stop_out annotation(
          Placement(visible = true, transformation(origin = {0, 88}, extent = {{100, -60}, {120, -40}}, rotation = 0), iconTransformation(origin = {119, 41}, extent = {{-19, -19}, {19, 19}}, rotation = 0)));
        Modelica.Blocks.Sources.Ramp ServoMotorTorque(duration = ServoMotorTorqueDuration, height = ServoMotorTorqueAmplitude, offset = ServoMotorTorqueOffset, startTime = ServoMotorTorqueStartTime) annotation(
          Placement(visible = true, transformation(origin = {8, -68}, extent = {{-80, 40}, {-60, 60}}, rotation = 0)));
        Modelica.Blocks.Sources.Ramp ServoMotorVoltage(duration = ServoMotorVoltageDuration, height = ServoMotorVoltageAmplitude, offset = ServoMotorVoltageOffset, startTime = ServoMotorVoltageStartTime) annotation(
          Placement(visible = true, transformation(origin = {-20, -100}, extent = {{-80, 40}, {-60, 60}}, rotation = 0)));
        Modelica.Blocks.Math.Gain SpeedConv(k = SpeedConversion) annotation(
          Placement(visible = true, transformation(origin = {-68, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput AO1 annotation(
          Placement(visible = true, transformation(origin = {-118, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput SM_w_out annotation(
          Placement(visible = true, transformation(origin = {112, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {119, 81}, extent = {{-19, -19}, {19, 19}}, rotation = 0)));
        Modelica.Blocks.Math.Gain TorqueConv(k = TorqueConversion) annotation(
          Placement(visible = true, transformation(origin = {0, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput AO2 annotation(
          Placement(visible = true, transformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-122, -78}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        Modelica.Blocks.Math.Product Power annotation(
          Placement(visible = true, transformation(origin = {40, -62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Product product annotation(
          Placement(visible = true, transformation(origin = {68, 82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Step Delay(height = 1, offset = 0, startTime = DelayStartTime) annotation(
          Placement(visible = true, transformation(origin = {-2, 68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Continuous.Filter filter(f_cut = 1) annotation(
          Placement(visible = true, transformation(origin = {-48, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Nonlinear.Limiter div0protect(uMax = Modelica.Constants.inf, uMin = Modelica.Constants.small) annotation(
          Placement(visible = true, transformation(origin = {-82, -80}, extent = {{6, 6}, {-6, -6}}, rotation = -180)));
      equation
        connect(and1.u1, greaterThreshold1.y) annotation(
          Line(points = {{6, 38}, {-27, 38}}, color = {255, 0, 255}));
        connect(booleanToInteger.u, and1.y) annotation(
          Line(points = {{40, 38}, {29, 38}}, color = {255, 0, 255}));
        connect(and1.u2, greaterThreshold2.y) annotation(
          Line(points = {{6, 30}, {-2.5, 30}, {-2.5, 4}, {-11, 4}}, color = {255, 0, 255}));
        connect(booleanToInteger.y, SM_start_stop_out) annotation(
          Line(points = {{63, 38}, {110, 38}}, color = {255, 127, 0}));
        connect(ServoMotorTorque.y, SM_start_torque_out) annotation(
          Line(points = {{-51, -18}, {31.5, -18}, {31.5, 0}, {112, 0}}, color = {0, 0, 127}));
        connect(greaterThreshold2.u, ServoMotorTorque.y) annotation(
          Line(points = {{-34, 4}, {-40, 4}, {-40, -18}, {-51, -18}}, color = {0, 0, 127}));
        connect(ServoMotorVoltage.y, SM_speed_out) annotation(
          Line(points = {{-79, -50}, {15.5, -50}, {15.5, -42}, {110, -42}}, color = {0, 0, 127}));
        connect(ServoMotorVoltage.y, greaterThreshold1.u) annotation(
          Line(points = {{-79, -50}, {-79, 38}, {-50, 38}}, color = {0, 0, 127}));
        connect(SpeedConv.u, AO1) annotation(
          Line(points = {{-80, 80}, {-118, 80}}, color = {0, 0, 127}));
        connect(TorqueConv.y, Power.u2) annotation(
          Line(points = {{11, -80}, {45, -80}, {45, -68}, {28, -68}}, color = {0, 0, 127}));
        connect(SpeedConv.y, Power.u1) annotation(
          Line(points = {{-56, 80}, {-24, 80}, {-24, -56}, {28, -56}}, color = {0, 0, 127}));
        connect(product.y, SM_w_out) annotation(
          Line(points = {{80, 82}, {112, 82}, {112, 80}}, color = {0, 0, 127}));
        connect(Delay.y, product.u2) annotation(
          Line(points = {{10, 68}, {56, 68}, {56, 76}}, color = {0, 0, 127}));
        connect(SpeedConv.y, product.u1) annotation(
          Line(points = {{-56, 80}, {-24, 80}, {-24, 88}, {56, 88}}, color = {0, 0, 127}));
        connect(filter.y, TorqueConv.u) annotation(
          Line(points = {{-36, -80}, {-12, -80}}, color = {0, 0, 127}));
        connect(filter.u, div0protect.y) annotation(
          Line(points = {{-60, -80}, {-76, -80}}, color = {0, 0, 127}));
        connect(div0protect.u, AO2) annotation(
          Line(points = {{-90, -80}, {-120, -80}}, color = {0, 0, 127}));
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false)),
          Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})),
          experiment(StopTime = 300, StartTime = 0, Tolerance = 1e-06, Interval = 0.6));
      end SMModel;

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
        Modelica.Blocks.Math.Gain SM_Torque_Conversion(k = 17.2) annotation(
          Placement(visible = true, transformation(origin = {-34, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput SM_M_Speed_In annotation(
          Placement(visible = true, transformation(origin = {-120, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput SM_M_Torque_In annotation(
          Placement(visible = true, transformation(origin = {-120, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        Modelica.Blocks.Interfaces.IntegerOutput TM_Forward_Reverse_Out annotation(
          Placement(visible = true, transformation(origin = {110, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant Speed_Conversion(k = 455) annotation(
          Placement(visible = true, transformation(origin = {-87, 59}, extent = {{-9, -9}, {9, 9}}, rotation = 0)));
        Modelica.Blocks.Nonlinear.Limiter div0protect(uMax = Modelica.Constants.inf, uMin = Modelica.Constants.small) annotation(
          Placement(visible = true, transformation(origin = {-88, -20}, extent = {{6, 6}, {-6, -6}}, rotation = -180)));
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
        connect(div0protect.u, SM_M_Torque_In) annotation(
          Line(points = {{-95, -20}, {-120, -20}}, color = {0, 0, 127}));
        connect(booleanToInteger1.u, greaterThreshold.y) annotation(
          Line(points = {{71.2, 60}, {64.2, 60}}, color = {255, 0, 255}));
        connect(add.y, greaterThreshold.u) annotation(
          Line(points = {{45, 60}, {45, 60.5}, {37.6, 60.5}, {37.6, 60}, {54.6, 60}}, color = {0, 0, 127}));
        connect(add.y, TM_Torque_Out) annotation(
          Line(points = {{45, 60}, {47.6, 60}, {47.6, 80}, {110, 80}}, color = {0, 0, 127}));
        connect(filter.y, SM_Torque_Conversion.u) annotation(
          Line(points = {{-53, -20}, {-46, -20}}, color = {0, 0, 127}));
        connect(filter.u, div0protect.y) annotation(
          Line(points = {{-76, -20}, {-81, -20}}, color = {0, 0, 127}));
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
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false)),
          Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-140, 120}, {160, -80}}), graphics = {Text(extent = {{120, 88}, {120, 88}}, textString = "text")}),
          experiment(StopTime = 300, StartTime = 0, Tolerance = 1e-06, Interval = 0.6));
      end SimpleTest;
    end MotorTest;
  end ActiveWork;

  package Hydro
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
        experiment(StartTime = 0, StopTime = 300, Tolerance = 1e-6, Interval = 0.6),
        Diagram(coordinateSystem(extent = {{-140, 100}, {140, -40}})));
    end SignalChecker;

    model SMModel1
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
      Modelica.Blocks.Sources.Step step(startTime = DelayStartTime) annotation(
        Placement(visible = true, transformation(origin = {18, 74}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
      Modelica.Blocks.Math.Product product annotation(
        Placement(visible = true, transformation(origin = {56, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
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
      connect(step.y, product.u2) annotation(
        Line(points = {{27, 74}, {44, 74}}, color = {0, 0, 127}));
      connect(SpeedConv.y, product.u1) annotation(
        Line(points = {{-39, 80}, {-20, 80}, {-20, 86}, {42, 86}, {42, 84}, {44, 84}, {44, 86}}, color = {0, 0, 127}));
      connect(product.y, SM_w_out) annotation(
        Line(points = {{67, 80}, {108, 80}}, color = {0, 0, 127}));
      connect(Power.u1, SpeedConv.y) annotation(
        Line(points = {{44, -68}, {28, -68}, {28, -60}, {-80, -60}, {-80, 60}, {-20, 60}, {-20, 80}, {-39, 80}}, color = {0, 0, 127}));
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = false)),
        Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-140, 100}, {160, -100}})),
        experiment(StopTime = 300, StartTime = 0, Tolerance = 1e-06, Interval = 0.6));
    end SMModel1;

    model HPM "Model of a hydropower system with a simple turbine turbine"
      extends Modelica.Icons.Example;
      parameter Real TorqueScaling = 200000 "Scale down of the torque signal out";
      parameter Real GuideVaneOpeningOffset = 0.7493 "Start possition of theguide vane";
      parameter Real GuideVaneOpeningChange = -0.04615 "Change in guide vane possition";
      parameter Real GuideVaneOpeningDuration = 30 "Duration of the change in seconds";
      parameter Real GuideVaneOpeningStartTime = 50 "Start time for change happening in seconds";
      OpenHPL.Waterway.Reservoir reservoir(useLevel = true, h_0 = 48) annotation(
        Placement(transformation(origin = {-90, 30}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Sources.Ramp control(duration = GuideVaneOpeningDuration, height = GuideVaneOpeningChange, offset = GuideVaneOpeningOffset, startTime = GuideVaneOpeningStartTime) annotation(
        Placement(transformation(origin = {-10, 70}, extent = {{-10, -10}, {10, 10}})));
      OpenHPL.Waterway.Pipe intake(H = 23, Vdot(fixed = true)) annotation(
        Placement(transformation(extent = {{-70, 20}, {-50, 40}})));
      OpenHPL.Waterway.Pipe discharge(H = 0.5, L = 600) annotation(
        Placement(transformation(extent = {{50, -10}, {70, 10}})));
      OpenHPL.Waterway.Reservoir tail(useLevel = true, h_0 = 5) annotation(
        Placement(transformation(origin = {90, 0}, extent = {{-10, 10}, {10, -10}}, rotation = 180)));
      replaceable OpenHPL.Waterway.Pipe penstock(D_i = 3, H = 428.5, L = 600, vertical = true) constrainedby OpenHPL.Interfaces.TwoContact annotation(
         Placement(transformation(origin = {0, 30}, extent = {{-10, -10}, {10, 10}})));
      OpenHPL.Waterway.SurgeTank surgeTank(h_0 = 69.9) annotation(
        Placement(transformation(origin = {-30, 30}, extent = {{-10, -10}, {10, 10}})));
      OpenHPL.ElectroMech.Turbines.Turbine turbine(C_v = 3.7, ConstEfficiency = true, enable_nomSpeed = false, enable_f = false, enable_P_out = true, enable_w_in = false) annotation(
        Placement(transformation(origin = {30, 10}, extent = {{-10, -10}, {10, 10}})));
      inner OpenHPL.Data data annotation(
        Placement(transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Sources.Constant const(k = 48) annotation(
        Placement(visible = true, transformation(origin = {-80, -12}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant const1(k = 5) annotation(
        Placement(visible = true, transformation(origin = {74, -68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput Tourque_out "Mechanical Output power" annotation(
        Placement(visible = true, transformation(origin = {6, 12}, extent = {{102, 60}, {122, 80}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{102, 60}, {122, 80}}, rotation = 0)));
      Modelica.Blocks.Math.Division division1 annotation(
        Placement(visible = true, transformation(origin = {86, 82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Division division annotation(
        Placement(visible = true, transformation(origin = {36, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Blocks.Nonlinear.Limiter div0protect(uMax = Modelica.Constants.inf, uMin = Modelica.Constants.small) annotation(
        Placement(visible = true, transformation(origin = {54, 34}, extent = {{6, -6}, {-6, 6}}, rotation = -90)));
      Modelica.Blocks.Sources.Constant constant2(k = TorqueScaling) annotation(
        Placement(visible = true, transformation(origin = {82, 36}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Blocks.Interfaces.RealInput W_in annotation(
        Placement(visible = true, transformation(origin = {-120, -42}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-122, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    equation
      connect(turbine.o, discharge.i) annotation(
        Line(points = {{40, 10}, {44, 10}, {44, 0}, {50, 0}}, color = {28, 108, 200}));
      connect(control.y, turbine.u_t) annotation(
        Line(points = {{1, 70}, {16, 70}, {16, 40}, {22, 40}, {22, 22}}, color = {0, 0, 127}));
      connect(penstock.o, turbine.i) annotation(
        Line(points = {{10, 30}, {14.95, 30}, {14.95, 10}, {20, 10}}, color = {28, 108, 200}));
      connect(reservoir.o, intake.i) annotation(
        Line(points = {{-80, 30}, {-70, 30}}, color = {28, 108, 200}));
      connect(intake.o, surgeTank.i) annotation(
        Line(points = {{-50, 30}, {-40, 30}}, color = {28, 108, 200}));
      connect(surgeTank.o, penstock.i) annotation(
        Line(points = {{-20, 30}, {-10, 30}}, color = {28, 108, 200}));
      connect(discharge.o, tail.o) annotation(
        Line(points = {{70, 0}, {80, 0}}, color = {28, 108, 200}));
      connect(const.y, reservoir.level) annotation(
        Line(points = {{-91, -12}, {-100, -12}, {-100, -14}, {-118, -14}, {-118, 36}, {-102, 36}}, color = {0, 0, 127}));
      connect(const1.y, tail.level) annotation(
        Line(points = {{85, -68}, {114, -68}, {114, 6}, {102, 6}}, color = {0, 0, 127}));
      connect(constant2.y, division1.u2) annotation(
        Line(points = {{82, 47}, {82, 62}, {52, 62}, {52, 76}, {74, 76}}, color = {0, 0, 127}));
      connect(division.y, division1.u1) annotation(
        Line(points = {{36, 71}, {38, 71}, {38, 88}, {74, 88}}, color = {0, 0, 127}));
      connect(division.u2, div0protect.y) annotation(
        Line(points = {{42, 48}, {42, 45.15}, {54, 45.15}, {54, 41}}, color = {0, 0, 127}));
      connect(division1.y, Tourque_out) annotation(
        Line(points = {{98, 82}, {118, 82}}, color = {0, 0, 127}));
      connect(division.u1, turbine.P_out) annotation(
        Line(points = {{30, 48}, {30, 22}, {34, 22}}, color = {0, 0, 127}));
      connect(W_in, div0protect.u) annotation(
        Line(points = {{-120, -42}, {46, -42}, {46, 26}, {54, 26}}, color = {0, 0, 127}));
      annotation(
        experiment(StopTime = 300, StartTime = 0, Tolerance = 1e-06, Interval = 0.6));
    end HPM;

    model HydroM
      TestPackage.Hydro.SignalChecker SignalChecker annotation(
        Placement(visible = true, transformation(origin = {38, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
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
        Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-102, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput SM_speed_in annotation(
        Placement(visible = true, transformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-102, 78}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      HPM hpm annotation(
        Placement(visible = true, transformation(origin = {-2, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      SMModel1 sMModel1 annotation(
        Placement(visible = true, transformation(origin = {-54, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(SignalChecker.TM_speed_torque, TM_Speed_Torque_Out) annotation(
        Line(points = {{50, 4}, {66, 4}, {66, 0}, {110, 0}}, color = {255, 127, 0}));
      connect(SignalChecker.TM_forward_reverse, TM_Forward_Reverse_Out) annotation(
        Line(points = {{50, 8}, {80, 8}, {80, 20}, {110, 20}}, color = {255, 127, 0}));
      connect(SignalChecker.TM_stop_start, TM_Start_Stop_Out) annotation(
        Line(points = {{50, 12}, {70, 12}, {70, 40}, {110, 40}}, color = {255, 127, 0}));
      connect(SignalChecker.Test_motor_torque_out, TM_Torque_Out) annotation(
        Line(points = {{50, 16}, {60, 16}, {60, 60}, {110, 60}}, color = {0, 0, 127}));
      connect(hpm.Tourque_out, SignalChecker.Torque_In) annotation(
        Line(points = {{10, 22}, {18, 22}, {18, 8}, {26, 8}}, color = {0, 0, 127}));
      connect(sMModel1.SM_w_out, hpm.W_in) annotation(
        Line(points = {{-42, 18}, {-14, 18}, {-14, 14}}, color = {0, 0, 127}));
      connect(sMModel1.SM_start_stop_out, SM_Start_Stop_Out) annotation(
        Line(points = {{-42, 14}, {-30, 14}, {-30, 80}, {110, 80}}, color = {255, 127, 0}));
      connect(sMModel1.SM_speed_out, SM_Speed_Out) annotation(
        Line(points = {{-42, 10}, {-30, 10}, {-30, -20}, {110, -20}}, color = {0, 0, 127}));
      connect(sMModel1.SM_start_torque_out, SM_Start_Torque_out) annotation(
        Line(points = {{-42, 6}, {-34, 6}, {-34, -40}, {110, -40}}, color = {0, 0, 127}));
      connect(SM_torque_in, sMModel1.AO2) annotation(
        Line(points = {{-120, 0}, {-66, 0}, {-66, 2}}, color = {0, 0, 127}));
      connect(sMModel1.AO1, SM_speed_in) annotation(
        Line(points = {{-66, 18}, {-80, 18}, {-80, 80}, {-120, 80}}, color = {0, 0, 127}));
      annotation(
        experiment(StartTime = 0, StopTime = 300, Tolerance = 1e-06, Interval = 0.6));
    end HydroM;

    model HPM2 "Model of a hydropower system with a simple turbine turbine"
      extends Modelica.Icons.Example;
      parameter Real TorqueScaling = 200000 "Scale down of the torque signal out";
      parameter Real GuideVaneOpeningOffset = 0.7493 "Start possition of theguide vane";
      parameter Real GuideVaneOpeningChange = -0.04615 "Change in guide vane possition";
      parameter Real GuideVaneOpeningDuration = 30 "Duration of the change in seconds";
      parameter Real GuideVaneOpeningStartTime = 50 "Start time for change happening in seconds";
      OpenHPL.Waterway.Reservoir reservoir(useLevel = true, h_0 = 48) annotation(
        Placement(transformation(origin = {-90, 30}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Sources.Ramp control(duration = GuideVaneOpeningDuration, height = GuideVaneOpeningChange, offset = GuideVaneOpeningOffset, startTime = GuideVaneOpeningStartTime) annotation(
        Placement(transformation(origin = {-10, 70}, extent = {{-10, -10}, {10, 10}})));
      OpenHPL.Waterway.Pipe intake(H = 23, Vdot(fixed = true)) annotation(
        Placement(transformation(extent = {{-70, 20}, {-50, 40}})));
      OpenHPL.Waterway.Pipe discharge(H = 0.5, L = 600) annotation(
        Placement(transformation(extent = {{50, -10}, {70, 10}})));
      OpenHPL.Waterway.Reservoir tail(useLevel = true, h_0 = 5) annotation(
        Placement(transformation(origin = {90, 0}, extent = {{-10, 10}, {10, -10}}, rotation = 180)));
      replaceable OpenHPL.Waterway.Pipe penstock(D_i = 3, H = 428.5, L = 600, vertical = true) constrainedby OpenHPL.Interfaces.TwoContact annotation(
         Placement(transformation(origin = {0, 30}, extent = {{-10, -10}, {10, 10}})));
      OpenHPL.Waterway.SurgeTank surgeTank(h_0 = 69.9) annotation(
        Placement(transformation(origin = {-30, 30}, extent = {{-10, -10}, {10, 10}})));
      OpenHPL.ElectroMech.Turbines.Turbine turbine(C_v = 3.7, ConstEfficiency = true, enable_nomSpeed = false, enable_f = false, enable_P_out = true, enable_w_in = true) annotation(
        Placement(transformation(origin = {30, 10}, extent = {{-10, -10}, {10, 10}})));
      inner OpenHPL.Data data annotation(
        Placement(transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Sources.Constant const(k = 48) annotation(
        Placement(visible = true, transformation(origin = {-80, -12}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant const1(k = 5) annotation(
        Placement(visible = true, transformation(origin = {74, -68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput Tourque_out "Mechanical Output power" annotation(
        Placement(visible = true, transformation(origin = {6, 12}, extent = {{102, 60}, {122, 80}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{102, 60}, {122, 80}}, rotation = 0)));
      Modelica.Blocks.Math.Division division1 annotation(
        Placement(visible = true, transformation(origin = {86, 82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Division division annotation(
        Placement(visible = true, transformation(origin = {36, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Blocks.Nonlinear.Limiter div0protect(uMax = Modelica.Constants.inf, uMin = Modelica.Constants.small) annotation(
        Placement(visible = true, transformation(origin = {54, 34}, extent = {{6, -6}, {-6, 6}}, rotation = -90)));
      Modelica.Blocks.Sources.Constant constant2(k = TorqueScaling) annotation(
        Placement(visible = true, transformation(origin = {82, 36}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Blocks.Interfaces.RealInput W_in annotation(
        Placement(visible = true, transformation(origin = {-120, -42}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-122, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant constant1(k = 52) annotation(
        Placement(visible = true, transformation(origin = {2, -82}, extent = {{10, -10}, {-10, 10}}, rotation = 180)));
    equation
      connect(turbine.o, discharge.i) annotation(
        Line(points = {{40, 10}, {44, 10}, {44, 0}, {50, 0}}, color = {28, 108, 200}));
      connect(control.y, turbine.u_t) annotation(
        Line(points = {{1, 70}, {16, 70}, {16, 40}, {22, 40}, {22, 22}}, color = {0, 0, 127}));
      connect(penstock.o, turbine.i) annotation(
        Line(points = {{10, 30}, {14.95, 30}, {14.95, 10}, {20, 10}}, color = {28, 108, 200}));
      connect(reservoir.o, intake.i) annotation(
        Line(points = {{-80, 30}, {-70, 30}}, color = {28, 108, 200}));
      connect(intake.o, surgeTank.i) annotation(
        Line(points = {{-50, 30}, {-40, 30}}, color = {28, 108, 200}));
      connect(surgeTank.o, penstock.i) annotation(
        Line(points = {{-20, 30}, {-10, 30}}, color = {28, 108, 200}));
      connect(discharge.o, tail.o) annotation(
        Line(points = {{70, 0}, {80, 0}}, color = {28, 108, 200}));
      connect(const.y, reservoir.level) annotation(
        Line(points = {{-91, -12}, {-100, -12}, {-100, -14}, {-118, -14}, {-118, 36}, {-102, 36}}, color = {0, 0, 127}));
      connect(const1.y, tail.level) annotation(
        Line(points = {{85, -68}, {114, -68}, {114, 6}, {102, 6}}, color = {0, 0, 127}));
      connect(constant2.y, division1.u2) annotation(
        Line(points = {{82, 47}, {82, 62}, {52, 62}, {52, 76}, {74, 76}}, color = {0, 0, 127}));
      connect(division.y, division1.u1) annotation(
        Line(points = {{36, 71}, {38, 71}, {38, 88}, {74, 88}}, color = {0, 0, 127}));
      connect(division.u2, div0protect.y) annotation(
        Line(points = {{42, 48}, {42, 45.15}, {54, 45.15}, {54, 41}}, color = {0, 0, 127}));
      connect(division1.y, Tourque_out) annotation(
        Line(points = {{98, 82}, {118, 82}}, color = {0, 0, 127}));
      connect(division.u1, turbine.P_out) annotation(
        Line(points = {{30, 48}, {30, 22}, {34, 22}}, color = {0, 0, 127}));
      connect(W_in, div0protect.u) annotation(
        Line(points = {{-120, -42}, {46, -42}, {46, 26}, {54, 26}}, color = {0, 0, 127}));
      connect(constant1.y, turbine.w_in) annotation(
        Line(points = {{14, -82}, {22, -82}, {22, -2}}, color = {0, 0, 127}));
      annotation(
        experiment(StopTime = 300, StartTime = 0, Tolerance = 1e-06, Interval = 0.6));
    end HPM2;

    model HydroM2
      TestPackage.Hydro.SignalChecker SignalChecker annotation(
        Placement(visible = true, transformation(origin = {38, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
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
        Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-102, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput SM_speed_in annotation(
        Placement(visible = true, transformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-102, 78}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      TestPackage.Hydro.SMModel1 sMModel1 annotation(
        Placement(visible = true, transformation(origin = {-56, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      HPM2 hpm2 annotation(
        Placement(visible = true, transformation(origin = {2, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(SignalChecker.TM_speed_torque, TM_Speed_Torque_Out) annotation(
        Line(points = {{50, 4}, {66, 4}, {66, 0}, {110, 0}}, color = {255, 127, 0}));
      connect(SignalChecker.TM_forward_reverse, TM_Forward_Reverse_Out) annotation(
        Line(points = {{50, 8}, {80, 8}, {80, 20}, {110, 20}}, color = {255, 127, 0}));
      connect(SignalChecker.TM_stop_start, TM_Start_Stop_Out) annotation(
        Line(points = {{50, 12}, {70, 12}, {70, 40}, {110, 40}}, color = {255, 127, 0}));
      connect(SignalChecker.Test_motor_torque_out, TM_Torque_Out) annotation(
        Line(points = {{50, 16}, {60, 16}, {60, 60}, {110, 60}}, color = {0, 0, 127}));
      connect(sMModel1.SM_start_stop_out, SM_Start_Stop_Out) annotation(
        Line(points = {{-44, 14}, {-18, 14}, {-18, 80}, {110, 80}}, color = {255, 127, 0}));
      connect(sMModel1.SM_speed_out, SM_Speed_Out) annotation(
        Line(points = {{-44, 10}, {-28, 10}, {-28, -20}, {110, -20}}, color = {0, 0, 127}));
      connect(sMModel1.SM_start_torque_out, SM_Start_Torque_out) annotation(
        Line(points = {{-44, 6}, {-38, 6}, {-38, -40}, {110, -40}}, color = {0, 0, 127}));
      connect(SM_torque_in, sMModel1.AO2) annotation(
        Line(points = {{-120, 0}, {-68, 0}, {-68, 2}}, color = {0, 0, 127}));
      connect(sMModel1.AO1, SM_speed_in) annotation(
        Line(points = {{-68, 18}, {-80, 18}, {-80, 80}, {-120, 80}}, color = {0, 0, 127}));
      connect(hpm2.W_in, sMModel1.SM_w_out) annotation(
        Line(points = {{-10, 12}, {-28, 12}, {-28, 18}, {-44, 18}}, color = {0, 0, 127}));
      connect(hpm2.Tourque_out, SignalChecker.Torque_In) annotation(
        Line(points = {{14, 20}, {20, 20}, {20, 8}, {26, 8}}, color = {0, 0, 127}));
      annotation(
        experiment(StartTime = 0, StopTime = 300, Tolerance = 1e-06, Interval = 0.6),
        __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian --fmuCMakeBuild=true --fmuRuntimeDepends=none ");
    end HydroM2;

    model HPM4 "Model of a hydropower system with a simple turbine turbine"
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
        Placement(visible = true, transformation(origin = {-12, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0))) constrainedby OpenHPL.Interfaces.TwoContact annotation(
         Placement(transformation(origin = {0, 30}, extent = {{-10, -10}, {10, 10}})));
      OpenHPL.Waterway.SurgeTank surgeTank(h_0 = 69.9) annotation(
        Placement(visible = true, transformation(origin = {-40, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      OpenHPL.ElectroMech.Turbines.Turbine turbine(C_v = 3.7, ConstEfficiency = true, enable_nomSpeed = false, enable_f = false, enable_P_out = true, enable_w_in = true) annotation(
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
      Modelica.Blocks.Math.Add add4 annotation(
        Placement(visible = true, transformation(origin = {2, -44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant W_in_constant(k = 12) annotation(
        Placement(visible = true, transformation(origin = {-90, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(turbine.o, discharge.i) annotation(
        Line(points = {{34, 10}, {38, 10}, {38, 0}, {50, 0}}, color = {28, 108, 200}));
      connect(control.y, turbine.u_t) annotation(
        Line(points = {{1, 70}, {16, 70}, {16, 22}}, color = {0, 0, 127}));
      connect(penstock.o, turbine.i) annotation(
        Line(points = {{-2, 30}, {2.95, 30}, {2.95, 10}, {14, 10}}, color = {28, 108, 200}));
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
      connect(W_in, add4.u1) annotation(
        Line(points = {{-120, -24}, {-22, -24}, {-22, -38}, {-10, -38}}, color = {0, 0, 127}));
      connect(add4.y, turbine.w_in) annotation(
        Line(points = {{13, -44}, {17, -44}, {17, -2}, {16, -2}}, color = {0, 0, 127}));
      connect(W_in_constant.y, add4.u2) annotation(
        Line(points = {{-79, -50}, {-10, -50}}, color = {0, 0, 127}));
      connect(surgeTank.o, penstock.i) annotation(
        Line(points = {{-30, 30}, {-22, 30}}, color = {0, 128, 255}));
      annotation(
        experiment(StopTime = 300, StartTime = 0, Tolerance = 1e-06, Interval = 0.6),
        __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian --fmuCMakeBuild=true --fmuRuntimeDepends=none ",
        Diagram(coordinateSystem(extent = {{-140, 100}, {120, -100}})));
    end HPM4;

    model HydroM4
      TestPackage.Hydro.SignalChecker SignalChecker annotation(
        Placement(visible = true, transformation(origin = {38, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
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
      TestPackage.Hydro.SMModel1 sMModel1 annotation(
        Placement(visible = true, transformation(origin = {-56, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      TestPackage.Hydro.HPM4 hpm4 annotation(
        Placement(visible = true, transformation(origin = {-2, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(SignalChecker.TM_speed_torque, TM_Speed_Torque_Out) annotation(
        Line(points = {{50, 6}, {66, 6}, {66, 0}, {110, 0}}, color = {255, 127, 0}));
      connect(SignalChecker.TM_forward_reverse, TM_Forward_Reverse_Out) annotation(
        Line(points = {{50, 10}, {80, 10}, {80, 20}, {110, 20}}, color = {255, 127, 0}));
      connect(SignalChecker.TM_stop_start, TM_Start_Stop_Out) annotation(
        Line(points = {{50, 14}, {70, 14}, {70, 40}, {110, 40}}, color = {255, 127, 0}));
      connect(SignalChecker.Test_motor_torque_out, TM_Torque_Out) annotation(
        Line(points = {{50, 18}, {60, 18}, {60, 60}, {110, 60}}, color = {0, 0, 127}));
      connect(SM_torque_in, sMModel1.AO2) annotation(
        Line(points = {{-120, 2}, {-68, 2}}, color = {0, 0, 127}));
      connect(sMModel1.AO1, SM_speed_in) annotation(
        Line(points = {{-68, 18}, {-80, 18}, {-80, 80}, {-120, 80}}, color = {0, 0, 127}));
      connect(SignalChecker.Torque_In, hpm4.Tourque_out) annotation(
        Line(points = {{26, 10}, {10, 10}}, color = {0, 0, 127}));
      connect(hpm4.W_in, sMModel1.SM_w_out) annotation(
        Line(points = {{-14, 10}, {-44, 10}}, color = {0, 0, 127}));
      connect(sMModel1.SM_start_stop_out, SM_Start_Stop_Out) annotation(
        Line(points = {{-44, 18}, {-20, 18}, {-20, 80}, {110, 80}}, color = {255, 127, 0}));
      connect(sMModel1.SM_start_torque_out, SM_Start_Torque_out) annotation(
        Line(points = {{-44, 2}, {-40, 2}, {-40, -40}, {110, -40}}, color = {0, 0, 127}));
      connect(sMModel1.SM_speed_out, SM_Speed_Out) annotation(
        Line(points = {{-44, 6}, {-28, 6}, {-28, -20}, {110, -20}}, color = {0, 0, 127}));
      annotation(
        experiment(StartTime = 0, StopTime = 300, Tolerance = 1e-06, Interval = 0.6),
        __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian --fmuCMakeBuild=true --fmuRuntimeDepends=none ",
        Diagram(coordinateSystem(extent = {{-140, 100}, {140, -60}})));
    end HydroM4;

    model tt
      HPM4 hpm4 annotation(
        Placement(visible = true, transformation(origin = {-10, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Ramp ramp(duration = 0.9, height = 106.5, offset = 0, startTime = 0.1) annotation(
        Placement(visible = true, transformation(origin = {-66, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      SignalChecker signalChecker annotation(
        Placement(visible = true, transformation(origin = {36, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(ramp.y, hpm4.W_in) annotation(
        Line(points = {{-54, 6}, {-34, 6}, {-34, 4}, {-22, 4}}, color = {0, 0, 127}));
      connect(hpm4.Tourque_out, signalChecker.Torque_In) annotation(
        Line(points = {{2, 12}, {10, 12}, {10, 0}, {24, 0}}, color = {0, 0, 127}));
    end tt;
  end Hydro;
  annotation(
    uses(Modelica(version = "4.0.0"), OpenHPL(version = "2.0.1")));
end TestPackage;
