within ;
package Servo_Test_Rig_Models

  package VFD_test_controller
    model SimpleStartAndStopOfAInductionMotor
      parameter Real PulsStartTime=0 "start time off the puls";
      parameter Real PulsPeriod=60 "Puls Duration In Seconds";
      parameter Real PulsWidth=50 "Puls on time in %";
      parameter Real Voltage=5 "Motor speed";
      PulsSource pulsSource(
        PulsStartTime=PulsStartTime,
        PulsPeriod=PulsPeriod,
        PulsWidth=PulsWidth)
        annotation (Placement(transformation(extent={{-92,42},{-72,62}})));
      Modelica.Blocks.Interfaces.RealOutput MotorSpeed
        "Connector of Real output signal"
        annotation (Placement(transformation(extent={{100,10},{120,30}})));
      Modelica.Blocks.Math.Product product
        annotation (Placement(transformation(extent={{-26,10},{-6,30}})));
      Constant_Voltage constant_Voltage
        annotation (Placement(transformation(extent={{-92,4},{-72,24}})));
      Modelica.Blocks.Interfaces.RealOutput my1
                                     "Connector of Real output signal"
        annotation (Placement(transformation(extent={{102,50},{122,70}})));
    equation
      connect(MotorSpeed, product.y)
        annotation (Line(points={{110,20},{-5,20}}, color={0,0,127}));
      connect(MotorSpeed, MotorSpeed)
        annotation (Line(points={{110,20},{110,20}}, color={0,0,127}));
      connect(product.u1, pulsSource.my) annotation (Line(points={{-28,26},{-50,26},
              {-50,59.4},{-71,59.4}}, color={0,0,127}));
      connect(product.u2, constant_Voltage.constantVoltage)
        annotation (Line(points={{-28,14},{-70.6,14}}, color={0,0,127}));
      connect(pulsSource.my, my1) annotation (Line(points={{-71,59.4},{25.5,
              59.4},{25.5,60},{112,60}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)),
        experiment(StopTime=300, __Dymola_NumberOfIntervals=5000));
    end SimpleStartAndStopOfAInductionMotor;

    model PulsSource
      parameter Real PulsStartTime=0 "start time off the puls";
      parameter Real PulsPeriod=60 "Puls Duration In Seconds";
      parameter Real PulsWidth=50 "Puls on time in %";
      Modelica.Blocks.Interfaces.RealOutput my
                                     "Connector of Real output signal"
        annotation (Placement(transformation(extent={{100,64},{120,84}})));
      Modelica.Blocks.Sources.Pulse pulse(
        width=PulsWidth,
        period=PulsPeriod,
        offset=0,
        startTime=PulsStartTime)
        annotation (Placement(transformation(extent={{52,64},{72,84}})));
    equation
      connect(my, pulse.y)
        annotation (Line(points={{110,74},{73,74}}, color={0,0,127}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(StopTime=300, __Dymola_NumberOfIntervals=5000));
    end PulsSource;

    model Constant_Voltage
      Modelica.Blocks.Sources.Constant const(k=Voltage)
        annotation (Placement(transformation(extent={{-84,-10},{-64,10}})));
      Modelica.Blocks.Interfaces.RealOutput constantVoltage
        "Connector of Real output signal"
        annotation (Placement(transformation(extent={{104,-10},{124,10}})));
    equation
      connect(const.y, constantVoltage)
        annotation (Line(points={{-63,0},{114,0}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Constant_Voltage;

    model test_datatype
      Modelica.Blocks.Math.RealToInteger realToInteger
        annotation (Placement(transformation(extent={{0,-10},{20,10}})));
      Modelica.Blocks.Interfaces.IntegerOutput StartStop
        "Connector of Integer output signal"
        annotation (Placement(transformation(extent={{104,-10},{124,10}})));
      Modelica.Blocks.Sources.Pulse pulse(period=60)
        annotation (Placement(transformation(extent={{-66,-16},{-46,4}})));
      Modelica.Blocks.Sources.Constant const(k=5)
        annotation (Placement(transformation(extent={{-64,-72},{-44,-52}})));
      Modelica.Blocks.Math.Product product
        annotation (Placement(transformation(extent={{-18,-64},{2,-44}})));
      Modelica.Blocks.Interfaces.RealOutput MotorVoltage
        "Connector of Real output signal"
        annotation (Placement(transformation(extent={{120,-64},{140,-44}})));
    equation
      connect(realToInteger.y, StartStop)
        annotation (Line(points={{21,0},{114,0}}, color={255,127,0}));
      connect(pulse.y, product.u1) annotation (Line(points={{-45,-6},{-32,-6},{
              -32,-48},{-20,-48}}, color={0,0,127}));
      connect(const.y, product.u2) annotation (Line(points={{-43,-62},{-32,-62},
              {-32,-60},{-20,-60}}, color={0,0,127}));
      connect(realToInteger.u, pulse.y) annotation (Line(points={{-2,0},{-24,0},
              {-24,-6},{-45,-6}}, color={0,0,127}));
      connect(product.y, MotorVoltage)
        annotation (Line(points={{3,-54},{130,-54}}, color={0,0,127}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(StopTime=300));
    end test_datatype;
  end VFD_test_controller;
  annotation (uses(Modelica(version="4.0.0")),
    version="1",
    conversion(noneFromVersion=""));
end Servo_Test_Rig_Models;
