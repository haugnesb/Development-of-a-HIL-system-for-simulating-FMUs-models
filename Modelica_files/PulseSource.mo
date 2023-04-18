within ;
model PulseSource
  parameter Real PulsStartTime=0 "start time off the puls";
  parameter Real PulsPeriod=60 "Puls Duration In Seconds";
  parameter Real PulsWidth=50 "Puls on time in %";
  Modelica.Blocks.Interfaces.RealOutput StartStop
    annotation (Placement(transformation(extent={{100,50},{120,70}})));
  Modelica.Blocks.Sources.Pulse pulse(
    width=PulsWidth,
    period=PulsPeriod,
    offset=0,
    startTime=PulsStartTime)
    annotation (Placement(transformation(extent={{-72,50},{-52,70}})));
equation
  connect(StartStop, pulse.y) annotation (Line(points={{110,60},{-51,60}},
                     color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    uses(Modelica(version="3.2.3")),
    experiment(StopTime=100));
end PulseSource;
