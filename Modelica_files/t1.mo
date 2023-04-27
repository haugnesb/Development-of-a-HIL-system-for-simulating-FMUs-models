model t1 "Model of a hydropower system with a simple turbine turbine"
  extends Modelica.Icons.Example;
  import Modelica.Constants.pi;
Modelica.Blocks.Interfaces.RealInput u annotation(
    Placement(visible = true, transformation(origin = {-134, 28}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-102, 32}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
inner OpenHPL.Data data annotation(
    Placement(visible = true, transformation(origin = {-90, 88}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
OpenHPL.Waterway.Reservoir reservoir(h_0 = 48, useInflow = false, useLevel = true) annotation(
    Placement(visible = true, transformation(origin = {-80, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
OpenHPL.Waterway.Pipe intake(H(displayUnit = "Mm"), Vdot(fixed = true)) annotation(
    Placement(visible = true, transformation(origin = {4, 0}, extent = {{-70, 20}, {-50, 40}}, rotation = 0)));
OpenHPL.Waterway.Reservoir tail(h_0 = 5, useLevel = true) annotation(
    Placement(visible = true, transformation(origin = {-26, 30}, extent = {{-10, 10}, {10, -10}}, rotation = 180)));
Modelica.Blocks.Sources.Constant constant1(k = 5) annotation(
    Placement(visible = true, transformation(origin = {-20, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(reservoir.level, u) annotation(
    Line(points = {{-92, 36}, {-134, 36}, {-134, 28}}, color = {0, 0, 127}));
  connect(reservoir.o, intake.i) annotation(
    Line(points = {{-70, 30}, {-66, 30}}, color = {0, 128, 255}));
  connect(intake.o, tail.o) annotation(
    Line(points = {{-46, 30}, {-36, 30}}, color = {0, 128, 255}));
  connect(constant1.y, tail.level) annotation(
    Line(points = {{-9, -18}, {-4, -18}, {-4, 36}, {-14, 36}}, color = {0, 0, 127}));
  annotation(
    experiment(StopTime = 100, StartTime = 0, Tolerance = 1e-06, Interval = 0.2),
    Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}})));
end t1;
