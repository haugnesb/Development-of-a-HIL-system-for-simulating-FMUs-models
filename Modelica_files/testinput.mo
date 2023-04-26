model testinput
  Modelica.Blocks.Interfaces.RealInput u(start=3) annotation(
    Placement(visible = true, transformation(origin = {-46, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-54, 22}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Sources.Step step annotation(
    Placement(visible = true, transformation(origin = {-86, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  annotation(
    uses(Modelica(version = "4.0.0")));
end testinput;
