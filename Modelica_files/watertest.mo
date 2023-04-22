within ;
package watertest
  model TestReservoir
    OpenHPL.Waterway.Reservoir tail(h_0=5) annotation (Placement(transformation(
          origin={-4,28},
          extent={{-10,10},{10,-10}},
          rotation=180)));
    inner OpenHPL.Data data annotation (Placement(transformation(
          origin={-90,90},
          extent={{-10,-10},{10,10}})));
    OpenHPL.Waterway.Pipe intake(H=0, Vdot(fixed=true))    annotation (Placement(transformation(extent={{-40,18},
              {-20,38}})));
    OpenHPL.Waterway.Reservoir reservoir(
      useLevel=true,
      useInflow=false,
      h_0=48)                                    annotation (Placement(transformation(
          origin={-68,28},
          extent={{-10,-10},{10,10}})));
    OpenHPL.Waterway.RunOff_zones runOff_zones
      annotation (Placement(transformation(extent={{-90,-44},{-70,-24}})));
    OpenHPL.Waterway.VolumeFlowSource volumeFlowSource
      annotation (Placement(transformation(extent={{-8,-36},{12,-16}})));
  equation
    connect(intake.o, tail.o)
      annotation (Line(points={{-20,28},{-14,28}}, color={0,128,255}));
    connect(intake.i, reservoir.o)
      annotation (Line(points={{-40,28},{-58,28}}, color={0,128,255}));
    connect(reservoir.level, runOff_zones.Vdot_runoff) annotation (Line(points=
            {{-80,34},{-76,34},{-76,-34},{-70,-34}}, color={0,0,127}));
    annotation (
      Icon(coordinateSystem(preserveAspectRatio=false)),
      Diagram(coordinateSystem(preserveAspectRatio=false)),
      experiment(StopTime=2000, __Dymola_NumberOfIntervals=5000));
  end TestReservoir;

  model VolumeFlowSource "Example demonstrating the use of VolumeFlowSource"
    extends Modelica.Icons.Example;
    OpenHPL.Waterway.Reservoir tail1 annotation (Placement(transformation(extent={{58,76},
              {38,96}})));
    inner OpenHPL.Data data annotation (Placement(transformation(extent={{-100,80},
              {-80,100}})));
    OpenHPL.Waterway.VolumeFlowSource volumeFlowConstant annotation (Placement(transformation(extent={{-52,76},
              {-32,96}})));
    OpenHPL.Waterway.Pipe pipe1(H=0)
      annotation (Placement(transformation(extent={{-12,76},{8,96}})));
    OpenHPL.Waterway.Pipe pipe2(H=0)
      annotation (Placement(transformation(extent={{-14,36},{6,56}})));
    OpenHPL.Waterway.Reservoir tail2 annotation (Placement(transformation(extent={{58,36},
              {38,56}})));
    OpenHPL.Waterway.VolumeFlowSource volumeFlowInput(useInput=true, useFilter=false) annotation (Placement(transformation(extent={{-50,36},
              {-30,56}})));
    Modelica.Blocks.Sources.Sine sine(f=0.01) annotation (Placement(transformation(extent={{-82,36},
              {-62,56}})));
    Modelica.Blocks.Sources.CombiTimeTable logdata(table=[1,0.256342739; 2,0.245221436; 3,0.266113698; 4,0.249561667; 5,0.525063097; 6,0.479064316; 7,0.494991362; 8,0.489708632; 9,0.50391084; 10,0.492354929; 11,0.509279788; 12,0.495274216; 13,0.493877858; 14,0.520679474; 15,0.499932915; 16,0.494227827; 17,0.472558141; 18,0.441845328; 19,0.398698032; 20,0.369865984; 21,0.341512531; 22,0.317958236; 23,0.300121665; 24,0.283421665; 25,0.271103382; 26,0.29000932; 27,0.276437521; 28,0.279719085; 29,0.273819894;
          30,0.530646265; 31,0.494690746; 32,0.668753743; 33,0.748319328; 34,1.156479597; 35,1.622769237; 36,1.455329537; 37,1.440503478; 38,1.388660312; 39,1.321571827; 40,1.428969026; 41,1.335716724; 42,1.240077496; 43,1.147016048; 44,1.046641827; 45,0.957466781; 46,0.877434254; 47,0.760209978; 48,0.685476542; 49,0.611937046; 50,0.5434497; 51,0.486470848; 52,0.437200874; 53,0.387043357; 54,0.346913666; 55,0.321531206; 56,0.3025949; 57,0.289946347; 58,0.76049149; 59,1.301532745; 60,1.66047287; 61,
          1.468578339; 62,1.445258141; 63,1.381029367; 64,1.308333635; 65,1.429936647; 66,1.324193954; 67,1.215524554; 68,1.132795691; 69,1.039966226; 70,0.957139969; 71,0.881121516; 72,0.764806509; 73,0.679720461; 74,1.153712869; 75,1.617045045; 76,1.342065096; 77,1.21108222; 78,1.097126365; 79,0.956687152; 80,0.846820831; 81,0.726776063; 82,0.637088716; 83,0.566960931; 84,0; 85,0.081746899; 86,0.287258357; 87,0.440648884; 88,0.623197138; 89,0.745818675; 90,0.877141893; 91,1.14376235; 92,1.091307402;
          93,1.166081309; 94,1.204966307; 95,1.155335188; 96,1.090149403; 97,1.023901701; 98,0.955312788; 99,0.895717919; 100,0.781589091; 101,0.711237729; 102,0.642216742; 103,0.592425168; 104,0.53075856; 105,0.470919639; 106,0.424907953; 107,0.379154414; 108,0.341206133; 109,0.313439339; 110,0.291419089; 111,0.282484144], extrapolation=Modelica.Blocks.Types.Extrapolation.HoldLastPoint)
                                                                       annotation (Placement(transformation(extent={{-82,-4},
              {-62,16}})));
    OpenHPL.Waterway.Pipe pipe3(H=0)
      annotation (Placement(transformation(extent={{-12,-4},{8,16}})));
    OpenHPL.Waterway.Reservoir tail3
      annotation (Placement(transformation(extent={{58,-4},{38,16}})));
    OpenHPL.Waterway.VolumeFlowSource volumeFlowFiltered(useInput=true,
        useFilter=true)
      annotation (Placement(transformation(extent={{-50,-4},{-30,16}})));
    OpenHPL.Waterway.Reservoir tail4 annotation (Placement(transformation(extent={{60,-28},
              {40,-8}})));
    OpenHPL.Waterway.Pipe pipe4(H=0)
      annotation (Placement(transformation(extent={{-10,-28},{10,-8}})));
    OpenHPL.Waterway.Reservoir reservoir(useInflow=false, h_0=48)
                                                 annotation (Placement(transformation(
          origin={-40,-18},
          extent={{-10,-10},{10,10}})));
    OpenHPL.Waterway.Reservoir tail5 annotation (Placement(transformation(extent={{60,-58},
              {40,-38}})));
    OpenHPL.Waterway.Pipe pipe5(H=0)
      annotation (Placement(transformation(extent={{-10,-58},{10,-38}})));
    OpenHPL.Waterway.Pipe pipe6(H=0)
      annotation (Placement(transformation(extent={{-94,-92},{-74,-72}})));
    OpenHPL.Waterway.Reservoir tail6 annotation (Placement(transformation(extent={{76,-92},
              {56,-72}})));
    OpenHPL.Waterway.VolumeFlowSource volumeFlowInput1(useInput=false,
        useFilter=false)                                                              annotation (Placement(transformation(extent={{-148,
              -92},{-128,-72}})));
    OpenHPL.Waterway.VolumeFlowSource volumeFlowInput2(useInput=true, useFilter=
         false)                                                                       annotation (Placement(transformation(extent={{-40,-58},
              {-20,-38}})));
    Modelica.Blocks.Sources.Ramp ramp1(
      height=-0.5,
      duration=1,
      offset=-1,
      startTime=100)
      annotation (Placement(transformation(extent={{-110,-58},{-90,-38}})));
    OpenHPL.Waterway.PenstockKP penstockKP
      annotation (Placement(transformation(extent={{22,-92},{42,-72}})));
  equation
    connect(pipe1.o, tail1.o) annotation (Line(points={{8,86},{38,86}},  color={0,128,255}));
    connect(volumeFlowConstant.o, pipe1.i) annotation (Line(points={{-32,86},{
            -12,86}},                                                                   color={0,128,255}));
    connect(volumeFlowInput.o, pipe2.i) annotation (Line(points={{-30,46},{-14,
            46}},                                                                  color={0,128,255}));
    connect(pipe2.o, tail2.o) annotation (Line(points={{6,46},{38,46}},color={0,128,255}));
    connect(sine.y, volumeFlowInput.outFlow) annotation (Line(points={{-61,46},
            {-52,46}},                                                                  color={0,0,127}));
    connect(volumeFlowFiltered.o, pipe3.i) annotation (Line(points={{-30,6},{
            -12,6}},                                                                      color={0,128,255}));
    connect(pipe3.o,tail3. o) annotation (Line(points={{8,6},{38,6}},  color={0,128,255}));
    connect(volumeFlowFiltered.outFlow, logdata.y[1]) annotation (Line(points={{-52,6},
            {-61,6}},                                                                                color={0,0,127}));
    connect(pipe4.o,tail4. o) annotation (Line(points={{10,-18},{40,-18}},
                                                                         color={0,128,255}));
    connect(pipe4.i, reservoir.o)
      annotation (Line(points={{-10,-18},{-30,-18}}, color={0,128,255}));
    connect(pipe5.o,tail5. o) annotation (Line(points={{10,-48},{40,-48}},
                                                                         color={0,128,255}));
    connect(volumeFlowInput1.o, pipe6.i)
      annotation (Line(points={{-128,-82},{-94,-82}}, color={0,128,255}));
    connect(pipe5.i, volumeFlowInput2.o)
      annotation (Line(points={{-10,-48},{-20,-48}}, color={0,128,255}));
    connect(volumeFlowInput2.outFlow, ramp1.y)
      annotation (Line(points={{-42,-48},{-89,-48}}, color={0,0,127}));
    connect(tail6.o, penstockKP.o)
      annotation (Line(points={{56,-82},{42,-82}}, color={0,128,255}));
    connect(pipe6.o, penstockKP.i)
      annotation (Line(points={{-74,-82},{22,-82}}, color={0,128,255}));
    annotation (experiment(StopTime=1000),
      Diagram(coordinateSystem(extent={{-200,-100},{140,100}})),
      Icon(coordinateSystem(extent={{-200,-100},{140,100}})));
  end VolumeFlowSource;

  model Simple "Model of a hydropower system with a simple turbine turbine"
    extends Modelica.Icons.Example;
    OpenHPL.Waterway.Reservoir reservoir(useInflow=false,
                                         h_0=48) annotation (Placement(transformation(
          origin={-90,30},
          extent={{-10,-10},{10,10}})));
    Modelica.Blocks.Sources.Ramp control(
      duration=30, height = -0.04615, offset = 0.7493,
      startTime=500) annotation (
      Placement(transformation(origin={-10,70}, extent = {{-10, -10}, {10, 10}})));
    OpenHPL.Waterway.Pipe intake(H=23, Vdot(fixed = true)) annotation (Placement(transformation(extent={{-70,20},{-50,40}})));
    OpenHPL.Waterway.Pipe discharge(H=0.5, L=600) annotation (Placement(transformation(extent={{50,-10},{70,10}})));
    OpenHPL.Waterway.Reservoir tail(h_0=5) annotation (Placement(transformation(
          origin={90,0},
          extent={{-10,10},{10,-10}},
          rotation=180)));
    replaceable OpenHPL.Waterway.Pipe penstock(
      D_i=3,
      D_o=3,
      H=428.5,
      L=600,
      vertical=true) constrainedby OpenHPL.Interfaces.TwoContact
                                                         annotation (Placement(transformation(origin={0,30}, extent={{-10,-10},{10,10}})));
    OpenHPL.Waterway.SurgeTank surgeTank(h_0=69.9) annotation (Placement(transformation(
          origin={-30,30},
          extent={{-10,-10},{10,10}})));
    OpenHPL.ElectroMech.Turbines.Turbine turbine(
      C_v=3.7,
      ConstEfficiency=false,
      enable_nomSpeed=true,
      enable_f=false) annotation (Placement(transformation(origin={30,10},
            extent={{-10,-10},{10,10}})));
    inner OpenHPL.Data data annotation (Placement(transformation(
          origin={-90,90},
          extent={{-10,-10},{10,10}})));
  equation
    connect(turbine.o, discharge.i) annotation (Line(points={{40,10},{44,10},{44,0},{50,0}}, color={28,108,200}));
    connect(control.y, turbine.u_t) annotation (Line(points={{1,70},{16,70},{16,40},{22,40},{22,22}},
                                                                                      color={0,0,127}));
    connect(penstock.o, turbine.i) annotation (Line(points={{10,30},{14.95,30},{14.95,10},{20,10}}, color={28,108,200}));
    connect(reservoir.o, intake.i) annotation (
      Line(points={{-80,30},{-70,30}}, color = {28, 108, 200}));
    connect(intake.o, surgeTank.i) annotation (
      Line(points={{-50,30},{-40,30}}, color = {28, 108, 200}));
    connect(surgeTank.o, penstock.i) annotation (Line(points={{-20,30},{-10,30}}, color={28,108,200}));
    connect(discharge.o, tail.o) annotation (Line(points={{70,0},{80,0}}, color={28,108,200}));
    annotation (experiment(StopTime=2000, __Dymola_NumberOfIntervals=5000));
  end Simple;

  model Simple1 "Model of a hydropower system with a simple turbine turbine"
    extends Modelica.Icons.Example;
    Modelica.Blocks.Sources.Ramp control(
      duration=30, height = -0.04615, offset = 0.7493,
      startTime=500) annotation (
      Placement(transformation(origin={-10,70}, extent = {{-10, -10}, {10, 10}})));
    OpenHPL.Waterway.Pipe intake(H=23, Vdot(fixed = true)) annotation (Placement(transformation(extent={{-70,20},{-50,40}})));
    OpenHPL.Waterway.Pipe discharge(H=0.5, L=600) annotation (Placement(transformation(extent={{50,-10},{70,10}})));
    OpenHPL.Waterway.Reservoir tail(h_0=5) annotation (Placement(transformation(
          origin={90,0},
          extent={{-10,10},{10,-10}},
          rotation=180)));
    replaceable OpenHPL.Waterway.Pipe penstock(
      D_i=3,
      D_o=3,
      H=428.5,
      L=600,
      vertical=true) constrainedby OpenHPL.Interfaces.TwoContact
                                                         annotation (Placement(transformation(origin={0,30}, extent={{-10,-10},{10,10}})));
    OpenHPL.Waterway.SurgeTank surgeTank(h_0=69.9) annotation (Placement(transformation(
          origin={-30,30},
          extent={{-10,-10},{10,10}})));
    OpenHPL.ElectroMech.Turbines.Turbine turbine(
      C_v=3.7,
      ConstEfficiency=false,
      enable_nomSpeed=true,
      enable_f=false) annotation (Placement(transformation(origin={30,10},
            extent={{-10,-10},{10,10}})));
    inner OpenHPL.Data data annotation (Placement(transformation(
          origin={-90,90},
          extent={{-10,-10},{10,10}})));
    OpenHPL.Waterway.VolumeFlowSource volumeFlowConstant(Vdot_0=data.Vdot_0)
                                                         annotation (Placement(transformation(extent={{-98,20},
              {-78,40}})));
  equation
    connect(turbine.o, discharge.i) annotation (Line(points={{40,10},{44,10},{44,0},{50,0}}, color={28,108,200}));
    connect(control.y, turbine.u_t) annotation (Line(points={{1,70},{16,70},{16,40},{22,40},{22,22}},
                                                                                      color={0,0,127}));
    connect(penstock.o, turbine.i) annotation (Line(points={{10,30},{14.95,30},{14.95,10},{20,10}}, color={28,108,200}));
    connect(intake.o, surgeTank.i) annotation (
      Line(points={{-50,30},{-40,30}}, color = {28, 108, 200}));
    connect(surgeTank.o, penstock.i) annotation (Line(points={{-20,30},{-10,30}}, color={28,108,200}));
    connect(discharge.o, tail.o) annotation (Line(points={{70,0},{80,0}}, color={28,108,200}));
    connect(intake.i, volumeFlowConstant.o)
      annotation (Line(points={{-70,30},{-78,30}}, color={0,128,255}));
    annotation (experiment(StopTime=2000, __Dymola_NumberOfIntervals=5000));
  end Simple1;

  model Simple2 "Model of a hydropower system with a simple turbine turbine"
    extends Modelica.Icons.Example;
    Modelica.Blocks.Sources.Ramp control(
      duration=30, height = -0.04615, offset = 0.7493,
      startTime=500) annotation (
      Placement(transformation(origin={-10,70}, extent = {{-10, -10}, {10, 10}})));
    OpenHPL.Waterway.Pipe intake(H=23, Vdot(fixed = true)) annotation (Placement(transformation(extent={{-70,20},
              {-50,40}})));
    OpenHPL.Waterway.Pipe discharge(H=0.5, L=600) annotation (Placement(transformation(extent={{50,-10},{70,10}})));
    OpenHPL.Waterway.Reservoir tail(h_0=5) annotation (Placement(transformation(
          origin={90,0},
          extent={{-10,10},{10,-10}},
          rotation=180)));
    replaceable OpenHPL.Waterway.Pipe penstock(
      D_i=3,
      D_o=3,
      H=428.5,
      L=600,
      vertical=true) constrainedby OpenHPL.Interfaces.TwoContact
                                                         annotation (Placement(transformation(origin={0,30}, extent={{-10,-10},{10,10}})));
    OpenHPL.Waterway.SurgeTank surgeTank(h_0=69.9) annotation (Placement(transformation(
          origin={-30,30},
          extent={{-10,-10},{10,10}})));
    OpenHPL.ElectroMech.Turbines.Turbine turbine(
      C_v=3.7,
      ConstEfficiency=false,
      enable_nomSpeed=true,
      enable_f=false) annotation (Placement(transformation(origin={30,10},
            extent={{-10,-10},{10,10}})));
    inner OpenHPL.Data data annotation (Placement(transformation(
          origin={-90,90},
          extent={{-10,-10},{10,10}})));
    OpenHPL.Waterway.VolumeFlowSource volumeFlowConstant(Vdot_0=data.Vdot_0,
        useInput=true)                                   annotation (Placement(transformation(extent={{-98,20},
              {-78,40}})));
    Modelica.Blocks.Sources.Ramp ramp(
      height=-10,
      duration=1,
      offset=data.Vdot_0,
      startTime=100)
      annotation (Placement(transformation(extent={{-84,-46},{-64,-26}})));
  equation
    connect(turbine.o, discharge.i) annotation (Line(points={{40,10},{44,10},{44,0},{50,0}}, color={28,108,200}));
    connect(control.y, turbine.u_t) annotation (Line(points={{1,70},{16,70},{16,40},{22,40},{22,22}},
                                                                                      color={0,0,127}));
    connect(penstock.o, turbine.i) annotation (Line(points={{10,30},{14.95,30},{14.95,10},{20,10}}, color={28,108,200}));
    connect(intake.o, surgeTank.i) annotation (
      Line(points={{-50,30},{-40,30}}, color = {28, 108, 200}));
    connect(surgeTank.o, penstock.i) annotation (Line(points={{-20,30},{-10,30}}, color={28,108,200}));
    connect(discharge.o, tail.o) annotation (Line(points={{70,0},{80,0}}, color={28,108,200}));
    connect(intake.i, volumeFlowConstant.o)
      annotation (Line(points={{-70,30},{-78,30}}, color={0,128,255}));
    connect(ramp.y, volumeFlowConstant.outFlow) annotation (Line(points={{-63,
            -36},{-82,-36},{-82,12},{-100,12},{-100,30}}, color={0,0,127}));
    annotation (experiment(StopTime=2000, __Dymola_NumberOfIntervals=5000));
  end Simple2;

  model Detailed "Model of a hydropower system with a simple turbine turbine"
    extends Modelica.Icons.Example;
    OpenHPL.Waterway.Reservoir reservoir(useInflow=false,
                                         h_0=48) annotation (Placement(transformation(
          origin={-90,30},
          extent={{-10,-10},{10,10}})));
    Modelica.Blocks.Sources.Ramp control(
      duration=30, height = -0.04615, offset = 0.7493,
      startTime=500) annotation (
      Placement(transformation(origin={-10,70}, extent = {{-10, -10}, {10, 10}})));
    OpenHPL.Waterway.Pipe intake(H=23, Vdot(fixed = true)) annotation (Placement(transformation(extent={{-74,20},
              {-54,40}})));
    OpenHPL.Waterway.Pipe discharge(H=0.5, L=600) annotation (Placement(transformation(extent={{50,-10},{70,10}})));
    OpenHPL.Waterway.Reservoir tail(h_0=5) annotation (Placement(transformation(
          origin={90,0},
          extent={{-10,10},{10,-10}},
          rotation=180)));
    OpenHPL.Waterway.SurgeTank surgeTank(h_0=69.9) annotation (Placement(transformation(
          origin={-30,30},
          extent={{-10,-10},{10,10}})));
    OpenHPL.ElectroMech.Turbines.Turbine turbine(
      C_v=3.7,
      ConstEfficiency=false,
      enable_nomSpeed=true,
      enable_f=false) annotation (Placement(transformation(origin={30,10},
            extent={{-10,-10},{10,10}})));
    inner OpenHPL.Data data annotation (Placement(transformation(
          origin={-90,90},
          extent={{-10,-10},{10,10}})));
    OpenHPL.Waterway.PenstockKP penstockKP
      annotation (Placement(transformation(extent={{-12,18},{8,38}})));
  equation
    connect(turbine.o, discharge.i) annotation (Line(points={{40,10},{44,10},{44,0},{50,0}}, color={28,108,200}));
    connect(control.y, turbine.u_t) annotation (Line(points={{1,70},{16,70},{16,40},{22,40},{22,22}},
                                                                                      color={0,0,127}));
    connect(reservoir.o, intake.i) annotation (
      Line(points={{-80,30},{-74,30}}, color = {28, 108, 200}));
    connect(intake.o, surgeTank.i) annotation (
      Line(points={{-54,30},{-40,30}}, color = {28, 108, 200}));
    connect(discharge.o, tail.o) annotation (Line(points={{70,0},{80,0}}, color={28,108,200}));
    connect(surgeTank.o, penstockKP.i) annotation (Line(points={{-20,30},{-16,
            30},{-16,28},{-12,28}}, color={0,128,255}));
    connect(turbine.i, penstockKP.o) annotation (Line(points={{20,10},{14,10},{
            14,28},{8,28}}, color={0,128,255}));
    annotation (experiment(StopTime=2000, __Dymola_NumberOfIntervals=50000));
  end Detailed;

  model Detailed1 "Model of a hydropower system with a simple turbine turbine"
    extends Modelica.Icons.Example;
    Modelica.Blocks.Sources.Ramp control(
      duration=30,
      height=-0.04615,
      offset=0.7493,
      startTime=500) annotation (
      Placement(transformation(origin={-10,70}, extent = {{-10, -10}, {10, 10}})));
    OpenHPL.Waterway.Pipe intake(H=23, Vdot(fixed = true)) annotation (Placement(transformation(extent={{-74,18},
              {-54,38}})));
    OpenHPL.Waterway.Pipe discharge(H=0.5, L=600) annotation (Placement(transformation(extent={{50,-10},{70,10}})));
    OpenHPL.Waterway.Reservoir tail(h_0=5) annotation (Placement(transformation(
          origin={90,0},
          extent={{-10,10},{10,-10}},
          rotation=180)));
    OpenHPL.ElectroMech.Turbines.Turbine turbine(
      C_v=3.7,
      ConstEfficiency=false,
      enable_nomSpeed=true,
      enable_w=false,
      enable_f=false,
      enable_P_out=true) annotation (Placement(transformation(origin={30,10},
            extent={{-10,-10},{10,10}})));
    inner OpenHPL.Data data annotation (Placement(transformation(
          origin={-90,90},
          extent={{-10,-10},{10,10}})));
    OpenHPL.Waterway.PenstockKP penstockKP
      annotation (Placement(transformation(extent={{-12,18},{8,38}})));
    OpenHPL.ElectroMech.Generators.SimpleGen simpleGen(enable_w_in=false)
      annotation (Placement(transformation(extent={{26,68},{46,88}})));
    OpenHPL.Waterway.SurgeTank surgeTank(h_0=69.9) annotation (Placement(transformation(
          origin={-30,30},
          extent={{-10,-10},{10,10}})));
    OpenHPL.Waterway.Reservoir reservoir(useInflow=false, h_0=48)
                                                 annotation (Placement(transformation(
          origin={-90,28},
          extent={{-10,-10},{10,10}})));
    Modelica.Blocks.Math.Gain loadLevel(k=1) annotation (Placement(transformation(extent={{76,76},
              {56,96}})));
  equation
    connect(turbine.o, discharge.i) annotation (Line(points={{40,10},{44,10},{44,0},{50,0}}, color={28,108,200}));
    connect(control.y, turbine.u_t) annotation (Line(points={{1,70},{16,70},{16,
            32},{22,32},{22,22}},                                                     color={0,0,127}));
    connect(discharge.o, tail.o) annotation (Line(points={{70,0},{80,0}}, color={28,108,200}));
    connect(turbine.i, penstockKP.o) annotation (Line(points={{20,10},{14,10},{
            14,28},{8,28}}, color={0,128,255}));
    connect(turbine.flange, simpleGen.flange) annotation (Line(points={{30,10},
            {32,10},{32,78},{36,78}}, color={0,0,0}));
    connect(intake.o, surgeTank.i)
      annotation (Line(points={{-54,28},{-48,28},{-48,30},{-40,30}},
                                                   color={0,128,255}));
    connect(penstockKP.i, surgeTank.o) annotation (Line(points={{-12,28},{-16,
            28},{-16,30},{-20,30}}, color={0,128,255}));
    connect(intake.i, reservoir.o) annotation (Line(points={{-74,28},{-80,28}},
                               color={0,128,255}));
    connect(simpleGen.Pload, loadLevel.y) annotation (Line(points={{36,90},{46,
            90},{46,86},{55,86}}, color={0,0,127}));
    connect(turbine.P_out, loadLevel.u) annotation (Line(points={{34,21},{54,21},
            {54,34},{82,34},{82,86},{78,86}}, color={0,0,127}));
    annotation (experiment(StopTime=2000, __Dymola_NumberOfIntervals=5000));
  end Detailed1;

  model DetailedGen
    "Hydropower system using KP scheme based penstock with generator"
    extends OpenHPL.Examples.SimpleGen(redeclare OpenHPL.Waterway.PenstockKP
        penstock, data(SteadyState=false));
    annotation (experiment(StopTime=1000));
  end DetailedGen;

  model uifeuifuef "Model of a hydropower system with a simple turbine turbine"
    extends Modelica.Icons.Example;
    Modelica.Blocks.Sources.Ramp control(
      duration=30,
      height=-0.04615,
      offset=0.7493,
      startTime=500) annotation (
      Placement(transformation(origin={-10,70}, extent = {{-10, -10}, {10, 10}})));
    OpenHPL.Waterway.Pipe intake(H=23, Vdot(fixed=true))   annotation (Placement(transformation(extent={{-70,20},
              {-50,40}})));
    OpenHPL.Waterway.Pipe discharge(H=0.5, L=600) annotation (Placement(transformation(extent={{50,-10},{70,10}})));
    OpenHPL.Waterway.Reservoir tail(h_0=5) annotation (Placement(transformation(
          origin={90,0},
          extent={{-10,10},{10,-10}},
          rotation=180)));
                OpenHPL.Waterway.PenstockKP
                                      penstock           annotation (Placement(transformation(origin={0,30}, extent={{-10,-10},{10,10}})));
    OpenHPL.Waterway.SurgeTank surgeTank(h_0=69.9) annotation (Placement(transformation(
          origin={-30,30},
          extent={{-10,-10},{10,10}})));
    OpenHPL.ElectroMech.Turbines.Turbine
                                 turbine(
      enable_nomSpeed=false,
      enable_P_out=true,
      C_v=3.7,
      ConstEfficiency=false,
      enable_f=false)
      annotation (Placement(transformation(
          origin={30,10},
          extent={{-10,-10},{10,10}})));
    inner OpenHPL.Data data(SteadyState=false)
                            annotation (Placement(transformation(
          origin={-90,90},
          extent={{-10,-10},{10,10}})));
    OpenHPL.ElectroMech.Generators.SimpleGen
                                     simpleGen annotation (Placement(transformation(extent={{20,50},{40,70}})));
    Modelica.Blocks.Math.Gain loadLevel(k=1) annotation (Placement(transformation(extent={{72,70},{52,90}})));
    OpenHPL.Waterway.VolumeFlowSource volumeFlowConstant(Vdot_0=data.Vdot_0)
                                                         annotation (Placement(transformation(extent={{-100,20},
              {-80,40}})));
  equation
    connect(turbine.o,discharge. i) annotation (Line(points={{40,10},{44,10},{44,0},{50,0}}, color={28,108,200}));
    connect(control.y,turbine. u_t) annotation (Line(points={{1,70},{16,70},{16,40},{22,40},{22,22}},
                                                                                      color={0,0,127}));
    connect(penstock.o,turbine. i) annotation (Line(points={{10,30},{14.95,30},{14.95,10},{20,10}}, color={28,108,200}));
    connect(intake.o,surgeTank. i) annotation (
      Line(points={{-50,30},{-40,30}}, color = {28, 108, 200}));
    connect(surgeTank.o,penstock. i) annotation (Line(points={{-20,30},{-10,30}}, color={28,108,200}));
    connect(discharge.o,tail. o) annotation (Line(points={{70,0},{80,0}}, color={28,108,200}));
    connect(turbine.P_out,loadLevel. u) annotation (Line(points={{34,21},{34,30},{86,30},{86,80},{74,80}}, color={0,0,127}));
    connect(loadLevel.y,simpleGen. Pload) annotation (Line(points={{51,80},{30,80},{30,72}}, color={0,0,127}));
    connect(simpleGen.flange,turbine. flange) annotation (Line(
        points={{30,60},{30,10}},
        color={0,0,0}));
    connect(intake.i, volumeFlowConstant.o)
      annotation (Line(points={{-70,30},{-80,30}}, color={0,128,255}));
    annotation (experiment(StopTime=2000, __Dymola_NumberOfIntervals=5000));
  end uifeuifuef;
  annotation (uses(Modelica(version="4.0.0"), OpenHPL(version="2.0.1")));
end watertest;
