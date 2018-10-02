within ;
package FM3217_2018
  package Tutorial1
    model SimplePendulum "Model of a simple pendulum"
      import SI = Modelica.SIunits;
      constant SI.Acceleration g = 9.81 "Gravitational Constant";
      parameter SI.Length L(min=0) = 1 "Length of the Pendelum";
      // Now com the variables
      SI.Angle Theta(start=0.1, fixed= true) "Angle of the pendelum";
      SI.AngularVelocity ThetaDot  "der(Angle of the pendelum)";
    equation
      ThetaDot = der(Theta);
      der(ThetaDot) = - g/L*sin(Theta);
    end SimplePendulum;
  end Tutorial1;

  package Tutorial2
    model Motor
      Modelica.Electrical.Analog.Basic.Resistor resistor(R=0.5)
        annotation (Placement(transformation(extent={{-48,44},{-28,64}})));
      Modelica.Electrical.Analog.Basic.Inductor inductor(L=0.05)
        annotation (Placement(transformation(extent={{-16,44},{4,64}})));
      Modelica.Electrical.Analog.Basic.EMF emf
        annotation (Placement(transformation(extent={{10,8},{30,28}})));
      Modelica.Electrical.Analog.Basic.Ground ground
        annotation (Placement(transformation(extent={{-60,-44},{-40,-24}})));
      Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage
        annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=-90,
            origin={-58,8})));
      Modelica.Mechanics.Rotational.Components.Inertia inertia(J=0.001)
        annotation (Placement(transformation(extent={{44,8},{64,28}})));
      Modelica.Blocks.Interfaces.RealInput u
        annotation (Placement(transformation(extent={{-154,-18},{-114,22}})));
      Modelica.Mechanics.Rotational.Interfaces.Flange_b flange
        "Right flange of shaft"
        annotation (Placement(transformation(extent={{94,-10},{114,10}})));
    equation
      connect(resistor.n, inductor.p)
        annotation (Line(points={{-28,54},{-16,54}}, color={0,0,255}));
      connect(signalVoltage.p, resistor.p) annotation (Line(points={{-58,18},{
              -58,54},{-48,54}}, color={0,0,255}));
      connect(inductor.n, emf.p)
        annotation (Line(points={{4,54},{20,54},{20,28}}, color={0,0,255}));
      connect(emf.flange, inertia.flange_a)
        annotation (Line(points={{30,18},{44,18}}, color={0,0,0}));
      connect(signalVoltage.n, emf.n) annotation (Line(points={{-58,-2},{-58,-2},
              {-58,-10},{-58,-10},{-58,-10},{20,-10},{20,8}}, color={0,0,255}));
      connect(ground.p, emf.n) annotation (Line(points={{-50,-24},{-50,-10},{20,
              -10},{20,8}}, color={0,0,255}));
      connect(signalVoltage.v, u) annotation (Line(points={{-65,8},{-92,8},{-92,
              2},{-134,2}}, color={0,0,127}));
      connect(inertia.flange_b, flange) annotation (Line(points={{64,18},{84,18},
              {84,0},{104,0}}, color={0,0,0}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Motor;

    model MotorDrive
      Motor motor annotation (Placement(transformation(extent={{-8,-8},{8,8}})));
      Modelica.Blocks.Math.Feedback feedback
        annotation (Placement(transformation(extent={{-76,-10},{-56,10}})));
      Modelica.Blocks.Sources.Step step(startTime=0.5)
        annotation (Placement(transformation(extent={{-106,-10},{-86,10}})));
      Modelica.Blocks.Continuous.PID PID
        annotation (Placement(transformation(extent={{-46,-10},{-26,10}})));
      Modelica.Mechanics.Rotational.Components.IdealGear idealGear(ratio=100)
        annotation (Placement(transformation(extent={{14,-10},{34,10}})));
      Modelica.Mechanics.Rotational.Components.Inertia inertia(J=5)
        annotation (Placement(transformation(extent={{38,-10},{58,10}})));
      Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor annotation (
         Placement(transformation(
            extent={{-9,-9},{9,9}},
            rotation=-90,
            origin={81,-29})));
    equation
      connect(motor.u, PID.y) annotation (Line(points={{-10.72,0.16},{-18,0.16},
              {-18,0},{-25,0}}, color={0,0,127}));
      connect(feedback.y, PID.u)
        annotation (Line(points={{-57,0},{-48,0}}, color={0,0,127}));
      connect(step.y, feedback.u1)
        annotation (Line(points={{-85,0},{-74,0}}, color={0,0,127}));
      connect(motor.flange, idealGear.flange_a)
        annotation (Line(points={{8.32,0},{14,0}}, color={0,0,0}));
      connect(idealGear.flange_b, inertia.flange_a)
        annotation (Line(points={{34,0},{38,0}}, color={0,0,0}));
      connect(inertia.flange_b, angleSensor.flange) annotation (Line(points={{
              58,0},{80,0},{80,-20},{81,-20}}, color={0,0,0}));
      connect(feedback.u2, angleSensor.phi) annotation (Line(points={{-66,-8},{
              -66,-58},{81,-58},{81,-38.9}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                -120,-100},{100,100}})),                             Diagram(
            coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},{
                100,100}})));
    end MotorDrive;
  end Tutorial2;

  package Tutorial3
    package Components
      model Motor
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=0.5)
          annotation (Placement(transformation(extent={{-48,20},{-28,40}})));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=0.05)
          annotation (Placement(transformation(extent={{-16,20},{4,40}})));
        Modelica.Electrical.Analog.Basic.EMF emf
          annotation (Placement(transformation(extent={{10,-10},{30,10}})));
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(extent={{-60,-60},{-40,-40}})));
        Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage
          annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=-90,
              origin={-58,0})));
        Modelica.Mechanics.Rotational.Components.Inertia inertia(J=0.001)
          annotation (Placement(transformation(extent={{44,-10},{64,10}})));
        Modelica.Blocks.Interfaces.RealInput u
          annotation (Placement(transformation(extent={{-134,-20},{-94,20}})));
        Modelica.Mechanics.Rotational.Interfaces.Flange_b flange
          "Right flange of shaft"
          annotation (Placement(transformation(extent={{94,-10},{114,10}})));
      equation
        connect(resistor.n, inductor.p)
          annotation (Line(points={{-28,30},{-16,30}}, color={0,0,255}));
        connect(signalVoltage.p, resistor.p) annotation (Line(points={{-58,10},
                {-58,30},{-48,30}},color={0,0,255}));
        connect(inductor.n, emf.p)
          annotation (Line(points={{4,30},{20,30},{20,10}}, color={0,0,255}));
        connect(emf.flange, inertia.flange_a)
          annotation (Line(points={{30,0},{44,0}},   color={0,0,0}));
        connect(signalVoltage.n, emf.n) annotation (Line(points={{-58,-10},{-58,
                -28},{20,-28},{20,-10}},                        color={0,0,255}));
        connect(ground.p, emf.n) annotation (Line(points={{-50,-40},{-50,-28},{
                20,-28},{20,-10}},
                              color={0,0,255}));
        connect(signalVoltage.v, u) annotation (Line(points={{-65,0},{-114,0}},
                              color={0,0,127}));
        connect(inertia.flange_b, flange) annotation (Line(points={{64,0},{104,
                0}},             color={0,0,0}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end Motor;

      model DCMachine


        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)    annotation (Placement(transformation(extent={{-48,44},{-28,64}})));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L)    annotation (Placement(transformation(extent={{-16,44},{4,64}})));
        Modelica.Electrical.Analog.Basic.EMF emf    annotation (Placement(transformation(extent={{10,8},{30,28}})));
        Modelica.Electrical.Analog.Basic.Ground ground    annotation (Placement(transformation(extent={{-64,-102},{-44,-82}})));
        Modelica.Mechanics.Rotational.Components.Inertia inertia(J=J)    annotation (Placement(transformation(extent={{44,8},{64,28}})));
        Modelica.Mechanics.Rotational.Interfaces.Flange_b flange      "Right flange of shaft"    annotation (Placement(transformation(extent={{94,-10},{114,10}})));
        Modelica.Electrical.Analog.Interfaces.PositivePin p    "Positive pin (potential p.v > n.v for positive voltage drop v)"    annotation (Placement(transformation(extent={{-132,58},{-90,100}}),
              iconTransformation(extent={{-132,58},{-90,100}})));
        Modelica.Electrical.Analog.Interfaces.NegativePin n annotation (Placement(
              transformation(extent={{-132,-100},{-90,-58}}), iconTransformation(
                extent={{-132,-100},{-90,-58}})));

        parameter Modelica.SIunits.Inductance L=0.1 "Inductance of the DC Machine";
        parameter Modelica.SIunits.Reactance R=0.5 "Resistance of the armature"    annotation (Dialog(group="Electrical"));
        parameter Modelica.SIunits.Inertia J=0.001 "Inertia of the DC Machine"    annotation (Dialog(tab="Mechanical", group="MechGroup"));

      equation
        connect(resistor.n, inductor.p)    annotation (Line(points={{-28,54},{-16,54}}, color={0,0,255}));
        connect(inductor.n, emf.p)    annotation (Line(points={{4,54},{20,54},{20,28}}, color={0,0,255}));
        connect(emf.flange, inertia.flange_a)    annotation (Line(points={{30,18},{44,18}}, color={0,0,0}));
        connect(inertia.flange_b, flange) annotation (Line(points={{64,18},{84,18},
                {84,0},{104,0}}, color={0,0,0}));
        connect(resistor.p, p) annotation (Line(points={{-48,54},{-74,54},{-74,79},{-111,
                79}}, color={0,0,255}));
        connect(emf.n, n)    annotation (Line(points={{20,8},{20,-79},{-111,-79}}, color={0,0,255}));
        connect(n, n) annotation (Line(points={{-111,-79},{-112,-79},{-112,-80},{-123,
                -80},{-111,-80},{-111,-79}}, color={0,0,255}));
        connect(ground.p, n) annotation (Line(points={{-54,-82},{-54,-79},{-111,-79}},
              color={0,0,255}));
        connect(p, p) annotation (Line(points={{-111,79},{-110.5,79},{-110.5,79},{-111,
                79}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                Bitmap(extent={{100,-68},{-100,74}}, fileName="modelica://FM3217_2018/../FM3217/Resources/Images/dc-motor.jpg")}),
                                                                       Diagram(
              coordinateSystem(preserveAspectRatio=false)),
          Documentation(info="<html>
<h4>THis is a simple model of a DC machine</h4>
<p><br><br><img src=\"modelica://FM3217_2018/../FM3217/Resources/Images/dc-motor.jpg\"/></p>
</html>"));
      end DCMachine;

      model Rload "Restistor load"
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R_load)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={0,2})));
        Modelica.Electrical.Analog.Interfaces.PositivePin p1
          "Positive pin (potential p.v > n.v for positive voltage drop v)"
          annotation (Placement(transformation(extent={{-10,90},{10,110}})));
        Modelica.Electrical.Analog.Interfaces.NegativePin n1
                      "Negative pin"
          annotation (Placement(transformation(extent={{-10,-112},{10,-92}})));
        parameter Modelica.SIunits.Resistance R_load=0.5
          "Ohmic value of the resistive load";
      equation
        connect(resistor.p, p1)
          annotation (Line(points={{0,12},{0,100}}, color={0,0,255}));
        connect(resistor.n, n1)
          annotation (Line(points={{0,-8},{0,-102}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end Rload;

      model RLload
        extends Rload;
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L_load)
          annotation (Placement(transformation(
              extent={{10,10},{-10,-10}},
              rotation=90,
              origin={-42,2})));
        parameter Modelica.SIunits.Inductance L_load=0.1 "Load inductance";
      equation
        connect(inductor.p, p1) annotation (Line(points={{-42,12},{-42,54},{0,
                54},{0,100}}, color={0,0,255}));
        connect(inductor.n, n1) annotation (Line(points={{-42,-8},{-42,-44},{0,
                -44},{0,-102}}, color={0,0,255}));
      end RLload;

      model RLCload
        extends RLload;
        Modelica.Electrical.Analog.Basic.Capacitor capacitor(C=C_load)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={32,2})));
        parameter Modelica.SIunits.Capacitance C_load=0.001
          "Capacitance of the load";
      equation
        connect(capacitor.p, p1) annotation (Line(points={{32,12},{32,12},{32,
                54},{32,54},{32,54},{0,54},{0,100}}, color={0,0,255}));
        connect(capacitor.n, n1) annotation (Line(points={{32,-8},{32,-8},{32,
                -44},{32,-44},{32,-44},{0,-44},{0,-102}}, color={0,0,255}));
      end RLCload;
    end Components;

    package Tests
      model MotorDrive
        Tutorial2.Motor motor
          annotation (Placement(transformation(extent={{-8,-8},{8,8}})));
        Modelica.Blocks.Math.Feedback feedback
          annotation (Placement(transformation(extent={{-76,-10},{-56,10}})));
        Modelica.Blocks.Sources.Step step(startTime=0.5)
          annotation (Placement(transformation(extent={{-106,-10},{-86,10}})));
        Modelica.Blocks.Continuous.PID PID
          annotation (Placement(transformation(extent={{-46,-10},{-26,10}})));
        Modelica.Mechanics.Rotational.Components.IdealGear idealGear(ratio=100)
          annotation (Placement(transformation(extent={{14,-10},{34,10}})));
        Modelica.Mechanics.Rotational.Components.Inertia inertia(J=5)
          annotation (Placement(transformation(extent={{38,-10},{58,10}})));
        Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor annotation (
           Placement(transformation(
              extent={{-9,-9},{9,9}},
              rotation=-90,
              origin={81,-29})));
      equation
        connect(motor.u, PID.y) annotation (Line(points={{-10.72,0.16},{-18,0.16},
                {-18,0},{-25,0}}, color={0,0,127}));
        connect(feedback.y, PID.u)
          annotation (Line(points={{-57,0},{-48,0}}, color={0,0,127}));
        connect(step.y, feedback.u1)
          annotation (Line(points={{-85,0},{-74,0}}, color={0,0,127}));
        connect(motor.flange, idealGear.flange_a)
          annotation (Line(points={{8.32,0},{14,0}}, color={0,0,0}));
        connect(idealGear.flange_b, inertia.flange_a)
          annotation (Line(points={{34,0},{38,0}}, color={0,0,0}));
        connect(inertia.flange_b, angleSensor.flange) annotation (Line(points={{
                58,0},{80,0},{80,-20},{81,-20}}, color={0,0,0}));
        connect(feedback.u2, angleSensor.phi) annotation (Line(points={{-66,-8},{
                -66,-58},{81,-58},{81,-38.9}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                  -120,-100},{100,100}})),                             Diagram(
              coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},{
                  100,100}})));
      end MotorDrive;

      model DCMachineTest
        Components.DCMachine DCMachine    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
        Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage    annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=-90,
              origin={-50,0})));
        Modelica.Blocks.Sources.Step step(startTime=1.5)    annotation (Placement(transformation(extent={{-94,-10},{-74,10}})));
      equation
        connect(signalVoltage.p, DCMachine.p) annotation (Line(points={{-50,10},{-50,20},
                {-20,20},{-20,7.9},{-11.1,7.9}}, color={0,0,255}));
        connect(signalVoltage.n, DCMachine.n) annotation (Line(points={{-50,-10},{-50,
                -10},{-50,-22},{-20,-22},{-20,-7.9},{-11.1,-7.9}}, color={0,0,255}));
        connect(signalVoltage.v, step.y) annotation (Line(points={{-57,1.22125e-015},{
                -66,1.22125e-015},{-66,0},{-73,0}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end DCMachineTest;

      model DCGeneratorTest
        Components.DCMachine DCMachine    annotation (Placement(transformation(extent={{-40,-10},
                  {-20,10}})));
        Modelica.Mechanics.Rotational.Components.Inertia inertia(J=5)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
        Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque(
            tau_constant=10)
          annotation (Placement(transformation(extent={{44,-10},{24,10}})));
        Components.RLCload RLCload
          annotation (Placement(transformation(extent={{-86,-8},{-66,12}})));
      equation
        connect(DCMachine.flange, inertia.flange_a)
          annotation (Line(points={{-19.6,0},{-10,0}}, color={0,0,0}));
        connect(inertia.flange_b, constantTorque.flange)
          annotation (Line(points={{10,0},{24,0}}, color={0,0,0}));
        connect(rLload.p1, DCMachine.p) annotation (Line(points={{-76,12},{-76,12},{-76,
                18},{-40,18},{-40,7.9},{-41.1,7.9}}, color={0,0,255}));
        connect(rLload.n1, DCMachine.n) annotation (Line(points={{-76,-8.2},{-76,-18},
                {-41.1,-18},{-41.1,-7.9}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end DCGeneratorTest;
    end Tests;
  end Tutorial3;
  annotation (uses(Modelica(version="3.2.2")));
end FM3217_2018;
