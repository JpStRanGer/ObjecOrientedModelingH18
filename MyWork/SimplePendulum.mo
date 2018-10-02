within ;
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
