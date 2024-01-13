%% Determine Angle to be launched at

function [angle, energy] = launch(mProj, distHoleX, distHoleY, distTarX)

%% inputs
% "mProj" is the mass of the accelerometer container + the mass of the
% accelerometer itself in kg
% "distHoleX" is the horizontal distance between the launch point and the
% wall in m
% "distHoleY" is the height of the midpoint  of the hole in the wall in m
% "distTarX" is the horizontal distance from the launch point to the target
% in m

%% outputs
% "angle" is the launch angle up from horizontal measured in degrees
% "distPull" is how far to pull back the projectile before releasing 
% measured in centimeters
% "energy" is the work input associated with launching the projectile
% measured in J

%% declare non-input constants
g = 9.81; %m/s^2
h0 = 0.3048*3/4; %m(init height at projectile launch, 1ft when 90deg angle)
lengthCat = 0.381; %how far behind the line the projectile is released

%% Set up system of equations to find v and angle using kinematics eqns
syms v angle0;
eqn1 = v*cos(angle0)/g * (v*sin(angle0) + sqrt(v^2*(sin(angle0))^2 + ...
    2*g*h0)) == distTarX + lengthCat;
eqn2 = tan(angle0)*(distHoleX+lengthCat)-1/2*g*((distHoleX+lengthCat)^2/ ...
    v^2/(cos(angle0))^2)+h0 == distHoleY;
eqns = [eqn1, eqn2];
solnStruct = solve(eqns, [v, angle0]);
v0 = -1 * double(solnStruct.v);
angle1 = double(solnStruct.angle0);

%% return values of distance to be pulled and the angle at which to launch
angle = mod(angle1*180/pi, 360) - 180; %output1
energy = 1/2*mProj*v0^2; %output3