%% Bottle Rocket Launcher
% Eugenia Kritsuk, Ryan Real, Madeleine Drefke

% This is a script that finds the trajectory of a bottle after launch. 
% For the sake of simplity, we assumed that the force  of thrust would 
% remain constant while the butane evaporated once in contact with the 
% water.

function [totalDist, maxY, YwallX, toWall] = totalTraj(massAdd, qBot, wallX)

%% inputs
% massAdd is the total mass added to the bottle (the mass of the water
% added plus the mass of the butane added) measured in kg
% qBot is the angle of the bottle link (counterclockwise  from horizotal),
%measured in radians. This is also the launch angle of the bottle.
% wallX is the horizontal distance to the wall in meters

%% outputs
% totalDist is the horizontal distance traveled by the bottle once it is 
% launched measured in meters.
% maxY is the maximum height of the bottle reached during either portion of
% its flight measured in meters.
% pos is an array of concatenated postion vectors where each component of
% the position is measured in meters.
% toWall is a boolean value that is true if the projectile passes the wall

%% call the "thrust" function to find the trajectory of the bottle when 
% there is thrust. The final conditions of that piece of the trajectory are
% the initial conditions for the portion that does is not characterized by
% a thrust force.

[v0, q0, vx0, vy0, pos0] = thrust(massAdd, qBot);

%% initialize values
v = v0;
q = q0;
massBot = 0.0536; %kg
delT = 0.001; %s
Fd = 1/2*1.28*1.204*0.0014725*v^2;
acc = ([0, -9.81*massBot]-Fd.*[cos(q), sin(q)])./massBot;
vel = [vx0, vy0];
pos = pos0;

%% model movement
while pos(end,2)>=0
    Fd = 1/2*1.28*1.204*0.0014725.*((vel(end,1)).^2 ...
        + (vel(end,2)).^2)*[cos(q), sin(q)];
    acc = vertcat(acc, [0, -9.81*massBot]./massBot - Fd(end,:)./massBot);
    vel = vertcat(vel, vel(end,:) + acc(end,:).*delT);
    pos = vertcat(pos, pos(end,:) + vel(end,:).*delT);
    q = vertcat(q, atan(vel(end, 2)/vel(end,1)));
end

%% Find the maximum height and horizontal distance of the bottle.
maxY = max(pos(:,2));
totalDist = pos(end,1);

%% Find height of the projectile at the horizontal coordinate of the wall
YwallX = max(pos(:,2)); %intialize at a value that cannot be the final result
minDist = wallX;
toWall = false(1);
for i = 1:length(pos)
    distToWall = abs(pos(i,1)-wallX);
    if distToWall < minDist
        minDist = distToWall;
        YwallX = pos(i,2);
    end
    if i< length(pos) && pos(i+1,1) > wallX
        toWall = true(1);
    end
end

plot(pos(:,1), pos(:,2));
