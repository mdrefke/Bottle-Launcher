%% Bottle Rocket Launcher
%Eugenia Kritsuk, Ryan Real, Madeleine Drefke

% This is a script that finds the trajectory of a bottle after launch. 
% For the sake of simplity, we assumed that the force  of thrust would 
% remain constant while the butane evaporated once in contact with the 
% water. This script finds only the portion of trajectory experienced
% while thrust is still present.

function [velFinal, qProjFinal, velX, velY, pos] = thrust(massAdd, qBot)


%% Inputs
% massAdd is the total mass added to the bottle (the mass of the water
% added plus the mass of the butane added) measured in kg
% qBot is the angle of the bottle link (counterclockwise  from horizotal),
% measured in radians. This is also the launch angle of the bottle.

%% Outputs
% velFinal is the absolute value of the velocity vector at the end of the
% time when there is thrust from the bottle measured in m/s
% qProjFinal is the angle of the bottle trajectory at the end of the time
% when there is thrust from the bottle measured in radians.
% velX is the x-component of the velocity and the end of the time when
% there is thrust from the bottle measured in m/s
% velY is the y-component of the velocity and the end of the time when
% there is thrust from the bottle measured in m/s
% pos is an array of concatenated postion vectors where each component of
% the position is measured in meters.

%% define non-input constants
tDel = 0.001; %s 
t = 0:tDel:0.82; %0.82 is the time required for all of the butane to evaporate
area = 3.4636*10^(-4); %opening of bottle (m^2)
pBlock = 227000; %pa
qProj = qBot; %the projectile launch angle begins at the bottle link angle
thr = 2*pBlock*area; %N
massBot = 0.0536; %kg
mass = massAdd+massBot;
Fg = [0, mass*-9.81];

%% define starting position, velocity, forces, and accelerations
pos = [0,0];
vel = [0,0];
Ft = [thr*cos(qProj),thr*sin(qProj)];
F = Fg + Ft;
acc = (F)./mass;

%% determine forces, velocities, angles, accelerations, and positions throughout time during the projectile launch
for i = 1:length(t)-1
    Fg = vertcat(Fg, [0, -9.81*mass]);
    Fd = 1/2*1.28*1.204*0.0014725*((vel(i,1))^2 + (vel(i,2))^2);
    vel = vertcat(vel, vel(i,:) + tDel.*acc(i,:));
    if atan(vel(i+1,2)/vel(i+1,1)) < 0
        qProj = vertcat(qProj, -atan(vel(i+1,2)/vel(i+1,1)));
    else 
        qProj = vertcat(qProj, atan(vel(i+1,2)/vel(i+1,1)));
    end
    Ft = vertcat(Ft, thr.*[cos(qProj(i+1)), sin(qProj(i+1))]);
    F = vertcat(F, Ft(i+1,:) + Fg(i+1, :))-Fd*[cos(qProj(i+1)), sin(qProj(i+1))];
    acc = vertcat(acc, F(i+1,:)./mass);
    pos = vertcat(pos, pos(i,:) + tDel.*vel(i+1,:));
    mass = mass-massAdd/0.82*tDel;
    if pos(i,2) < 0
        break
    end
end

%% determine final velocities, angles, and positions once the butane has left the bottle
velFinal =  sqrt((vel(end,1))^2 + (vel(end,2))^2);
qProjFinal = qProj(end);
xFinal = pos(end, 1);
yFinal = pos(end, 2);
velY = vel(end,2);
velX = vel(end,1);