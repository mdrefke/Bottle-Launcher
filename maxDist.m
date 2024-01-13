%% Bottle Rocket Launcher
%Eugenia Kritsuk, Ryan Real, and Madeleine Drefke

%%This will determine the distance that may be covered by a butane rocket
%when the  mass of butane specified by an input is added to a bottle of
%mass specified by another input when the middle link length is adjusted.

function [distXMax, maxY, lenMid] = maxDist(massBot, massButane, wallX, wallY)

%% inputs
% massBot is the mass of the bottle including the water inside, but not
% including the mass of the butane, measured in kilograms
% massButane is the mass of liquid butane that is added to bottle, before
% any evaporates, measured in kilograms
% wallX is the horizontal distance to the wall in meters
% wallY is the vertical height of the wall in meters

%% outputs
% distX is the total horizontal distance traveled by the bottle after launch
% maxY is the maximum height the bottle will reach after launch
% lenMid is the length of the middle link that will allow the bottle to
% travel as far as possible and also make it over the wall

%% Find the distance covered when the middle link is adjusted between 24cm and 28cm

%initialize values
dist = zeros;
massAdd = massBot + massButane;
distXMax = 0;
lenMid = 0;

%iterate through lengths of the middle link
lens = 0.24:0.002:0.28;
for i = 1:length(lens)
    [dwellAngle] = links(lens(i));
    [distX, maxY, YwallX, toWall] = totalTraj(massAdd, dwellAngle, wallX);
    if YwallX >= wallY
        dist = vertcat(dist, distX);
    elseif YwallX<wallY && toWall 
        dist = vertcat(dist, wallX); %if it cant get over the wall, it will stop at the wall
    else
        dist = vertcat(dist, distX);
    end
    if dist(end) > distXMax
        distXMax = dist(end); %update max distance
        lenMid = lens(i); %store middle link length
    end
end

