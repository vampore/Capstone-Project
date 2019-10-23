%% Declare Varibales
%> Initial source location
initialLoc = [0 0];
%> Next source location
newSource = [0 0];
%> Distance moved by each step
dist_move = 0.5;
%> Hold the correct source location
resultSource = [];
%> Hold good image source
resultIS=[];
%> Hold the good slope to verify walls
resultSlope = [];
%> Hold the intercept
resultIntercept = [];
%> How many stops have been confirmed
stops = 1;
%> Assumed source-microphone distance
srd = 0.5;
%> Hold the potential wall lines
potentialLines = [];
%> Hold the found lines by by slope and intercept
foundLines = [];
%> Variable to hold how the robot has moved
sourcePlot = [];
disp('Necessary variables have been declared');