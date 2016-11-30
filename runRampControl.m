% Creates a piecewise trajectory, ramps up and down
% srats with a flat zero, ramps up, ramps down, then flat again
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
checkDependency('lcm');
javaaddpath('LCMTypes/acrobot_types.jar')

% Set how long each piece of trajectory should take,
beginningFlatTime = 10; %2nd argument for linspace
rampUpTime = 10;
midFlatTime = 10;
rampDownTime = 10;
endFlatTime = 1;
datapoints = 1e4; %3rd argument for linspace

% Set shape of ramps
upMin = 0; % where ramp up starts from
upMax = .2; % maximum value ramp goes up to
downMax = 0; % where ramp down starts from
downMin = -.2; %negative value ramp goes down to


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Robot
p = AcrobotPlantSmooth();

% Frame to publish input commands to
frOut = LCMCoordinateFrameWCoder('acrobot_u',1,'u',AcrobotInputCoder);

%calculate transition times between pieces
flatToUp = beginningFlatTime;
upToMid = beginningFlatTime+rampUpTime;
midToDown = beginningFlatTime+rampUpTime+midFlatTime;
downToFlat = beginningFlatTime + rampUpTime+midFlatTime + rampDownTime;
totalTime = beginningFlatTime + rampUpTime+midFlatTime + rampDownTime + endFlatTime;

%create time arrays
t_start = linspace(0, beginningFlatTime, datapoints);
t_up = linspace(beginningFlatTime, upToMid, datapoints);
t_mid = linspace(upToMid, midToDown, datapoints);
t_down = linspace(midToDown, downToFlat, datapoints);
t_end = linspace(downToFlat, totalTime, datapoints);

%create trajectory pieces to stitch together
flatTrajStart = PPTrajectory(foh(t_start, zeros(1,length(t_start))));
rampUp = PPTrajectory(foh(t_up,linspace(upMin,upMax, length(t_up))));
flatMid = PPTrajectory(foh(t_mid,zeros(1,length(t_mid))));
rampDown = PPTrajectory(foh(t_down, linspace(downMax,downMin, length(t_down))));
flatTrajEnd = PPTrajectory(foh(t_end, zeros(1,length(t_end))));

%stitch trajectories together
uc = flatTrajStart.append(rampUp).append(flatMid).append(rampDown).append(flatTrajEnd);


ts = [0 5 25 65 85];
us = [0 0 .5 -.5 0];
uc = PPTrajectory(foh(ts,us));

uc = uc.setOutputFrame(frOut);


disp('Sending commands now...')
runLCM(uc,[],[]);
disp('Done');