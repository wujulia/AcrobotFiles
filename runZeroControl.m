% Robot
p = AcrobotPlantSmooth();

% Frame to publish input commands to
frOut = LCMCoordinateFrameWCoder('acrobot_u',1,'u',AcrobotInputCoder);

% Constant controller
% uc = ConstantControl();

uc = ConstantTrajectory(Point(frOut, 0.0)); % 1.0 Nm

disp('Sending commands now...')
runLCM(uc,[],[]);
disp('Done');

%PPTrajectory(foh(t,x))