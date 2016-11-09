% Robot
p = AcrobotPlantSmooth();

% Frame to publish input commands to
frOut = LCMCoordinateFrameWCoder('acrobot_u',1,'u',AcrobotInputCoder);

ts = linspace(0,10,1e4);
uc = PPTrajectory(foh(ts,sin(ts)));
uc = uc.setOutputFrame(frOut);


disp('Sending commands now...')
runLCM(uc,[],[]);
disp('Done');
