function runTrajectorySwingup

ad = AcrobotPlant;
av = AcrobotVisualizer;
c = TrajectorySwingupAcro(ad);

for i=1:5
  xtraj = simulate(ad,c,[0 6],.01*randn(2,1));
  playback(av,xtraj);
end
