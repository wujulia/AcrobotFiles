checkDependency('lcm');
javaaddpath('LCMTypes/acrobot_types.jar')

filename = '/data/mposa/Dropbox (MIT)/AcrobotLogs/11-11-2016/lcmlog-2016-11-11.00';
channels = {'acrobot_y','acrobot_xhat','acrobot_u','acrobot_out'};
coders = {AcrobotYCoder(),AcrobotStateCoder(),AcrobotInputCoder(),AcrobotOutCoder()};
data = readLog(filename,channels,coders);

%%
plant = AcrobotPlantSmooth;

t = data{2}.t;
x = data{2}.data;
u = data{3}.data;

u = interp1(data{3}.t,u,t);

t = t-t(1);


xdot = x*0;
for i=1:length(t),
  xdot(:,i) = plant.dynamics(t(i),x(:,i),u(i));
end

q = x(1:2,:);
v = x(3:4,:);
v_diff = [diff(q(1,:))./diff(t);diff(q(2,:))./diff(t)];
v_diff = [zeros(2,1) v_diff];

vdot_diff = [diff(v(1,:))./diff(t);diff(v(2,:))./diff(t)];
vdot_diff = [zeros(2,1) vdot_diff];