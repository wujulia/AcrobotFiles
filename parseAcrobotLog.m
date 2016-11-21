checkDependency('lcm');
javaaddpath('LCMTypes/acrobot_types.jar')

filename = '/data/mposa/Dropbox (MIT)/AcrobotLogs/11-11-2016/lcmlog-2016-11-11.00';
channels = {'acrobot_y','acrobot_xhat','acrobot_u','acrobot_out'};
coders = {AcrobotYCoder(),AcrobotStateCoder(),AcrobotInputCoder(),AcrobotOutCoder()};
data = readLog(filename,channels,coders);

%%
plant = AcrobotPlantSmooth;
