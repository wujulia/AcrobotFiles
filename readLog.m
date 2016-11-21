function readLog(filename,channels,coders)

filename = '/data/mposa/Dropbox (MIT)/AcrobotLogs/11-11-2016/lcmlog-2016-11-11.00'
log = lcm.logging.Log(filename,'r');
%%
while true
  event = log.readNext;
end