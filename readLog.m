function data=readLog(filename,channels,coders,start_frac,end_frac)

if nargin < 4
  start_frac = 0;
end
if nargin < 5
  end_frac = 1;
end


n = length(channels);
log = lcm.logging.Log(filename,'r');
log.seekPositionFraction(start_frac);
for i=1:n,
  data{i} = LCMData();
  channels{i} = java.lang.String(channels{i}).hashCode();
end

display_frac = .1;

%%
while log.getPositionFraction < end_frac
  if log.getPositionFraction > display_frac
    display(sprintf('Log %d percent complete',display_frac*100));
    display_frac = display_frac + .1;
  end
  event = log.readNext();
  code=event.channel.hashCode();
  for i=1:n,
    if isequal(code,channels{i})
      [y,t] = coders{i}.decode(event);
      data{i}.addData(t,y);
      break;
    end
  end
end

display('Log 100 percent complete');

for i=1:n,
  data{i}.truncateData();
end

