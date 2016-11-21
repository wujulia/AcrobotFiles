function data=readLog(filename,channels,coders)


n = length(channels);
log = lcm.logging.Log(filename,'r');
% log.seekPositionFraction(.99);
for i=1:n,
  data{i} = LCMData();
  channels{i} = java.lang.String(channels{i}).hashCode();
end

display_frac = .1;

%%
while log.getPositionFraction < 1
  if log.getPositionFraction > display_frac
    display(sprintf('Log %f percent complete',display_frac));
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

for i=1:n,
  data{i}.truncateData();
end