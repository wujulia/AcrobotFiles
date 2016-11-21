function data=readLog(filename,channels,coders)


n = length(channels);
log = lcm.logging.Log(filename,'r');
log.seekPositionFraction(.99);
for i=1:n,
  data{i} = LCMData();
  channels{i} = java.lang.String(channels{i}).hashCode();
end

%%
while log.getPositionFraction < 1
  event = log.readNext();
  code=event.channel.hashCode();
  for i=1:n,
    if isequal(code,channels{i})
      [t,y] = coders{i}.decode(event);
      data{i}.addData(t,y);
      break;
    end
  end
end