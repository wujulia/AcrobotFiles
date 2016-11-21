clear all
checkDependency('lcm');
javaaddpath('LCMTypes/acrobot_types.jar')


storage = LCMStorage('acrobot_u');

lcm_y_coder = AcrobotYCoder();
lcm_x_coder = AcrobotStateCoder();
lcm_u_coder = AcrobotInputCoder();

lc = lcm.lcm.LCM.getSingleton();
y_aggregator = lcm.lcm.MessageAggregator();
y_aggregator.setMaxMessages(1000);

u_aggregator = lcm.lcm.MessageAggregator();
u_aggregator.setMaxMessages(1000);

lc.subscribe('acrobot_y',y_aggregator);
lc.subscribe('acrobot_u',u_aggregator);



x_data = zeros(4,1e5);
u_data = zeros(1,1e5);
t_data = zeros(1,1e5);
tu_data = zeros(1,1e5);
y_data = zeros(4,1e5);
data_index = 1;
udata_index = 1;

while true
  msg = y_aggregator.getNextMessage(100);  % 100ms timeout
  if (~isempty(msg))
    [y,t]=lcm_y_coder.decode(msg);
    
    
    % record data
    %         u_data(data_index) = u;
    t_data(data_index) = t;
    %         x_data(:,data_index) = x_new;
    y_data(:,data_index) = y;
    data_index = data_index + 1;
    
    % dynamically double the size if its filling up
    if (data_index == length(u_data)),
      y_data = [y_data, zeros(4,length(u_data))];
%       x_data = [x_data, zeros(4,length(u_data))];
      t_data = [t_data, zeros(1,length(u_data))];
%       u_data = [u_data, zeros(1,length(u_data))];
    end
  else
    display('No message')
  end
  
  msg = u_aggregator.getNextMessage(100);  % 100ms timeout
  if (~isempty(msg))
    [u,t]=lcm_u_coder.decode(msg);
    
    
    % record data
            u_data(udata_index) = u;
    tu_data(udata_index) = t;
    %         x_data(:,data_index) = x_new;
%     y_data(:,data_index) = y;
    udata_index = udata_index + 1;
    
    % dynamically double the size if its filling up
    if (udata_index == length(u_data)),
%       y_data = [y_data, zeros(4,length(u_data))];
%       x_data = [x_data, zeros(4,length(u_data))];
      tu_data = [tu_data, zeros(1,length(u_data))];
      u_data = [u_data, zeros(1,length(u_data))];
    end
  else
    display('No message')
  end
end