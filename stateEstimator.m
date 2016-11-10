checkDependency('lcm');

store_histories = true;
do_scope = false;

% Calibrate first
% x_offset = calibrateAcrobot([pi;0]);
x_offset = zeros(4,1);

storage = LCMStorage('acrobot_u');

p = AcrobotPlantSmooth;

lcm_y_coder = AcrobotYCoder();
lcm_x_coder = AcrobotStateCoder();
lcm_u_coder = AcrobotInputCoder();

lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
aggregator.setMaxMessages(1);  % make it a last-message-only queue

lc.subscribe('acrobot_y',aggregator);

x_last=[];
t_last=[];
u_last = 0;
x_new = [];

% Observer gain
% completely trust position measurements
L = diag([1;1;.08;.08]);

% Get initial time
msg = aggregator.getNextMessage(1000);
if isempty(msg)
  error('No measurement LCM message received. Ensure that acrobot_y is being published at an appropriate rate')
end
[y,t0]=lcm_y_coder.decode(msg);

if store_histories
  x_hist = zeros(4,1e5);
  u_hist = zeros(1,1e5);
  t_hist = zeros(1,1e5);
  hist_index = 1;
end


% fileID = fopen('Acrobot_log_June24_10.txt','w');

% Estimator loop
while true
  msg = aggregator.getNextMessage(100);  % 100ms timeout
  if (~isempty(msg))
    [y,t]=lcm_y_coder.decode(msg);
    %u = lcm_coder.decodeU(getNextMessage(aggregatoru));
    umsg = storage.GetLatestMessage(100);
    
    if ~isempty(umsg)
      u = -lcm_u_coder.decode(umsg); % Sign flip
    else
      u = 0;
    end    
    
    q1 = -y(1); % Note the sign flip!
    q2 = -y(2); % Note the sign flip!
    
    if(~isempty(x_last))
      if(abs(t-t_last) > eps)
        q1dot = (q1-q_last(1))/(t-t_last);
        q2dot = (q2-q_last(2))/(t-t_last);
      else
        q1dot = 0;
        q2dot = 0;
      end
    end
      
    x_est = [q1;q2;q1dot;q2dot] - x_offset;
    
    % observer dynamics
    xdot_pred = p.dynamics(0,x_last,u_last);    
    x_pred = x_last + (t-t_last)*xdot_pred;
    x_new = x_pred  + L*(x_est-x_last);
    
    
    %introducing wrapping
    %x_new(1:2) = mod(x_new(1:2)+pi/2,2*pi)-pi/2;
    
%     x_new(1:2)
    
    % x_hist = [x_hist,x_new];
    % t_hist = [t_hist,t-t0];
    % u_hist = [u_hist,u];
    
    q_last = [q1;q2];
    x_last=x_new;
    t_last=t;
    
    u_last = u;
    
    
    % record histories
    if store_histories
      u_hist(hist_index) = u;
      t_hist(hist_index) = t;
      x_hist(:,hist_index) = x_new;
      hist_index = hist_index + 1;
      
      % dynamically double the size if its filling up
      if (hist_index == length(u_hist)),        
        x_hist = [x_hist, zeros(4,length(u_hist))];
        t_hist = [t_hist, zeros(1,length(u_hist))];
        u_hist = [u_hist, zeros(1,length(u_hist))];
      end
    end
    
    if do_scope
      % code below is probably very old and won't work, preserving it for
      % now
      nump=10000;
      scope('Acrobot','theta2raw',t,ywrapped(2),struct('scope_id',2,'num_points',nump));
      scope('Acrobot','theta2filtered',t,x_new(2),struct('scope_id',2,'linespec','r--','num_points',nump));
      scope('Acrobot','u',t,u,struct('scope_id',3,'num_points',nump));
      scope('acrobot','theta1raw',t,x_new(1),struct('scope_id',1,'num_points',nump));
      scope('acrobot','theta1dot',t,x_new(3),struct('scope_id',1,'linespec','r--','num_points',nump));
    end
    
    statemsg = lcm_x_coder.encode(t,x_new);
    lc.publish('acrobot_xhat',statemsg);
    
    %         fprintf(fileID,'%12.8f %12.8f %12.8f %12.8f\n', x_new);
  else
    display('LCM timeout, no message received')
  end
end



