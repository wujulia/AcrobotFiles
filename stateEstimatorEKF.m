% an Extended Kalman Filter state estimator for the Acrobot

checkDependency('lcm');
javaaddpath('LCMTypes/acrobot_types.jar')

q_cal = [pi; 0];

% Calibrate first
x_offset = calibrateAcrobot(q_cal);
% x_offset = zeros(4,1);

storage = LCMStorage('acrobot_u');

p = AcrobotPlantSmooth;

lcm_y_coder = AcrobotYCoder();
lcm_x_coder = AcrobotStateCoder();
lcm_u_coder = AcrobotInputCoder();

lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
aggregator.setMaxMessages(1);  % make it a last-message-only queue

lc.subscribe('acrobot_y',aggregator);

C = [eye(2) zeros(2)]; % measurement model
x_est=[q_cal;0;0];
P = eye(4);  % initial covariance, set arbitrarily. Will converge quickly
R = diag([1e-4;3e-4]); % measurement covariance, from tick resolution
Q = diag([1e-7;1e-7;.01;.01]); %process noise covariance. May need to be tuned.

u_last = 0;

% Observer gain
% completely trust position measurements
L = diag([1;1;.08;.08]);

% Get initial time
msg = aggregator.getNextMessage(1000);
if isempty(msg)
  error('No measurement LCM message received. Ensure that acrobot_y is being published at an appropriate rate')
end
[y,t0]=lcm_y_coder.decode(msg);
t_last = t0;


% fileID = fopen('Acrobot_log_June24_10.txt','w');

% Estimator loop
while true
  msg = aggregator.getNextMessage(100);  % 100ms timeout
  if (~isempty(msg))
    [y,t]=lcm_y_coder.decode(msg);
    if t-t_last > eps
      
      umsg = storage.GetLatestMessage(0);
      
      if ~isempty(umsg)
        u = lcm_u_coder.decode(umsg); 
      else
        u = 0;
      end
      
      % dynamics linearization
      dt = t - t_last;
      [xdot_est,dxdot_est] = p.dynamics(0,x_est,u_last);
      F = eye(4) + dt*dxdot_est(:,2:5);
      
      % predict step
      x_pred = x_est + dt*xdot_est;
      P_pred = F*P*F' + Q;
      
      %update step
      y_resid = y(1:2) - C*x_pred;
      S = C*P_pred*C' + R;
      K = P_pred*C'/S;
      
      x_est = x_pred + K*y_resid;
      P = (eye(4) - K*C)*P_pred;
      
      t_last=t;
      u_last = u;
                  
      
      statemsg = lcm_x_coder.encode(t,x_Est);
      lc.publish('acrobot_xhat',statemsg);      
    else
      display('LCM timeout, no message received')
    end
  end
end



