% an Extended Kalman Filter state estimator for the Acrobot
% Also estimating a bias term in the sensor

checkDependency('lcm');
javaaddpath('LCMTypes/acrobot_types.jar')

q_cal = [0; 0];

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

C = [eye(2) zeros(2,4)]; % measurement model
x_est=[q_cal;0;0;0;0];
P = diag([1;1;1;1;.05;.05]);  % initial covariance
R = diag([1e-4;3e-4]); % measurement covariance, from tick resolution
Q = diag([1e-7;1e-7;.01;.01;1e-4;1e-4]); %process noise covariance

T = [eye(4) [eye(2);zeros(2)]];

u_last = 0;


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
    y = y-x_offset;
    if t-t_last > eps
      
      umsg = storage.GetLatestMessage(0);
      
      if ~isempty(umsg)
        u = lcm_u_coder.decode(umsg); 
      else
        u = 0;
      end
      
      % dynamics linearization
      dt = t - t_last;
      [xdot_est,dxdot_est] = p.dynamics(0,T*x_est,u);
      xdot_est = [xdot_est;0;0];
      F = eye(6) + dt*[dxdot_est(:,2:5)*T; zeros(2,6)];
      
      % predict step
      x_pred = x_est + dt*xdot_est;
      P_pred = F*P*F' + Q;
      
      %update step
      y_resid = y(1:2) - C*x_pred;
      S = C*P_pred*C' + R;
      K = P_pred*C'/S;
      
      x_est = x_pred + K*y_resid;
      P = (eye(6) - K*C)*P_pred;
      
      t_last=t;
      u_last = u;
                  
      
      statemsg = lcm_x_coder.encode(t,T*x_est);
      lc.publish('acrobot_xhat',statemsg);      
    else
      display('LCM timeout, no message received')
    end
  end
end



