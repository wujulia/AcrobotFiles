function x_offset = calibrateAcrobot(q0)
% Calibrate encoders about position q0, likely either [0;0] or [pi;0]

display(sprintf('Calibrating position encoders. Hold acrobot at (%f,%f) and press any key.',q0(1),q0(2)))
pause

lcm_coder = AcrobotYCoder();

lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
aggregator.setMaxMessages(1);  % make it a last-message-only queue

lc.subscribe('acrobot_y',aggregator);

display('Calibration: waiting for LCM message')
msg = [];
while isempty(msg)
  msg = aggregator.getNextMessage(100); %timeout
end

[y,t]=lcm_coder.decode(msg);
q1 = y(1);
q2 = y(2);

% calibrate about upright
x_offset = [q1;q2;0;0] - [q0;0;0];

display('Calibration complete')
end