lcm_coder = AcrobotYCoder();

lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
aggregator.setMaxMessages(1);  % make it a last-message-only queue

lc.subscribe('acrobot_y',aggregator); 

msg = getNextMessage(aggregator);

[y,t]=lcm_coder.decode(msg);
q1 = -y(1); % Note the sign flip!
q2 = -y(2); % Note the sign flip!

x_offset = [q1;q2;0;0];