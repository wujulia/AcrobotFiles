classdef AcrobotStateCoder < LCMCoder
% Encodes and Decodes Acrobot-specific LCM smessages 

  methods
    
      function [x,t] = decode(obj,msg)
          % decodes the state message
          if isnumeric(msg)
            msg = acrobot_types.lcmt_acrobot_x(msg);
          else
            msg = acrobot_types.lcmt_acrobot_x(msg.data);
          end
          x = [msg.theta1; msg.theta2; msg.theta1Dot; msg.theta2Dot];
          t = msg.timestamp/1000;
      end
    
      function msg = encode(obj,t,x)
          % encodes the state message
          msg = acrobot_types.lcmt_acrobot_x();
          msg.timestamp = t*1000;
          msg.theta1 = x(1); 
          msg.theta2 = x(2);
          msg.theta1Dot = x(3);
          msg.theta2Dot = x(4);
      end
    
    function str = timestampName(obj)
        str = 'timestamp';
    end
    
    function d = dim(obj)
        d = 4;
    end
    
    function names = coordinateNames(obj)
        names = {'theta1','theta2','theta1dot','theta2dot'};
    end
    
  end
end







