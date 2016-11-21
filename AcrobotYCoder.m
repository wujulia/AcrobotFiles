classdef AcrobotYCoder < LCMCoder
% Encodes and Decodes Acrobot-specific LCM smessages 

  methods
    
      function [y,t] = decode(obj,msg)
          msg = acrobot_types.lcmt_acrobot_y(msg.data);
          y = [msg.theta1; msg.theta2; 0; 0];
          conv = 1/5215;
          ticks_1 = 32768; %2^15;
          ticks_2 = 10000;
          conv = [2*pi/ticks_1; 2*pi/ticks_2; 0; 0];
          y = conv.*y;
          %warning('thetadot not implemented yet');
          t = msg.timestamp/1000;
      end
    
      function msg = encode(obj,t,y)
          msg = acrobot_types.lcmt_acrobot_y();
          msg.timestamp = t*1000;
          msg.theta1 = y(1);
          msg.theta2 = y(2);
          warning('tau as output is not implemented yet');
          msg.tau = 0;
      end
    
    function str = timestampName(obj)
        str = 'timestamp';
    end
    
    function d = dim(obj)
        d = 4;
    end
    
    function names = coordinateNames(obj)
        names = [];
    end
    
  end
end






