classdef AcrobotInputCoder < LCMCoder
% Encodes and Decodes Acrobot-specific LCM smessages 

  methods
    
      function [u,t] = decode(obj,msg)
          % decodes the input message
          msg = acrobot_types.lcmt_acrobot_u(msg.data);
          u = msg.tau;
          t = msg.timestamp/1000;
      end
    
      function msg = encode(obj,t,u)
          % encodes the input message
          msg = acrobot_types.lcmt_acrobot_u();
          msg.timestamp = t*1000;
          msg.tau = -u(1); % Notice sign flip! This makes sure things are same as robotlib visualizer
      end
    
    function str = timestampName(obj)
        str = 'timestamp';
    end
    
    function d = dim(obj)
        d = 1; % 1 input
    end
    
    function names = coordinateNames(obj)
        names = {'tau'};
    end
    
  end
end


