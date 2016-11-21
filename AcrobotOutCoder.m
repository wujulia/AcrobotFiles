classdef AcrobotOutCoder < LCMCoder
% Encodes and Decodes Acrobot-specific LCM smessages 

  methods
    
      function [x,t] = decode(obj,msg)
          % decodes the state message
          msg = acrobot_types.lcmt_motor_state(msg.data);
          x = [msg.current; msg.position; msg.position2; msg.fault];
          t = msg.timestamp/1000;
      end
    
      function msg = encode(obj,t,x)
          % encodes the state message
          msg = acrobot_types.lcmt_motor_state();
          msg.current = x(1);
          msg.position = x(2);
          msg.position2 = x(3);
          msg.fault = x(4);
          error('Not implemented');
      end
    
    function str = timestampName(obj)
        str = 'timestamp';
    end
    
    function d = dim(obj)
        d = 4;
    end
    
    function names = coordinateNames(obj)
        names = {'current','position','position2','fault'};
    end
    
  end
end







