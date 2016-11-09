classdef feedbackControl < DrakeSystem
    
    
    properties

        u_0; %PPTrajectory
        K; %drake system
        x_desired;%PPTrajectory

    end
    
  methods
    function obj = feedbackControl(u_0, K, x_desired)
        
        obj = obj@DrakeSystem(0, 0, 4, 1);
        
        obj.x_desired = x_desired;
        obj.K = K;
        obj.u_0 = u_0;
        
        obj.x_desired = x_desired;

        
        lcmInFrame = LCMCoordinateFrameWCoder('acrobot_xhat', 4, 'x', AcrobotStateCoder);
        obj = obj.setInputFrame(lcmInFrame);
        
        lcmOutFrame = LCMCoordinateFrameWCoder('acrobot_u',1,'u',AcrobotInputCoder);
        obj = obj.setOutputFrame(lcmOutFrame);
        
    end

    function u = output(obj,t,junk,x)
        % Implements control function.
        u = obj.u_0 + obj.K*(x - obj.x_desired);
        
    end
    
  end
  
  
  
end
