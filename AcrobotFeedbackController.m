classdef AcrobotFeedbackController < DrakeSystem
    
    
    properties

        u_0; %PPTrajectory
        K; %Trajectory
        x_desired; %PPTrajectory

    end
    
  methods
    function obj = AcrobotFeedbackController(u_0, K, x_desired)
        
        obj = obj@DrakeSystem(0, 0, 4, 1);
        
        obj.x_desired = x_desired;
        obj.K = K;
        obj.u_0 = u_0;

        
        lcmInFrame = LCMCoordinateFrameWCoder('acrobot_xhat', 4, 'x', AcrobotStateCoder);
        obj = obj.setInputFrame(lcmInFrame);
        
        lcmOutFrame = LCMCoordinateFrameWCoder('acrobot_u',1,'u',AcrobotInputCoder);
        obj = obj.setOutputFrame(lcmOutFrame);
        
    end

    function u = output(obj,t,~,x)
        % Implements control function.
        
        current_u_0 = obj.u_0.eval(t);
        current_x_des = object.x_desired.eval(t);
        current_K = obj.K.eval(t);
        u = current_u_0 - current_K*(x - current_x_des);
        
        
    end
    
  end
  
  
  
end
