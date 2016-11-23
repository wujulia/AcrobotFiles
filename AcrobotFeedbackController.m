classdef AcrobotFeedbackController < DrakeSystem
    % A generic feedback controller meant for the Acrobot 
    
    properties
        
        u_0;
        K;
        x_desired;
        p;
    end
    
  methods
    function obj = AcrobotFeedbackController(p, u_0, K, x_desired)
        % constructor for a generic feedback controller
        %
        % @param u_0 : A Trajectory object representing input torques.
        %              Varies with time.
        %              Needs to have a eval(t) method.
        %
        % @param K : Also a Trajectory object representing gains.
        %            Varies with time.
        %            Needs to have an eval(t) method.
        %
        % @param x_desired : A Trajectory object representing our target
        %                    Trajectory. Varies with Time. 
        %                    Needs to have an eval(t) method.
        
        
        % initialize as DrakeSystem with 4 inputs and 1 output
        obj = obj@DrakeSystem(0, 0, 4, 1);       
        
        
        if isnumeric(u_0)
          u_0 = ConstantTrajectory(u_0);
        end
        
        if isnumeric(K)
          K = ConstantTrajectory(K);
        end
        
        if isnumeric(x_desired)
          x_desired = ConstantTrajectory(x_desired);
        end
        
        % initialize properties
        obj.x_desired = x_desired;
        obj.K = K;
        obj.u_0 = u_0;
        obj.p = p;

        setup LCM frames
        lcmInFrame = LCMCoordinateFrame('acrobot_xhat', AcrobotStateCoder, 'x');
        obj = obj.setInputFrame(lcmInFrame);

        lcmOutFrame = LCMCoordinateFrame('acrobot_u',AcrobotInputCoder,'u');
        obj = obj.setOutputFrame(lcmOutFrame);
        
    end

    function u = output(obj,t,~,x)
        % Implements control function.
        
        %evaluate u_0, x_desired, K for current time t
        current_u_0 = obj.u_0.eval(t);
        current_x_des = obj.x_desired.eval(t);
        current_K = obj.K.eval(t);
        
        %control function
        u = current_u_0 - current_K*(x - current_x_des);
        
        umax = 9;
        u = max(u,-umax);
        u = min(u,umax);
    end
    
  end
  
  
  
end
