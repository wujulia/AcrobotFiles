classdef AcrobotFeedbackController < DrakeSystem
    % A generic feedback controller meant for the Acrobot 
    
    properties
        u_0;
        K;
        x_desired;
        storage
    end
    
  methods
    function obj = AcrobotFeedbackController(u_0, K, x_desired)
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
        
        % initialize properties
        obj.x_desired = x_desired;
        obj.K = K;
        obj.u_0 = u_0;
        
        obj.storage = LCMStorage('acrobot_storage');
        obj.storage.storage_struct.t_offset = [];

        % setup LCM frames
        lcmInFrame = LCMCoordinateFrameWCoder('acrobot_xhat', 4, 'x', AcrobotStateCoder);
        obj = obj.setInputFrame(lcmInFrame);
        
        lcmOutFrame = LCMCoordinateFrameWCoder('acrobot_u',1,'u',AcrobotInputCoder);
        obj = obj.setOutputFrame(lcmOutFrame);
        
    end

    function u = output(obj,t,~,x)
        if isempty(obj.storage.storage_struct.t_offset)
          obj.storage.storage_struct.t_offset = t;
        end
        
        t_offset = obj.storage.storage_struct.t_offset;
        
        % Implements control function.
        
        %evaluate u_0, x_desired, K for current time t
        current_u_0 = obj.u_0.eval(t-t_offset);
        current_x_des = obj.x_desired.eval(t-t_offset);
        current_K = obj.K.eval(t-t_offset);
        
        %control function
        u = current_u_0 - current_K*(x - current_x_des);
        
        umax = 9;
        u = max(u,-umax);
        u = min(u,umax);
    end
    
  end
  
  
  
end
