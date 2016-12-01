classdef AcrobotFeedbackController < DrakeSystem
  % A generic feedback controller meant for the Acrobot
  
  properties
    plant
    u_0;
    K;
    x_desired;
    storage
    t_zero % period at start to output zero
  end
  
  methods
    function obj = AcrobotFeedbackController(plant,u_0, K, x_desired, t_zero)
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
      
      if nargin < 5
        t_zero = 10;
      end
      obj.t_zero = t_zero;
      
      if isnumeric(x_desired)
        x_desired = ConstantTrajectory(x_desired);
      end
      
      if isnumeric(K)
        K = ConstantTrajectory(K);
      end
      
      if isnumeric(u_0)
        u_0 = ConstantTrajectory(u_0);
      end
      
      % initialize properties
      obj.plant = plant;
      obj.x_desired = x_desired;
      obj.K = K;
      obj.u_0 = u_0;
      
      obj.storage = LCMStorage('acrobot_storage');
      obj.storage.storage_struct.t_offset = [];
      
      
      obj = obj.setInputFrame(plant.getStateFrame);
      
      % setup LCM frame
      lcmOutFrame = LCMCoordinateFrameWCoder('acrobot_u',1,'u',AcrobotInputCoder);
      obj = obj.setOutputFrame(lcmOutFrame);
      
    end
    
    function u = output(obj,t,~,x)
      if isempty(obj.storage.storage_struct.t_offset)
        obj.storage.storage_struct.t_offset = t + obj.t_zero;
      end
      t_offset = obj.storage.storage_struct.t_offset;
      t = t - t_offset;
      
      if t < 0 || t > obj.x_desired.tspan(2)
        u = 0;
      else
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
%       u
    end
    
  end
  
  
  
end
