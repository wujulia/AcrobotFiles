classdef AcrobotPlantSmooth < DrakeSystem
  % Defines the dynamics for the real acrobot
  % Updated on Jan 20, 2014.
  
  properties
    
    
    %      % SysID data from July 16
    %     m1 = 2.1753;
    %     m2 = 0.1382;
    %     l1 = 0.4820;
    %     lc1 = 0.6019;
    %     lc2 = 3.4351;
    %     b1 = -0.0055;
    %     b2 = 0.0496;
    %     I1 = 0.6707;
    %     I2 = 0.2330;
    
    % July 26
    m1 = 2.1734;
    m2 = 0.2466;
    l1 = 0.4849;
    lc1 = 0.5963;
    lc2 = 1.9293;
    b1 = -0.0378;
    b2 = 0.0684;
    I1 = 0.6658;
    I2 = 0.2267;
    
    
    l2 = 0.5;
    g = 9.81;
    
  end
  
  methods
    function obj = AcrobotPlantSmooth()
      obj = obj@DrakeSystem(4,0,1,4,0,1);
      obj = obj.setOutputFrame(obj.getStateFrame);
    end
    
    function [xdot,df] = dynamics(obj,t,x,u)
      
      if nargout > 1
        [xdot,df] = geval(@obj.dynamicsFun,t,x,u);
      else
        xdot = obj.dynamicsFun(t,x,u);
      end
    end
    
    function xdot = dynamicsFun(obj,t,x,u)
      q = x(1:2);
      qd = x(3:4);
      
      xdot = x*0;
      
      % First derivatives
      xdot(1) = x(3);
      xdot(2) = x(4);
      
      % Compute manipulator dynamics
      % keep it readable:
      m1=obj.m1; m2=obj.m2; l1=obj.l1; g=obj.g; lc1=obj.lc1; lc2=obj.lc2; I1=obj.I1; I2=obj.I2; b1=obj.b1; b2=obj.b2;
      m2l1lc2 = m2*l1*lc2;  % occurs often!
      
      c = cos(q(1:2,:));  s = sin(q(1:2,:));  s12 = sin(q(1,:)+q(2,:));
      
      h12 = I2 + m2l1lc2*c(2);
      H = [ I1 + I2 + m2*l1^2 + 2*m2l1lc2*c(2), h12; h12, I2 ];
      
      C = [ -2*m2l1lc2*s(2)*qd(2), -m2l1lc2*s(2)*qd(2); m2l1lc2*s(2)*qd(1), 0 ];
      G = g*[ m1*lc1*s(1) + m2*(l1*s(1)+lc2*s12); m2*lc2*s12 ];
      
      % accumate total C and add a damping term:
      C = C*qd + G + [b1;b2].*qd;
      
      B = [0; 1];
      tau=B*u;
      
      % Compute second derivatives
      qdd = H\(tau - C);
      xdot(3) = qdd(1);
      xdot(4) = qdd(2);
      
      if (nargout>1)
        %             [df,d2f,d3f]= dynamicsGradients(obj,t,x,u,nargout-1);
      end
    end
    
    function y = output(obj,t,x,u)
      y = x;
    end
    
    function x = getInitialState(obj)
      x = [0 0 0 0]';
    end
    
  end
  
end






