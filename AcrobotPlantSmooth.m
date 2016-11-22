classdef AcrobotPlantSmooth < Manipulator
  % Defines the dynamics for the real acrobot
  % Updated on Jan 20, 2014.
  %#ok<*PROPLC>
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
      obj = obj@Manipulator(2,1,2);
      obj = obj.setOutputFrame(obj.getStateFrame);
    end    
    
    function [H,C,B,dH,dC,dB] = manipulatorDynamics(obj,q,v)      
      % Compute manipulator dynamics
      % keep it readable:
      m1=obj.m1; m2=obj.m2; l1=obj.l1; g=obj.g; lc1=obj.lc1; lc2=obj.lc2; I1=obj.I1; I2=obj.I2; b1=obj.b1; b2=obj.b2;
      m2l1lc2 = m2*l1*lc2;  % occurs often!
      
      c = cos(q(1:2,:));  s = sin(q(1:2,:));  s12 = sin(q(1,:)+q(2,:)); c12 = cos(q(1,:)+q(2,:));
      
      h12 = I2 + m2l1lc2*c(2);
      H = [ I1 + I2 + m2*l1^2 + 2*m2l1lc2*c(2), h12; h12, I2 ];
      
      
      C1 = [ -2*m2l1lc2*s(2)*v(2), -m2l1lc2*s(2)*v(2); m2l1lc2*s(2)*v(1), 0 ];
      

      
      G = g*[ m1*lc1*s(1) + m2*(l1*s(1)+lc2*s12); m2*lc2*s12 ];
      
      
      
      % accumate total C and add a damping term:
      b = diag([b1;b2]);
      C = C1*v + G + b*v;
      
      B = [0; 1];
      
      if nargout > 1
        dHdq = [zeros(4,1) [-2*m2l1lc2*s(2); -m2l1lc2*s(2); -m2l1lc2*s(2);0]];
        dH = [dHdq zeros(4,2)];
        
        dG = g*[[m1*lc1*c(1) + m2*(l1*c(1)+lc2*c12), m2*lc2*c12; m2*lc2*c12, m2*lc2*c12], zeros(2,2)];
        dC1dq2 = [-2*m2l1lc2*c(2)*v(2); m2l1lc2*c(2)*v(1);-m2l1lc2*c(2)*v(2); 0];
        dC1dv = [0 -2*m2l1lc2*s(2); m2l1lc2*s(2) 0; 0 -m2l1lc2*s(2); 0 0];
        dC1 = [zeros(4,1), dC1dq2, dC1dv];
        dC = matGradMult(dC1,v) + [zeros(2,2) C1+b] + dG;
        
        dB = zeros(2,4);
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






