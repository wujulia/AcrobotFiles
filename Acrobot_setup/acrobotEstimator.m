function acrobotEstimator(obj)

load robotlib_config;
addpath([conf.root,'/examples/Acrobot']);
lcm_coder = AcrobotLCMCoder;

lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
aggregator.setMaxMessages(1);  % make it a last-message-only queue

lc.subscribe('acrobot_y',aggregator); 

x_last=[];
t_last=[];
Lgain=1;
L2=1.5*[1/Lgain*eye(2); 10/Lgain^2*eye(2)];

while true
    msg = getNextMessage(aggregator);  % will block until receiving a message

  if (~isempty(msg)) 
    [y,t]=lcm_coder.decodeY(msg);
    
    u=y(3); y=y(1:2); y=y(:); 
    y(1) = -1*y(1);

    if(~isempty(x_last))
        if(abs(t-t_last) > eps*1000)
            dx=(y-ylast)/(t-t_last);
        else
            dx=x_last(3:4);
        end
        x_pred=x_last+(t-t_last)*obj.dynamics(t_last,x_last,u);
        %options=struct('RelTol',1e-3);
        %[T,X]=ode45(@(tval,xval) obj.dynamics(tval,xval,u), [t_last t], x_last,options);
        %x_pred=X(end,:)';
        x_new=x_pred+L2*(mod(y-x_pred(1:2)+pi,2*pi)-pi);
    else
        dx=zeros(2,1);
        x_new=[y; 0; 0];
    end
    
    %introducing wrapping
    x_new(1:2) = mod(x_new(1:2)+pi,2*pi)-pi;
    ywrapped = mod(y(1:2)+pi,2*pi)-pi;
    
    ylast=y;
    x_last=x_new;
    t_last=t;
    
    nump=1000;
    scope('Acrobot','theta2raw',t,ywrapped(2),struct('scope_id',2,'num_points',nump));
    scope('Acrobot','theta2filtered',t,x_new(2),struct('scope_id',2,'linespec','r--','num_points',nump));
    scope('Acrobot','u',t,u,struct('scope_id',3,'num_points',nump));
    scope('Acrobot','theta1raw',t,ywrapped(1),struct('scope_id',1,'num_points',nump));
    scope('Acrobot','theta1filtered',t,x_new(1),struct('scope_id',1,'linespec','r--','num_points',nump));
    
%     scope('Acrobot','theta1Dotraw',t,dx(1),struct('scope_id',1,'num_points',nump));
%     scope('Acrobot','theta1Dotfiltered',t,x_new(3),struct('scope_id',1,'linespec','r--','num_points',nump));
%     scope('Acrobot','theta2Dotraw',t,dx(2),struct('scope_id',2,'num_points',nump));
%     scope('Acrobot','theta2Dotfiltered',t,x_new(4),struct('scope_id',2,'linespec','r--','num_points',nump));
   
    statemsg = lcm_coder.encodeX(t,x_new);
    lc.publish('acrobot_xhat',statemsg);
  end
end
