% Calibrate first 
calibrate

storage = LCMStorage('acrobot_u');


p = AcrobotPlantSmooth;

lcm_y_coder = AcrobotYCoder();
lcm_x_coder = AcrobotStateCoder();
lcm_u_coder = AcrobotInputCoder();

lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
aggregator.setMaxMessages(1);  % make it a last-message-only queue

lc.subscribe('acrobot_y',aggregator); 

x_last=[];
t_last=[];
u_last = 0;
x_new = [pi;0;0;0];

x_offset = x_offset - [pi;0;0;0];

% t_hist = [];
% x_hist = [];
% u_hist = [];

minV = Inf;
V0set = false;
V0 = Inf;
Vfset = false;

L = 0.08; % 0.11

% Get initial time
[y,t0]=lcm_y_coder.decode(msg);

torques_hist = [];


% fileID = fopen('Acrobot_log_June24_10.txt','w');

% Estimator loop
while true
    msg = getNextMessage(aggregator);  % will block until receiving a message
    if (~isempty(msg)) 
        [y,t]=lcm_y_coder.decode(msg);
        %u = lcm_coder.decodeU(getNextMessage(aggregatoru));
        umsg = storage.GetLatestMessage(100);

        if (length(umsg) > 0)
            u = -lcm_u_coder.decode(umsg); % Sign flip 
        else
            u = 0;
        end
        
        torques_hist = [torques_hist, u];

        
        q1 = -y(1); % Note the sign flip!
        q2 = -y(2); % Note the sign flip!

        if(~isempty(x_last))
            if(abs(t-t_last) > eps)
                q1dot = (q1-q_last(1))/(t-t_last);
                q2dot = (q2-q_last(2))/(t-t_last);
            else
                q1dot = 0;
                q2dot = 0;
            end

             % Observer for velocity
             xdot_pred = p.dynamics(0,x_last,u_last); 
             
             x_new1 = x_last + (t-t_last)*xdot_pred;
             x_new = x_last + (t-t_last)*(xdot_pred) + L*([q1;q2;q1dot;q2dot]-x_last);
             x_new = [q1;q2;x_new(3);x_new(4)] - x_offset; 
            
            
        else
            q1dot = 0; %q1dot_hist(end);
            q2dot = 0; %q2dot_hist(end);
        end
        
        % Check if state is in initial ellipse
        x = x_new;

        % V0 for sos controller from sept 14
        % V0 = 391.64*x(1)^2+71.245*x(2)^2+192.14*x(2)*x(1)+22.483*x(3)^2+10.975*x(3)*x(1)+3.63*x(3)*x(2)+14.473*x(4)^2-0.20286*x(4)*x(1)+0.59651*x(4)*x(2)+9.408*x(4)*x(3);
        % V0 = 390.93*x(1)^2+75.547*x(2)^2+188.78*x(2)*x(1)+23.888*x(3)^2+14.666*x(3)*x(1)+5.0599*x(3)*x(2)+9.4736*x(4)^2-1.791*x(4)*x(1)-0.93173*x(4)*x(2)+16.093*x(4)*x(3);
              
        V0 = 407.2*x(1)^2+67.905*x(2)^2+205.76*x(2)*x(1)+17.128*x(3)^2+23.579*x(3)*x(1)+7.7175*x(3)*x(2)+7.6101*x(4)^2+6.3658*x(4)*x(1)+1.5459*x(4)*x(2)+8.8151*x(4)*x(3);
        if V0 < 0.9*18.6359 && ~V0set 
        % if V0 < 0.9*10.6226 && ~V0set
            tbegin = t-t0;
            % indBegin = length(t_hist)+1;
            V0set = true;
        end
        
        % Check if state is in final ellipse
        x = x_new - [pi;0;0;0];
    
        % Vf = 2.479723607014511*x(1)^2+1.366901555121524*x(1)*x(2)+0.190371062473987*x(2)^2+1.357652853883092*x(1)*x(3)+0.374274022700988*x(2)*x(3)+0.195526082425103*x(3)^2+0.433760969468681*x(1)*x(4)+0.119629166385022*x(2)*x(4)+0.124916802683898*x(3)*x(4)+0.019978061392570*x(4)^2;

        Vf = 2.69954233293504*x(1)^2+1.48444051277168*x(1)*x(2)+0.20599115755804*x(2)^2+1.47074667221093*x(1)*x(3)+0.40499545387148*x(2)*x(3)+0.21168733528110*x(3)^2+0.46457734054724*x(1)*x(4)+0.12802904526026*x(2)*x(4)+0.13360570728372*x(3)*x(4)+0.02113044820908*x(4)^2;
        if Vf < 1 && (t - t0 > 7) && ~Vfset 
           tend = t - t0;
           % indEnd = length(t_hist)+1;
           L = 0.05; % 0.15
           Vfset = true;
        end

        if t-t0 > 5
           minV = min(Vf,minV);
        end
        

        %introducing wrapping
        %x_new(1:2) = mod(x_new(1:2)+pi/2,2*pi)-pi/2;
        
        x_new(1:2)
        
        % x_hist = [x_hist,x_new];
        % t_hist = [t_hist,t-t0];
        % u_hist = [u_hist,u];
        
        q_last = [q1;q2];
        x_last=x_new;
        t_last=t;
        
        u_last = u;

        % nump=10000;
        %scope('Acrobot','theta2raw',t,ywrapped(2),struct('scope_id',2,'num_points',nump));
        %scope('Acrobot','theta2filtered',t,x_new(2),struct('scope_id',2,'linespec','r--','num_points',nump));
        %scope('Acrobot','u',t,u,struct('scope_id',3,'num_points',nump));
        
%         scope('acrobot','theta1raw',t,x_new(1),struct('scope_id',1,'num_points',nump));
%         scope('acrobot','theta1dot',t,x_new(3),struct('scope_id',1,'linespec','r--','num_points',nump));

        statemsg = lcm_x_coder.encode(t,x_new);
        lc.publish('acrobot_xhat',statemsg);
        
%         fprintf(fileID,'%12.8f %12.8f %12.8f %12.8f\n', x_new);
    end
end



