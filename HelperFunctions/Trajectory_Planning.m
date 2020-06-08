%% Trajectory Planning- Catmul-Rom Interpolation
%  Use Catmul-Rom splines to produce a smooth trajectory given a set of
%  points. Outputs a timeseries datatype to be used with simulink.
%
%  INPUTS:
%  'Pts' is an array of points [x1,y1 ; x2,y2; x3,y3 ...]
%  'u' is the desired travelling speed of the blimp
%
%  OUTPUTS:
%  'track_dat' is a timeseries datatype to be used with the
%  'Trajectory_Tracking_Subsystem.slx' block in simulink
%
%  https://qroph.github.io/2018/07/30/smooth-paths-using-catmull-rom-splines.html

function track_dat = trajectory_plan(pts,u)
%% Get Points
n = length(pts);

%% Plot Points
figure
hold on
plot(pts(:,1),pts(:,2),'x','MarkerSize',10,'LineWidth',2)

%% Interpolate Endpoints
%First Point
dy  = pts(2,2)-pts(1,2);
dx  = pts(2,1)-pts(1,1);
first_point = [pts(1,1)-dx, pts(1,2)-dy];

%Last Point
dy  = pts(n,2)-pts(n-1,2);
dx  = pts(n,1)-pts(n-1,1);
last_point = [pts(n,1)+dx, pts(n,2)+dy];

points = [first_point;pts;last_point];

%% Apply Catmul-Rom Curves
alpha = 0.5; %Catmul Rom Parameters 
tension=0.1;

tj = @(Pi,Pj,t_i) (sqrt( (Pj(1)-Pi(1))^2 + (Pj(2)-Pi(2))^2 ))^alpha + t_i;
t = 0:0.01:1;
t = transpose(t);
trajectory = [];
    
for i=2:n
    p0 = points(i-1,:);
    p1 = points(i,:);
    p2 = points(i+1,:);
    p3 = points(i+2,:);

    %% Find t0-t3
    t0 = 0;
    t1 = tj(p0,p1,t0);
    t2 = tj(p1,p2,t1);
    t3 = tj(p2,p3,t2);

    %% Find coefficients
    m1 = (1-tension)*(t2-t1)*( (p1-p0)/(t1-t0) - (p2-p0)/(t2-t0) + (p2-p1)/(t2-t1));
    m2 = (1-tension)*(t2-t1)*( (p2-p1)/(t2-t1) - (p3-p1)/(t3-t1) + (p3-p2)/(t3-t2));
    
    a = 2*p1 - 2*p2 + m1 + m2  ;
    b =-3*p1 + 3*p2 - 2*m1 - m2;
    c = m1                     ;
    d = p1                     ;
    
    %% Plot Curve   
    p = a.*t.^3 + b.*t.^2 + c.*t + d;
    plot(p(:,1),p(:,2),'r-')
    
    p = p(1:length(p)-1, :);
    trajectory = [trajectory;p];
end

title('Trajectory Planning')
xlabel('x')
ylabel('y')
pbaspect([1 1 1])

%% Create timeseries data

time = zeros(length(trajectory)-1,1); %initialise time vector

for i=1:length(trajectory)-1
    %Get heading angle (+ve is counter-clockwise)
    psi(i,1)= atan2((trajectory(i+1,2)-trajectory(i,2)),(trajectory(i+1,1)-trajectory(i,1)));
    
    %Get distance between points
    dst = sqrt((trajectory(i+1,2)-trajectory(i,2))^2 + (trajectory(i+1,1)-trajectory(i,1))^2);
    
    %Calculate time to reach point
    time(i,1) = dst/u;
    cumu_time(i,1) = sum(time(1:i));
end

for i=1:length(trajectory)-2 % To stop sudden 360 rotations assuming smooth trajectories NEEDS FIXING
    if psi(i+1)-psi(i) > pi %jumps from -ve to +ve 
        
        psi(i+1) = -pi - (pi-psi(i+1));
        
    elseif psi(i+1)-psi(i) < -pi %jumps from +ve to -ve
          
        psi(i+1) = pi + (pi+psi(i+1));
        
    end
end

%Output Timeseries data for use in simulink
track_dat = timeseries([trajectory(1:length(trajectory)-1,1),trajectory(1:length(trajectory)-1,2),psi],cumu_time); %Output to simulink
end



    
    
    
    