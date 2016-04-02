%% Trajectory tracking
clear;

% Fixed vehicle parameters
velocity = 10; % Speed
delta_max = 25*pi/180; % max steering angle
k = 25; % Gain
robot_length = 1; % Car length

% Desired trajectory as a plot of points. The plot of points defines
% multiple line segments all joined together. The vehicle will first follow
% along the first line segment. After it is has driven past the length of
% said line segment, it defines a new line segment to follow by taking the
% next point.

% box
box_size = [20, 5];
box_speed = 3; %metres per second

traj_points = [0 0];

traj_point_counter = 1; % Keep track of where we are in the trajectory
           
% Initial conditions in [x y heading]
x0 = [0 15 0]; 

% Simulation time
Tmax = 15;  % End point
dt =0.01; % Time step
T = 0:dt:Tmax; % Time vector

% Simulation setup
xd = zeros(length(T)-1,3); % Derivative of state ([edot psidot])
x = zeros(length(T),3);  % State ([e psi] 
x(1,:) = x0; % Initial condition
delta = zeros(length(T),1); % Steering angles

figure(1);clf; hold on;
axis([-1 30 -20 20]);
for i=1:length(T)-1
    % Update location of box
    traj_points = [traj_points(1)+box_speed*dt traj_points(2)];
    
    % The desired trajectory is a line segment consisting of 2 points from
    % the desired trajectory
    end_point = traj_points;
    beginning = [x(end,1) x(end,2)]; %current robot location
    traj_angle = atan2(end_point(2) - beginning(2), end_point(1) - beginning(1));
    
    [crosstrack_error, next_point] = distanceToLineSegment(beginning,end_point,x(i,1:2));
    
    % Calculate steering angle
    delta(i) = max(-delta_max,min(delta_max, angleWrap(traj_angle - x(i,3))+ atan2(-k*crosstrack_error,velocity)));
    % State derivatives
    xd(i,1) = velocity*cos(x(i,3));
    xd(i,2) = velocity*sin(x(i,3));
    xd(i,3) = velocity*tan(delta(i)/robot_length);
    
    % State update
    x(i+1,1) = x(i,1)+dt*xd(i,1);
    x(i+1,2) = x(i,2)+dt*xd(i,2);
    x(i+1,3) = x(i,3)+dt*xd(i,3);
    
    % angle wrap the heading
    x(i+1,3) = angleWrap(x(i+1,3));
    
 
    % Check if we have travelled the distance of the line segment. 
    % If we have, then get the next point
    if (next_point == 1)
        traj_point_counter = traj_point_counter+1;
        if (traj_point_counter == length(traj_points(:,1)))
            break;
        end
    end
    plot(x(1:i,1),x(1:i,2),'bo');
    rectangle('Position',[traj_points(1) traj_points(2) box_size(1) box_size(2)])
    pause(0.01)
end

%% Plotting

% Plotting the trajectory of the vehicle
figure(1);clf; hold on;
plot(x(1:i,1),x(1:i,2),'b-');

for t=1:30:i
      drawbox(x(t,1),x(t,2),x(t,3),.5,1);
end
xlabel('x (m)')
ylabel('y (m)')
axis equal

% Phase portrait
[crosstrack,heading] = meshgrid(-10:.5:10,-3:.2:3); % Create a grid over values of the crosstrack error and heading error
delta = max(-delta_max,min(delta_max,heading+atan2(k*crosstrack,velocity)));  % Calculate steering angle at each point
ed = velocity*sin(heading-delta); % Find crosstrack derivative
psid = -(velocity*sin(delta))/(robot_length); % Find heading derivative

psibplus = -atan2(k*crosstrack(1,:),velocity)+delta_max; % Find border of max region
psibminus = -atan2(k*crosstrack(1,:),velocity)-delta_max; % Find border of min region

figure(2);clf; hold on;
quiver(crosstrack,heading, ed, psid)
plot(crosstrack(1,:),psibplus,'r', 'LineWidth',2);
plot(crosstrack(1,:),psibminus,'r', 'LineWidth',2);
axis([-10 10 -3 3])
xlabel('e (m)')
ylabel('\psi (rad)')




