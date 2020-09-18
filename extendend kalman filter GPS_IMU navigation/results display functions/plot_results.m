% script to display results obtained in the ekf algorithm

close all
t = sensor_meas.time_s;

% position data
north_pos   = navigation_data.ned_pos(:,1);
east_pos    = navigation_data.ned_pos(:,2);
down_pos    = navigation_data.ned_pos(:,3);

GPS_north_pos = sensor_meas.GPS_north_pos;
GPS_east_pos  = sensor_meas.GPS_east_pos;
GPS_down_pos  = sensor_meas.GPS_down_pos;

est_north_pos = estimated_state.north_pos;
est_east_pos  = estimated_state.east_pos;
est_down_pos  = estimated_state.down_pos;

% orientation data
yaw   = navigation_data.orient_yaw_pitch_roll(:,1);
pitch = navigation_data.orient_yaw_pitch_roll(:,2);
roll  = navigation_data.orient_yaw_pitch_roll(:,3);

yaw_meas = sensor_meas.mag_orient;

est_yaw   = estimated_state.yaw_deg;
est_pitch = estimated_state.pitch_deg;
est_roll  = estimated_state.roll_deg;

% velocity data
ned_vel        = navigation_data.ned_vel;
sensor_ned_vel = sensor_meas.GPS_ned_vel;
est_ned_vel    = estimated_state.vel_ned;


%%%% Orientation plots
figure (1);  % navigation orientation
yaw = bounded_orientation(yaw);

subplot(3,1,1); 
plot(t,yaw); 
title ('Trajectory Yaw');
xlabel('Time [s]');ylabel('yaw [deg]');
xticks(0:10:300);
grid on
subplot(3,1,2);
plot(t,pitch); 
title ('Trajectory Pitch');
xlabel('Time [s]');ylabel('pitch [deg]');
xticks(0:10:300);
ylim([-30 40]);
grid on

subplot(3,1,3);
plot(t,roll); 
title ('Trajectory Roll');
xlabel('Time [s]');ylabel('roll [deg]');
xticks(0:10:300);
grid on
ylim([-30 40]);
xlabel('Time [s]');

figure (2); % measurement orientation
yaw_meas = bounded_orientation(yaw_meas);
subplot(3,1,1);
plot(t,yaw_meas);
title ('Sensor Yaw');
xlabel('Time [s]');ylabel('yaw [deg]');
xticks(0:10:300);
grid on
xlabel('time [s]');

figure (3); % estimated orientation
est_yaw = bounded_orientation(est_yaw);
subplot(3,1,1);
plot(t,est_yaw); 
title ('EKF estimated Yaw');
xlabel('Time [s]');ylabel('yaw [deg]');
xticks(0:10:300);
grid on

subplot(3,1,2); 
plot(t,est_pitch);
title ('EKF estimated Pitch');
xlabel('Time [s]');ylabel('pitch [deg]');
xticks(0:10:300);
grid on
ylim([-30 40]);

subplot(3,1,3);
plot(t,est_roll); 
title ('EKF estimated Roll');
xlabel('Time [s]');ylabel('roll [deg]');
xticks(0:10:300);
grid on
ylim([-30 40]);
xlabel('time [s]');

%%% Position plots
figure (4); % navigation trajectory position
plot3(north_pos,east_pos,down_pos,'-b','LineWidth',0.25);                         % desired navigation position
hold on
plot3(north_pos(1),east_pos(1),down_pos(1),'*k','LineWidth',5);                   % start point
plot3(north_pos(end),east_pos(end),down_pos(end),'*b','LineWidth',5);             % end point
grid on
xlabel('east [m]'); ylabel('north [m]'); zlabel('height [m]');
title('Unmanned aerial vehicle navigation trajectory position')
legend('Pos.','Start','End','location','best')

figure (5); % sensor measured position 
plot3(GPS_north_pos,GPS_east_pos,GPS_down_pos,'-b','LineWidth',0.25);             % sensor measured position
hold on
plot3(GPS_north_pos(1),GPS_east_pos(1),GPS_down_pos(1),'*k','LineWidth',5);       % start point
plot3(GPS_north_pos(end),GPS_east_pos(end),GPS_down_pos(end),'*b','LineWidth',5); % end point
grid on
xlabel('east [m]'); ylabel('north [m]'); zlabel('height [m]');
title('Unmanned aerial vehicle sensor measurements position')
legend('Pos.','Start','End','location','best')

figure (6); % ekf estimated position 
plot3(est_north_pos,est_east_pos,est_down_pos,'-b','LineWidth',0.25);               % ekf estimated position 
hold on
plot3(est_north_pos(1),est_east_pos(1),est_down_pos(1),'*k','LineWidth',5);         % start point
plot3(est_north_pos(end),est_east_pos(end),est_down_pos(end),'*b','LineWidth',5);   % end point
grid on
xlabel('east [m]'); ylabel('north [m]'); zlabel('height [m]');
title('Unmanned aerial vehicle ekf estimated position')
legend('Pos.','Start','End','location','best')

%%% velocity magnitude plots
figure (7);
ned_vel = magnitude(ned_vel);
sensor_ned_vel = magnitude(sensor_ned_vel);
est_ned_vel = magnitude(est_ned_vel);

subplot(3,1,1);
plot(t,ned_vel);
grid on
ylim([0 30]);
xlabel('Time [s]');ylabel('speed [m/s]')
xticks(0:10:300);
title('Trajectory Velocity')

subplot(3,1,2)
plot(t, sensor_ned_vel);
grid on
ylim([0 30]);
xlabel('Time [s]');ylabel('speed [m/s]')
xticks(0:10:300);
title('Sensor measured Velocity')

subplot(3,1,3)
plot(t,est_ned_vel); 
grid on
ylim([0 30]);
xlabel('Time [s]');ylabel('speed [m/s]')
xticks(0:10:300);
title('EKF estimated Velocity')

%%% orientation error covariance plot
figure (8)
orient_error = (estimated_state.P(:,[1 2 3]));        
subplot(3,1,1)
plot(t,orient_error(:,1),'b','LineWidth',1);
grid on;
xlabel('Time [s]'); ylabel('yaw error')
xticks(0:10:300);
ylim([-0.5 0.5]) 
title('Orientation Error Covariance [P(T)]');

subplot(3,1,2)
plot(t,orient_error(:,2),'b','LineWidth',1);
grid on;
xlabel('Time [s]'); ylabel('pitch error')
xticks(0:10:300);
ylim([-0.5 0.5]); 

subplot(3,1,3)
plot(t,orient_error(:,3),'b','LineWidth',1);
grid on;
xlabel('Time [s]'); ylabel('roll error')
xticks(0:10:300);
ylim([-0.5 0.5]); 

%%% position error covariance plots
figure (9) 
pos_error = (estimated_state.P(:,[4 5 6]));        
subplot(3,1,1)
plot(t,pos_error(:,1),'b','LineWidth',1);
grid on;
xlabel('Time [s]'); ylabel('north pos error')
xticks(0:10:300);
ylim([0 2]) 
title('Position Error Covariance [P(T)]');

subplot(3,1,2)
plot(t,pos_error(:,2),'b','LineWidth',1);
grid on;
xlabel('Time [s]'); ylabel('east pos error')
xticks(0:10:300);
ylim([0 2]); 

subplot(3,1,3)
plot(t,pos_error(:,3),'b','LineWidth',1);
grid on;
xlabel('Time [s]'); ylabel('down pos error')
xticks(0:10:300);
ylim([0 2]); 

%%% velocity errors plots
figure (10)
vel_error = (estimated_state.P(:,[7 8 9]));        
subplot(3,1,1)
plot(t,vel_error(:,1),'b','LineWidth',1);
grid on;
xlabel('Time [s]'); ylabel('north vel error')
xticks(0:10:300);
ylim([0 1])
title('Velocity Error Covariance [P(T)]');

subplot(3,1,2)
plot(t,vel_error(:,2),'b','LineWidth',1);
grid on;
xlabel('Time [s]'); ylabel('east vel error')
xticks(0:10:300);
ylim([0 1]); 

subplot(3,1,3)
plot(t,vel_error(:,3),'b','LineWidth',1);
grid on;
xlabel('Time [s]'); ylabel('down vel error')
xticks(0:10:300);
ylim([0 1]); 

tidy_working_space;

