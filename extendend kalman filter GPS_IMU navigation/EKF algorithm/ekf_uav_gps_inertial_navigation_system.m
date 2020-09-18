clc;
fprintf('Wait while the extended kalman filter algorithim for uav navigation is executing.....\n');

m = length(navigation_data.time_s);         % length of navigation data time
t = length(sensor_meas.time_s);             % length of sensor measurements time
n = 9;                                      % number of states to be estimated
global itr;                                 % number of algorithm iterations

% standard deviation of process noise (for process noise matrix Q) 
std_dev.orient_process_noise = 0.002;      % radians               
std_dev.pos_process_noise    = 0.00001;    % meters     
std_dev.vel_process_noise    = 2;          % m/s

% standard deviation of measurement noise (for measurement noise matrix R)
std_dev.mag_meas_noise  = 2.5;             % radians
std_dev.pos_meas_noise  = 2.0;             % meters 
std_dev.vel_meas_noise  = 1.0;             % m/s

% call linearization functions
xdot_A  = @x_dot_A_linearization;         % linearized dynamic state model
zhat_H  = @z_hat_H_linearization;         % linearized measurement state model 
W_k = W_linearization;                    % linearized process noise 
V_k = V_linearization;                    % linearized measurement noise 

tic; 
% initial state estimates X_0
euler_init = [-0.90 -0.90 -0.90];          % yaw,pitch,roll in radians estimates
pos_init   = [-0.15  0.90   110];          % north-east-down position in m
vel_init   = [-20.0 -4.00 -0.40];          % north-east-down velocity in m/s

X_0 = [ euler_init, pos_init, vel_init]';

% initial error covariance matrix P_0
P_orient_init = [0.52 0.52 0.52];                              
P_pos_init    = [100 100 100];         
P_vel_init    = [10  10  10 ];         
P_0 = diag([P_orient_init P_pos_init P_vel_init].^2);

% create space for estimated paramaters
estimated_state = [];
estimated_state.time_s    = zeros(m,1);
estimated_state.roll_deg  = zeros(m,1);
estimated_state.pitch_deg = zeros(m,1);
estimated_state.yaw_deg   = zeros(m,1);
estimated_state.north_pos = zeros(m,1);
estimated_state.east_pos  = zeros(m,1);
estimated_state.down_pos  = zeros(m,1);
estimated_state.vel_ned   = zeros(m,3);
estimated_state.xhat      = zeros(m,n);
estimated_state.P         = zeros(m,n);

x_k = X_0; % initialize state estimates  x_0
p_k = P_0; % initialize error covariance P_0 matrix

for i=1:t
    % obtain navigation parameters at ith time from the sensor measurements for ekf estimation
    GPS_east_pos  = sensor_meas.GPS_east_pos(i);
    GPS_north_pos = sensor_meas.GPS_north_pos(i);
    GPS_down_pos  = sensor_meas.GPS_down_pos(i);
    GPS_ned_vel   = sensor_meas.GPS_ned_vel(i,:)';         
    gyro_wb       = sensor_meas.gyro_wb(i,:)';
    gyro_bias     = sensor_meas.gyro_bias(i,:)';  
    accel_fb      = sensor_meas.accel_fb(i,:)';         
    accel_bias    = sensor_meas.accel_bias(i,:)';
    mag_orient    = sensor_meas.mag_orient(i);

    % continuous time state dynamics process noise covariance, Q.
    orient_proc_noise = std_dev.orient_process_noise*[1 1 1];
    pos_proc_noise    = std_dev.pos_process_noise*[1 1 1];
    vel_proc_noise    = std_dev.vel_process_noise*[1 1 1]; 
    Q = diag([orient_proc_noise, pos_proc_noise, vel_proc_noise].^2);    
    
    yaw_measurement_rad = unbound_orientation(x_k(3),mag_orient);   % ensure sensor yaw orientation measuments are not bound by 360 degrees relative to yaw orientation state
    meas_orient = yaw_measurement_rad;                              % magnetometer orientation measurements
    meas_pos    = [GPS_north_pos; GPS_east_pos; GPS_down_pos];      % gps position measurements in north_east_down frame
    meas_vel    = GPS_ned_vel;                                      % gps velocity measurements in north_east_down frame   
    z = [meas_orient; meas_pos; meas_vel];
       
    % measurement noise error covariance R
    orient_meas_noise = std_dev.mag_meas_noise;
    pos_meas_noise    = std_dev.pos_meas_noise*[1 1 1];
    vel_meas_noise    = std_dev.vel_meas_noise*[1 1 1]; 
    R = diag([orient_meas_noise, pos_meas_noise, vel_meas_noise].^2);    
    
    % Extended Kalman filter algorithm
    [x_k, p_k] = ekf_algorithim(xdot_A, x_k, p_k, Q, zhat_H, z, R, W_k, V_k, gyro_wb, accel_fb, gyro_bias, accel_bias);
    
    % store the estimated states for analysis at each ith time
    estimated_state.time_s(i,:)     = sensor_meas.time_s(i);
    estimated_state.roll_deg(i,:)   = x_k(1)*180/pi;            % estimated roll in degrees
    estimated_state.pitch_deg(i,:)  = x_k(2)*180/pi;            % estimated pitch in degrees
    estimated_state.yaw_deg(i,:)    = x_k(3)*180/pi;            % estimated yaw in degrees
    estimated_state.north_pos(i,:)  = x_k(4);                   % estimated north position in meters
    estimated_state.east_pos(i,:)   = x_k(5);                   % estimated east position in meters
    estimated_state.down_pos(i,:)   = x_k(6);                   % estimated down position (height) in meters 
    estimated_state.vel_ned(i,:)    = x_k(7:9);                 % estimated north_east_down_velocity

    % store all the estimated states x_k and the error error covariance matrix p
    estimated_state.xhat(i,:) = x_k';
    estimated_state.P(i,:)    = diag(p_k);   
end

fprintf('\n')
fprintf('Extended Kalman Filter performed for %d iterations.\n',itr);
fprintf('Run "plotresults.m" file to display results.\n');
fprintf('Run "moving_position_plot.m" file to display position animation.\n');
toc; % total ekf algorithm execution total time
tidy_working_space;




