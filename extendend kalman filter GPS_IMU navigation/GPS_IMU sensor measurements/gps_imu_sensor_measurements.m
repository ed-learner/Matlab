global g; % gravitational force
n = size(navigation_data.time_s);

% initialize structure
sensor_meas = [ ];
sensor_meas.std_dev = [ ];
sensor_meas.biases  = [ ];
sensor_meas.noise   = [ ];
sensor_meas.time_s  = navigation_data.time_s;

% GPS position noise and bias
GPS_pos_noise_std_dev     = 4;        
GPS_pos_bias_std_dev      = 0.02;      

sensor_meas.biases.GPS_east_pos    = GPS_pos_bias_std_dev*randn(n);
sensor_meas.biases.GPS_north_pos   = GPS_pos_bias_std_dev*randn(n);
sensor_meas.biases.GPS_down_pos    = GPS_pos_bias_std_dev*randn(n);

sensor_meas.noise.GPS_east_pos    = GPS_pos_noise_std_dev*randn(n);
sensor_meas.noise.GPS_north_pos   = GPS_pos_noise_std_dev*randn(n);
sensor_meas.noise.GPS_down_pos    = GPS_pos_noise_std_dev*randn(n);

% GPS velocity
sensor_meas.GPS_north_pos = navigation_data.ned_pos(:,1) + sensor_meas.biases.GPS_north_pos + sensor_meas.noise.GPS_north_pos;
sensor_meas.GPS_east_pos  = navigation_data.ned_pos(:,2) + sensor_meas.biases.GPS_east_pos +  sensor_meas.noise.GPS_east_pos;
sensor_meas.GPS_down_pos  = navigation_data.ned_pos(:,3) + sensor_meas.biases.GPS_down_pos +  sensor_meas.noise.GPS_down_pos;

% GPS Velocity
GPS_vel_noise_std_dev     = 0.5;        % GPS velocity noise and bias, m/s
GPS_vel_bias_std_dev      = 0.01;     

sensor_meas.bias.GPS_vel_east    = GPS_vel_bias_std_dev*randn(n);          
sensor_meas.bias.GPS_vel_north   = GPS_vel_bias_std_dev*randn(n);
sensor_meas.bias.GPS_vel_down    = GPS_vel_bias_std_dev*randn(n);

sensor_meas.noise.GPS_vel_east    = GPS_vel_noise_std_dev*randn(n);        
sensor_meas.noise.GPS_vel_north   = GPS_vel_noise_std_dev*randn(n);
sensor_meas.noise.GPS_vel_down    = GPS_vel_noise_std_dev*randn(n);

GPS_vel_north = navigation_data.ned_vel(:,1) + sensor_meas.bias.GPS_vel_north + sensor_meas.noise.GPS_vel_north;
GPS_vel_east  = navigation_data.ned_vel(:,2) + sensor_meas.bias.GPS_vel_east  + sensor_meas.noise.GPS_vel_north;
GPS_vel_down  = navigation_data.ned_vel(:,3) + sensor_meas.bias.GPS_vel_down  + sensor_meas.noise.GPS_vel_north;

 sensor_meas.GPS_ned_vel = [GPS_vel_north, GPS_vel_east, GPS_vel_down];

% GPS measurements is available at 1/sampling rate
data = gps_measurement_availability(navigation_data.time_s,sensor_meas.GPS_east_pos,sensor_meas.GPS_north_pos,sensor_meas.GPS_down_pos,sensor_meas.GPS_ned_vel);
                   
sensor_meas.GPS_east_pos  = data(:,1); 
sensor_meas.GPS_north_pos = data(:,2); 
sensor_meas.GPS_down_pos  = data(:,3); 
sensor_meas.GPS_ned_vel   = data(:,4:6); 

% IMU Gyro Angular Rate (wb) Measurements, body coordinates 
% Gyroscope angular velocity with noise and bias, rad/s
gyro_noise_std_dev  = 0.01;     
gyro_bias_std_dev   = 1e-3;

gyro_bias_x = gyro_bias_std_dev*randn(n);
gyro_bias_y = gyro_bias_std_dev*randn(n);
gyro_bias_z = gyro_bias_std_dev*randn(n);

gyro_noise_x = gyro_noise_std_dev*randn(n);
gyro_noise_y = gyro_noise_std_dev*randn(n);
gyro_noise_z = gyro_noise_std_dev*randn(n);

gyro_wb_x = navigation_data.angular_vel(:,1) + gyro_bias_x + gyro_noise_x;
gyro_wb_y = navigation_data.angular_vel(:,2) + gyro_bias_y + gyro_noise_y;
gyro_wb_z = navigation_data.angular_vel(:,3) + gyro_bias_z + gyro_noise_z;

sensor_meas.gyro_wb   = [gyro_wb_x,gyro_wb_y,gyro_wb_z];
sensor_meas.gyro_bias = [gyro_bias_x,gyro_bias_y,gyro_bias_z];

% Accelerometer noise and bias, m/s^2
accel_noise_std_dev  = 0.1;     
accel_bias_std_dev   = 1e-3;  

accel_bias_x = accel_bias_std_dev*randn;
accel_bias_y = accel_bias_std_dev*randn;
accel_bias_z = accel_bias_std_dev*randn;

accel_noise_x = accel_noise_std_dev*randn;
accel_noise_y = accel_noise_std_dev*randn;
accel_noise_z = accel_noise_std_dev*randn;

dv_north = gradient(navigation_data.ned_vel(:,1),navigation_data.time_s);
dv_east  = gradient(navigation_data.ned_vel(:,2),navigation_data.time_s);
dv_down  = gradient(navigation_data.ned_vel(:,3),navigation_data.time_s);

dv_ned = [dv_north, dv_east, dv_down]; 

orient_rad = pi/180*navigation_data.orient_yaw_pitch_roll;
yaw_rad    = orient_rad(:,1);
pitch_rad  = orient_rad(:,2);
roll_rad   = orient_rad(:,3);

for i = 1:n
    ned_to_body = C_ned_to_body(yaw_rad(i), pitch_rad(i), roll_rad(i));    % convert from ned to body frame
    
    accel_fb = ned_to_body*dv_ned(i,:)' - ned_to_body*[0;0;g];             % acceleration specific force
   
    accel_x = accel_fb(1,1) + accel_bias_x + accel_noise_x;                % acceleration specific force with noise and bias
    accel_y = accel_fb(2,1) + accel_bias_y + accel_noise_y;
    accel_z = accel_fb(3,1) + accel_bias_z + accel_noise_z; 
    
    sensor_meas.accel_fb(i,:)   = [accel_x,accel_y,accel_z];
    sensor_meas.accel_bias(i,:) = [accel_bias_x,accel_bias_y,accel_bias_z];
end
 
% Magnetometer measurement
mag_variation     = 2*randn;
mag_noise_std_dev = 0.4;     
mag_bias_std_dev  = 0.5;    

mag_bias_x = mag_bias_std_dev*randn;
mag_bias_y = mag_bias_std_dev*randn;
mag_bias_z = mag_bias_std_dev*randn;

mag_noise_x = mag_noise_std_dev*randn;
mag_noise_y = mag_noise_std_dev*randn;
mag_noise_z = mag_noise_std_dev*randn;

for i=1:n
    ned_to_body   = C_ned_to_body(yaw_rad(i),pitch_rad(i),roll_rad(i)); 
    mag_to_ned = C_mag_to_ned(mag_variation); 
    mag_x_y_z = ned_to_body*mag_to_ned*[1;0;0];
    
    mag_x = mag_x_y_z(1,1) + mag_bias_x + mag_noise_x;
    mag_y = mag_x_y_z(2,1) + mag_bias_y + mag_noise_y;
    mag_z = mag_x_y_z(3,1) + mag_bias_z + mag_noise_z; 
 
    mag_x_y_z = [mag_x, mag_y, mag_z];
    mag_x_y_z = 180/pi*mag_x_y_z;  % convert the measurements to degrees from radians
    
    % Store x-y-z body axis magnetometer measurements i
    sensor_meas.mag_x_y_z(i,:) = mag_x_y_z ; 
    
    % Use x,y components of the magnetometer measuremets to make the
    % measurement used in the measurment model equation
    mag_orient(i,1) = atan2(-mag_x_y_z(2), mag_x_y_z(1));
    mag_orient(i,1) = bounded_orientation( mag_orient(i,1) );
    sensor_meas.mag_orient(i,1) = 180/pi*mag_orient(i,1); % convert the measurements to degrees from radians
end
