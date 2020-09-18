% Extended Kalman Filter parameters initialization
clear variables, close all 

global g itr GPS_sampling_rate dt

itr = 1;                                      % number recursive iterations for the ekf algorithm
g = 9.81;                                     % gravity force
dt = 0.1;                                     % time between measurements
GPS_sampling_rate = 0.001;                    % GPS sampling rate (can be adjusted)

addpath ('gps_imu_measurements','linearization','navigation_data','reference_frames','support_functions');

load('navigation_trajectory_data.mat');       % load name of file with trajectory data to be estimated

gps_imu_sensor_measurements                   % generate GPS and INS sensor measurements from the loaded trajectory data
tidy_working_space;

