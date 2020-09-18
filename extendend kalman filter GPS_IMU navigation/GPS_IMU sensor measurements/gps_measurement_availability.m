function out= gps_measurement_availability(x, GPS_x_pos, GPS_y_pos, GPS_z_pos, GPS_v_ned)
% GPS measurements available at a rate of 1/GPS_sampling_rate, interpl is used to determine GPS valid times

global GPS_sampling_rate

v = 1:length(x);
xq = x(1):GPS_sampling_rate:x(end);

valid_GPS_time = interp1(x, v, xq,'nearest');                                       % linear interpolation to obatain sensor times at which measurements are available

GPS_meas_available = false(size(x));                                                % Initialze GPS_meas_available to all "false" values.
GPS_meas_available (valid_GPS_time) = true;                                         % GPS_meas_available "true" at GPS update times.

GPS_y_pos(~GPS_meas_available)   = 0;                                               % Set GPS values to 0 at times when measurements are not available
GPS_x_pos(~GPS_meas_available)   = 0;  
GPS_z_pos(~GPS_meas_available)   = 0; 
GPS_v_ned(~GPS_meas_available,:) = 0; 

out = [GPS_x_pos, GPS_y_pos, GPS_z_pos, GPS_v_ned];
end