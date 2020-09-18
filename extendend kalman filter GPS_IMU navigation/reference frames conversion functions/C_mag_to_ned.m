function out = C_mag_to_ned( mag_declination_deg )
% function to change magnetic variation/declination from the magnetic frame north
% to north-east-down north

mag_declination_rad = pi/180*mag_declination_deg;                          % change magnetic declination from degrees to radians


out = [ cos(-mag_declination_rad)  sin(-mag_declination_rad)   0; ...
    
        -sin(-mag_declination_rad)  cos(-mag_declination_rad)   0; ...
             
        0                           0                           1];

end