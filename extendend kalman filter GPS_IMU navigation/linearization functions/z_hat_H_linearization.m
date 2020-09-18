function [zhat, H] = z_hat_H_linearization(x)
% Run parameter initialization first

% state variables from state vector
phi          = x(1);  % orientation
theta        = x(2);
psi          = x(3);  
north_pos    = x(4);  % position
east_pos     = x(5);   
down_pos     = x(6);  
north_vel    = x(7);  % velocity
east_vel     = x(8);   
down_vel     = x(9);  

% measurements 
zhat = [[psi]; ...                              % yaw measurement
        [north_pos; east_pos ; down_pos]; ...   % position measurements (north, east, down)
        [north_vel; east_vel; down_vel]; ...    % velocity measurements (north, east, down)
       ];
% linearized measurements
%H pre_calculated with the function H_Jacobian_z_meas_vector.m for algorithm efficiency
H=[ [ 0, 0, 1, 0, 0, 0, 0, 0, 0]
    [ 0, 0, 0, 1, 0, 0, 0, 0, 0]
    [ 0, 0, 0, 0, 1, 0, 0, 0, 0]
    [ 0, 0, 0, 0, 0, 1, 0, 0, 0]
    [ 0, 0, 0, 0, 0, 0, 1, 0, 0]
    [ 0, 0, 0, 0, 0, 0, 0, 1, 0]
    [ 0, 0, 0, 0, 0, 0, 0, 0, 1]
   ];
end



