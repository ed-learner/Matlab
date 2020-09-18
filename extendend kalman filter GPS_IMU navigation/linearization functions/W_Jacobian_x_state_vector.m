function [out] = W_Jacobian_x_state_vector
% Run parameter initialization first
global g

% state vectors
phi         = sym('phi');
theta       = sym('theta');
psi         = sym('psi');
north_pos   = sym('north_pos');
east_pos    = sym('east_pos');
down_pos    = sym('down_pos');
north_vel   = sym('north_vel');
east_vel    = sym('east_vel');
down_vel    = sym('down_vel');

x= [phi; theta; psi; north_pos; east_pos; down_pos; north_vel; east_vel; down_vel];

% gyroscope x_y_z measurements in body frame 
syms wx wy wz 
% gyroscope measurements bias
syms bwx bwy bwz 
% accelerometer x_y_z specific force measurements in body frame 
syms fx fy fz
% accelerometer measurements bias
syms bax bay baz

% process noise
syms phi_noise theta_noise psi_noise n_pos_noise e_pos_noise d_pos_noise n_vel_noise e_vel_noise d_vel_noise
proc_noise = [phi_noise; theta_noise; psi_noise; n_pos_noise; e_pos_noise; d_pos_noise; n_vel_noise; e_vel_noise; d_vel_noise];

% calculate W Jacobian matrix of partial derivatives of dynamic equation
% with respect to process noise

body_rate_axis_to_euler_angle_rates = C_body_axis_rate_to_euler_angle_rate(phi,theta); 
ned_to_body = C_ned_to_body(psi,theta,phi);
body_to_ned = transpose(ned_to_body);

xdot = [ (body_rate_axis_to_euler_angle_rates*([wx;wy;wz]-[bwx;bwy;bwz])) + [phi_noise; theta_noise; psi_noise] ; ...     % derivative of orientation (roll,pitch,yaw)
         [north_vel; east_vel; -down_vel] + [n_pos_noise; e_pos_noise; d_pos_noise]; ...                                  % derivative of position (north,east,down)
         (body_to_ned*([fx;fy;fz]-[bax;bay;baz])+[0;0;g]) + [n_vel_noise; e_vel_noise; d_vel_noise]; ...                  % derivative of velocity (north,pos,pos) 
        ];
% F = jacobian(xdot,x);)
for n = 1:length(x)
    W(:,n) = diff(xdot,proc_noise(n)); % Derivative of xdot with respect to nth process noise variable
end
    out = W; 
end
