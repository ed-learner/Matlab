function [out] = V_Jacobian_z_meas_vector
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

x = [psi; north_pos; east_pos; down_pos; north_vel; east_vel; down_vel];

% measurement noise (noise = noise + bias)
syms phi_noise theta_noise psi_noise n_pos_noise e_pos_noise d_pos_noise n_vel_noise e_vel_noise d_vel_noise
meas_noise = [psi_noise; n_pos_noise; e_pos_noise; d_pos_noise; n_vel_noise; e_vel_noise; d_vel_noise];

% calculate V Jacobian matrix of partial derivatives of measurement equation
% with respect to measurement noise 
   zhat = [[psi] + psi_noise; ...                                                             % yaw measurement
           [north_pos; east_pos ; down_pos] + [n_pos_noise; e_pos_noise; d_pos_noise]; ...    % position measurements (north, east, down)
           [north_vel; east_vel; down_vel]  + [n_vel_noise; e_vel_noise; d_vel_noise]; ...    % velocity measurements (north, east, down)
         ];

% V = jacobian(xdot,measurement noise);)
for n = 1:length(x)
    V(:,n) = diff(zhat,meas_noise(n)); % Derivative of xdot with respect to nth process noise variable
end

  out = V;
end
