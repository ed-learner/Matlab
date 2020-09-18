function A_Jacobian_x_state_vector
% Run parameter_initialization.m first 

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

x = [phi; theta; psi; north_pos; east_pos; down_pos; north_vel; east_vel; down_vel];

% gyroscope x_y_z measurements in body axis frame 
syms wx wy wz 
gyro_wb = [wx; wy; wz];

% gyroscope measurements bias
syms bwx bwy bwz
gyro_bias = [bwx; bwy;bwz];

% accelerometer x_y_z specific force measurements in body axis frame 
syms fx fy fz
accel_fb = [fx; fy; fz];

% accelerometer measurements bias
syms bax bay baz
accel_bias=[bax; bay; baz];

% calculate xdot UAV dynamic equation and linearized dynamic equation A
xdot = x_dot_A_linearization(x, gyro_wb, accel_fb, gyro_bias, accel_bias);

% F = jacobian(xdot,x);)
for n=1:length(x)
    A(:,n) = diff(xdot,x(n)); % Derivative of xdot with respect to nth state variable
end
  fprintf('Xdot dynamic equation and its linearization A');
  
  display(xdot);
  display(A);

end
