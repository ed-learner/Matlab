function [x_dot, A] = x_dot_A_linearization(x, gyro_wb, accel_fb, gyro_bias, accel_bias)

global g;

phi          = x(1);  % orientation
theta        = x(2);
psi          = x(3);  
north_pos    = x(4);  % position
east_pos     = x(5);   
down_pos     = x(6);  
north_vel    = x(7);  % velocity
east_vel     = x(8);   
down_vel     = x(9);  

% angular rate and bias measurement in body frame from gyroscopes in radians per second
wx = gyro_wb(1); 
wy = gyro_wb(2);
wz = gyro_wb(3);

bwx = gyro_bias(1);
bwy = gyro_bias(2);
bwz = gyro_bias(3);

% specific force and bias measurements in body frame from accelerometers in meters/seconds^2
fx = accel_fb(1); 
fy = accel_fb(2);
fz = accel_fb(3);

bax = accel_bias(1);
bay = accel_bias(2);
baz = accel_bias(3);

body_rate_axis_to_euler_angle_rates = C_body_axis_rate_to_euler_angle_rate(phi,theta); 
ned_to_body = C_ned_to_body(psi,theta,phi);

body_to_ned = transpose(ned_to_body);

%xdot = dx/dt UAV dynamic equation
x_dot = [ [body_rate_axis_to_euler_angle_rates*([wx;wy;wz]-[bwx;bwy;bwz])]; ...  % derivative of orientation (roll,pitch,yaw)
          [north_vel; east_vel; -down_vel]; ...                                  % derivative of position (north,east,down)
          [body_to_ned*([fx;fy;fz]-[bax;bay;baz])+[0;0;g]]; ...                  % derivative of velocity (north,pos,pos) 
        ];

% linearized dynamic equation 
%A pre_calculated with the function A_Jacobian_x_state_vector.m for algorithm efficiency
A =[[                                                                 sin(phi)*tan(theta)*(bwz - wz) - cos(phi)*tan(theta)*(bwy - wy),                                  - cos(phi)*(bwz - wz)*(tan(theta)^2 + 1) - sin(phi)*(bwy - wy)*(tan(theta)^2 + 1),                                                                                                                                                              0, 0, 0, 0, 0, 0,  0]
    [                                                                                       cos(phi)*(bwz - wz) + sin(phi)*(bwy - wy),                                                                                                                  0,                                                                                                                                                              0, 0, 0, 0, 0, 0,  0]
    [                                                             (sin(phi)*(bwz - wz))/cos(theta) - (cos(phi)*(bwy - wy))/cos(theta),                    - (cos(phi)*sin(theta)*(bwz - wz))/cos(theta)^2 - (sin(phi)*sin(theta)*(bwy - wy))/cos(theta)^2,                                                                                                                                                              0, 0, 0, 0, 0, 0,  0]
    [                                                                                                                               0,                                                                                                                  0,                                                                                                                                                              0, 0, 0, 0, 1, 0,  0]
    [                                                                                                                               0,                                                                                                                  0,                                                                                                                                                              0, 0, 0, 0, 0, 1,  0]
    [                                                                                                                               0,                                                                                                                  0,                                                                                                                                                              0, 0, 0, 0, 0, 0, -1]
    [ - (sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))*(bay - fy) - (cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta))*(baz - fz), cos(psi)*sin(theta)*(bax - fx) - cos(phi)*cos(psi)*cos(theta)*(baz - fz) - cos(psi)*cos(theta)*sin(phi)*(bay - fy), (cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta))*(bay - fy) - (cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta))*(baz - fz) + cos(theta)*sin(psi)*(bax - fx), 0, 0, 0, 0, 0,  0]
    [   (cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta))*(bay - fy) + (cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta))*(baz - fz), sin(psi)*sin(theta)*(bax - fx) - cos(phi)*cos(theta)*sin(psi)*(baz - fz) - cos(theta)*sin(phi)*sin(psi)*(bay - fy), (cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta))*(bay - fy) - (sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))*(baz - fz) - cos(psi)*cos(theta)*(bax - fx), 0, 0, 0, 0, 0,  0]
    [                                                                 cos(theta)*sin(phi)*(baz - fz) - cos(phi)*cos(theta)*(bay - fy),                            cos(theta)*(bax - fx) + cos(phi)*sin(theta)*(baz - fz) + sin(phi)*sin(theta)*(bay - fy),                                                                                                                                                              0, 0, 0, 0, 0, 0,  0]
   ];

end



