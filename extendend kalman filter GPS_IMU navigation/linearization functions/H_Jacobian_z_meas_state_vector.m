function H_Jacobian_z_meas_state_vector
    
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


% calculate zhat symbolic measurement equation and linearized measurements H
zhat = z_hat_H_linearization(x);

% H = jacobian(zhat,x);)
for n=1:length(x)
    H(:,n) = diff(zhat,x(n)); % derivative of zhat with respect to nth state variable
end
 
  fprintf('zhat measurement equation and its linearization H');
  display(zhat);
  display(H);
end

