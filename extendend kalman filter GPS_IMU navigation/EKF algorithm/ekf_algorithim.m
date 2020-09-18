function [xhat_new, P_new] = ekf_algorithim(x_dot_A, xhat_k, P_k, Q_t, z_hat_H, z_k, R_k, W_k, V_k, gyro_meas, accel_meas, gyro_bias, accel_bias)

global itr dt    % number of algorithm iterations and time between measurements

    [xdot, A] = x_dot_A(xhat_k, gyro_meas, accel_meas, gyro_bias, accel_bias);    % obtain xdot and A (dynamic states and jacobian A)
    n=9;                                                % number of states for the discretization process
    
    F   = [-A Q_t; zeros(n,n) A']*dt;                   % discretize continous data for implementation using Van Loan method
    G   = expm(F); 
    A_k = G(n+1:2*n, n+1:2*n)';
    Q_k = A_k*G(1:n, n+1:2*n);
    
 for i=1:itr
    % Time update step
    xhat_k = xhat_k + xdot*dt;                          % project the state ahead

    P_k = A_k*P_k*A_k' + W_k*Q_k*W_k';                  % project the error covariance ahead

    % Measurmement update step

    [zhat, H] = z_hat_H(xhat_k);                        % obtain zhat and H (measured states and jacobian h)

    K = (P_k*H')/(H*P_k*H'+V_k*R_k*V_k');               % compute the kalman gain

    xhat_k = xhat_k + K*(z_k-zhat);                     % update estimate zhat with measurements z_k    

    P_k = (eye(length(xhat_k))-K*H)*P_k;                % update the error covarince
 
    % Recursive 
    xhat_new = xhat_k;                                  % xhat(k+1)
    P_new    = P_k;                                     % P(k+1)
end

end

