function [ V ] = V_linearization
% V is Jacobian matrix of partial derivatives of measurement equation
% with respect to measurement noise

%V is pre_calculated with the function V_Jacobian_z_meas_vector.m for algorithm
%efficiency

 V = [[ 1, 0, 0, 0, 0, 0, 0]
      [ 0, 1, 0, 0, 0, 0, 0]
      [ 0, 0, 1, 0, 0, 0, 0]
      [ 0, 0, 0, 1, 0, 0, 0]
      [ 0, 0, 0, 0, 1, 0, 0]
      [ 0, 0, 0, 0, 0, 1, 0]
      [ 0, 0, 0, 0, 0, 0, 1]
     ];
end