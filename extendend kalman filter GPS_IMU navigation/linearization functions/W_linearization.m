function [W] = W_linearization
% W is the Jacobian matrix of partial derivatives of dynamic equation
% with respect to process noise

% W pre_calculated with the function W_Jacobian_x_state_vector.m for algorithm
% efficiency

 W = [[ 1, 0, 0, 0, 0, 0, 0, 0, 0]
      [ 0, 1, 0, 0, 0, 0, 0, 0, 0]
      [ 0, 0, 1, 0, 0, 0, 0, 0, 0]
      [ 0, 0, 0, 1, 0, 0, 0, 0, 0]
      [ 0, 0, 0, 0, 1, 0, 0, 0, 0]
      [ 0, 0, 0, 0, 0, 1, 0, 0, 0]
      [ 0, 0, 0, 0, 0, 0, 1, 0, 0]
      [ 0, 0, 0, 0, 0, 0, 0, 1, 0]
      [ 0, 0, 0, 0, 0, 0, 0, 0, 1]
     ];
  
end