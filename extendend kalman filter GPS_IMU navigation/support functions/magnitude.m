function  mag  = magnitude (A)
 % function to calculate the magnitude of 3D values (x-y-z)
 % important especiaaly in obtaining x-y-z velocities of the UAV as one combined velocity
  
  norm = sqrt(sum(A.^2,2));

  mag = norm;
end