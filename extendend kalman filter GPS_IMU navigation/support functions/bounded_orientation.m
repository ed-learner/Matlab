function bounded_angle = bounded_orientation( angle )
% function to bound the orientation angle between +pi (180 degrees) and 
% -pi (-180 degrees) important to prevent orientation angle (yaw,pitch,roll)
% exceeding values of 360
  
  angle = angle + 180;
  
  angle_remainder = mod(angle,360);
  
  bounded_angle = angle_remainder - 180;
 
end

