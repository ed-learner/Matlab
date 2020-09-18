function out = C_body_axis_rate_to_euler_angle_rate(phi,theta)
% function to change the orientation from the body axis rates to euler angle rates

  out  = [1      sin(phi)*tan(theta)    cos(phi)*tan(theta) ; ...
      
          0      cos(phi)               -sin(phi)           ; ...
          
          0      sin(phi)*sec(theta)    cos(phi)*sec(theta)  ];
      
end