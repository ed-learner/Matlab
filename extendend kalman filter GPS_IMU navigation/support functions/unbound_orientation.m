function out = unbound_orientation(est_yaw, magnetometer_yaw)
        % This function ensures that the yaw orientation measurements are
        % not bounded by the 360 degrees/2pi radians relative to the estimated yaw state
        
        yaw_measurement_rad = magnetometer_yaw*pi/180;
        
        while (yaw_measurement_rad > est_yaw + pi)
               yaw_measurement_rad = yaw_measurement_rad - 2*pi;
        end
        
        while (yaw_measurement_rad < est_yaw - pi)
              yaw_measurement_rad = yaw_measurement_rad + 2*pi; 
        end
        
        out = yaw_measurement_rad;
end