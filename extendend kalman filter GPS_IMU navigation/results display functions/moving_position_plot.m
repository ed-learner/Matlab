% script to show the position movement of the UAV (true position,sensor measured position and EKF estimated position)

close all
n = length(sensor_meas.time_s);
animation_speed = 1e-3;
north_pos   = navigation_data.ned_pos(:,1);
east_pos    = navigation_data.ned_pos(:,2);
down_pos    = navigation_data.ned_pos(:,3);

GPS_north_pos = sensor_meas.GPS_north_pos;
GPS_east_pos  = sensor_meas.GPS_east_pos;
GPS_down_pos  = sensor_meas.GPS_down_pos;

est_north_pos = estimated_state.north_pos;
est_east_pos  = estimated_state.east_pos;
est_down_pos  = estimated_state.down_pos;


 hFig1 = figure;
 hPlot1 = plot3(NaN,NaN,NaN);
 grid on
 xlabel('east [m]'); ylabel('north [m]'); zlabel('height [m]');
 title('UAV navigation trajectory position');
 
 hFig2 = figure;
 hPlot2 = plot3(NaN,NaN,NaN);
 grid on
 xlabel('east [m]'); ylabel('north [m]'); zlabel('height [m]');
 title('UAV sensor measured position');
 
 hFig3 = figure;
 hPlot3 = plot3(NaN,NaN,NaN);
 grid on
 xlabel('east [m]'); ylabel('north [m]'); zlabel('height [m]');
 title('UAV EKF estimated position');
 
 for i=1:n  % loop values can changed to show different sections of the position trajectory
     xdata = get(hPlot1,'XData');
     ydata = get(hPlot1,'YData');
     zdata = get(hPlot1,'ZData');
     set(hPlot1,'XData',[xdata north_pos(i,:)],'YData',[ydata  east_pos(i,:)],'ZData',[zdata down_pos(i,:)]);
     xdata = get(hPlot2,'XData');
     ydata = get(hPlot2,'YData');
     zdata = get(hPlot2,'ZData');
     set(hPlot2,'XData',[xdata GPS_north_pos(i,:)],'YData',[ydata GPS_east_pos(i,:) ],'ZData',[zdata GPS_down_pos(i,:)]);
     xdata = get(hPlot3,'XData');
     ydata = get(hPlot3,'YData');
     zdata = get(hPlot3,'ZData');
     set(hPlot3,'XData',[xdata est_north_pos(i,:)],'YData',[ydata est_east_pos(i,:) ],'ZData',[zdata est_down_pos(i,:)]);
     pause(animation_speed);
 end

  tidy_working_space;
 