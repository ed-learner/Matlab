%Time series for position from the unicycle kinematics
ang = txy.data(:,1);
a = txy.data(:,2); 
b = txy.data(:,3);

%Time series for position from the trajectory generator
c = txyr.data(:,1);
d = txyr.data(:,2);
angr = txyr.data(:,3);

%position and angle error (unicycle kinematics-trajectory generator)
xerr = a-c;
yerr = b-d;
angerr = angr-ang;

%unicycle kinematics and trajectory generator position plot
figure (1);
plot(a,b,'-.')
hold on; 
plot(c,d)
title('Position comparison of Unicycle and Trajectory Generator')
legend('Unicycle Position','Trajectory Generator Position')

%unicycle kinematics and trajectory generator angle plot
figure (2);
plot(tout,ang)
hold on;
plot(tout,angr)
title('Angle comparison of the Unicycle and Trajectory Generator')
legend('Unicycle angle','Trajectory Generator angle')

% x,y Postion error plot
figure (3);
plot(tout,yerr,'-.')
hold on
plot(tout,xerr,'')
title(' X and Y Position Error')
legend('Y Position Error','X Position Error')

% angle error plot
figure (4);
plot(tout,angerr)
title('Angle Error')
legend('Angle Error')