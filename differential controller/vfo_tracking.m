%vfo control signals u1 and u2 are computed as an output of the vfo block

function out = vfo_tracking(in) % take the input from the trajectory xdot,ydot
global b r phif 

%% input from trajectory
xr = in(1); yr = in(2); phi = in(3); 
dx_t = in(4); dy_t = in(5);
ddx_t = in(6); ddy_t = in(7);
%% input from the feedback
dx = in(9); dy = in(10);
phi = in(11);
x = in(12); y = in(13);
%% constants
k = 1; k1 = 1; kp = 2;
%% main program
ep = [xr-x;yr-y];
dq = [dx_t;dy_t]; 

hp = kp*ep + dq;
h_2 = hp(1); h_3 = hp(2);

u2 = h_2*cos(phi) + h_3*sin(phi);
dhp = [(kp*[dx_t-u2*cos(phi)] + ddx_t);(kp*[dy_t-u2*sin(phi)] + ddy_t)]; 
dh_2 = dhp(1); dh_3 = dhp(2);

phi_a= Atan2((1*h_3),(1*h_2),phif);  % +1 is equal to sgn(k) here
phif = phi_a; 

e_a = phi_a - phi;

dphi_a =(dh_3 *h_2 - h_3 * dh_2)/(h_2*h_2 + h_3*h_3);

%% calculating the value of u1 and u2
u1 = k1*e_a + dphi_a;

%SB (Scaling block)
wr= (1/r)*(u2 + (b/2)*u1);
wl= (1/r)*(u2 - (b/2)*u1);
out=[wr;wl];
end


