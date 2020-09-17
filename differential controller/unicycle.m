function out = unicycle(in)
    global b r
    j = [r/b -r/b;r/2 r/2];
    wr = in(1);
    wl = in(2);
    w = [wr;wl];
    u = j*w;        %u1 angular velocity and u2 linear velocities
    theta = in(3);  %taking the feedback q1 only
    g = [1 0;0 cos(theta);0 sin(theta)];
    dq = g*u;  
    out = dq;
end
