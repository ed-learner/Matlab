function out =trajectory(in)
    % r-radius =2
    %xo and yo-trajectory origin
    %wx, wy-scaling for figure eight shape
    
    r = 2; xo = 0; yo = 0;
    wx = 0.5; wy =1;
    
    theta = 0.4*in(1);
    
    %% trajectory for figure eight shape

%    xr = r*cos(wx*theta);
%    yr = r*sin(wy*theta);
%    dxr = -r*wx*0.4*sin(wx*theta);
%    dyr = r*wy*0.4*cos(wy*theta);
% 
%    thr = atan2(dyr,dxr);
%    dqr = [dxr; dyr; thr];
%    ddxr = -r*wx*wx*0.16*cos(wx*theta);
%    ddyr = -r*wy*wy*0.16*sin(wy*theta);
   
    %% trajectory for circle
    xr = r*cos(theta) + xo;   
    yr = r*sin(theta) + yo;
    dxr = -r*0.4*sin(theta);
    dyr = r*0.4*cos(theta);
    ddxr = -r*0.16*cos(theta);
    ddyr = -r*0.16*sin(theta);
    thr = atan2(dyr,dxr);
 
%%  output
    out = [xr;yr;thr;dxr;dyr;ddxr;ddyr]; 
end
