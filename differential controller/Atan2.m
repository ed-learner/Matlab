%% Atan2c function

function out = Atan2ctest(in1,in2,in3)

i=2;
phi(i-1)=in3;
PHI(i)= atan2(in1,in2);
PHI(i-1)=atan2(sin(phi(i-1)),cos(phi(i-1)));
D_PHI(i)= PHI(i)-PHI(i-1);
if D_PHI(i)>pi
    d_phi(i)= D_PHI(i) -2*pi;
elseif D_PHI(i)<-pi
    d_phi(i)= D_PHI(i) +2*pi;
else
    d_phi(i)= D_PHI(i);
%PHI(i)= PHI(i-1) + d_phi(i);
end
PHI(i)= phi(i-1) + d_phi(i); %MK
out=PHI(i);
end