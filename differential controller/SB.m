% SB - Scaling Block is responsible for translating them into instantaneous velocity values
%wr and wl of the right and left wheels, respectively

function out = SB(in)
global b r

u1 = in(1);  u2 = in(2);
wr = (1/r)*(u2 + (b/2)*u1);
wl = (1/r)*(u2 - (b/2)*u1);
out = [wr,wl];
end