function W = slip_walker_at_apex(P, dx, y, E, foot_x)
% SLIP_WALKER_AT_APEX  Walker state at an apex in single support on leg 1.
%
%   dx      horizontal CoM position relative to the stance foot  [m]
%   y       CoM height                                         [m]
%   E       walker energy  E = 1/2 m v^2 + m g y + 1/2 k (l0 - l)^2   [J]
%           (at the apex the push-off is off, so the leg rest length is l0)
%   foot_x  stance-foot position (default 0)                   [m]
%
% Forward speed from the energy (Geyer 2006, App. A(b)):
%   xdot = sqrt( 2E/m - k/m (l0 - l)^2 - 2 g y ),  l = sqrt(dx^2 + y^2),  ydot = 0

if nargin < 5, foot_x = 0; end
l   = hypot(dx, y);
v2  = 2*E/P.m - P.k/P.m*(P.l0 - l)^2 - 2*P.g*y;
if v2 <= 0 || l > P.l0
    error('slip_walker_at_apex: no valid apex state for dx=%.3f, y=%.3f, E=%.1f', dx, y, E);
end
W.x  = foot_x + dx;
W.z  = y;
W.xd = sqrt(v2);
W.zd = 0;
W.contact   = [true false];    % stance on leg 1, leg 2 swinging
W.foot      = [foot_x NaN];
W.apexArmed = false;           % this apex is the start point, do not record it again
W.phi = 3*pi/2;                % CPG phase: leg 1 in mid-stance (Owaki 2017: stance pi..2pi)
end
