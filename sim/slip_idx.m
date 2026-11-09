function I = slip_idx(nW)
% SLIP_IDX  Positions in the state vector for nW walkers.
%   y = [x1 z1 xd1 zd1, ..., xN zN xdN zdN,  xo, xod,  Ediss_arm,  phi_1..phi_N,  W_act,  Ediss_leg]
io = 4*nW;
I.xo   = io + 1;               % object position
I.xod  = io + 2;               % object speed
I.Ea   = io + 3;               % energy dissipated in the arm dampers
I.phi  = io + 3 + (1:nW);      % CPG phase of each walker
I.Wact = io + 3 + nW + 1;      % work done by the push-off actuators
I.El   = io + 3 + nW + 2;      % energy lost in the leg dampers (+ at take-off)
I.n    = io + 3 + nW + 2;
end
