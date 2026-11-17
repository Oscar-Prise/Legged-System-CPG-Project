function [ic, mp] = sp_flight_start(mp, ic)
% SP_FLIGHT_START  Start state at a flight apex (running): no leg on the ground, both legs
% held at the angle of attack, leg 1 lands first (its take-off time is set earlier).
% Fills mp.init / mp.init_use in the layout of sp_logic_block's 'state' output.
for i = 1:mp.nW
    x = ic.pos(2*i-1);  z = ic.pos(2*i);  l0 = mp.l0(i);  a0 = mp.a0(i);
    S = zeros(1, 25);
    S(1:2)   = [0 0];                                % contact
    S(3:4)   = [0 0];                                % stance feet
    S(5:8)   = [l0 l0 l0 l0];                        % rest lengths lb, lamp
    S(9:10)  = (x + l0*cos(a0))*[1 1];               % swing feet at the angle of attack
    S(11:12) = (z - l0*sin(a0))*[1 1];
    S(13:16) = 0;                                    % foot velocities
    S(17:18) = x*[1 1];  S(19:20) = [0 0];           % take-off points / phases (foot model only)
    S(21:22) = [-1 -1];                              % last touchdown
    S(23:24) = [-1 -0.5];                            % leg 1 took off first -> lands first
    S(25)    = mp.phi0(i);
    mp.init(25*(i-1) + (1:25)) = S;
    mp.init_use(i) = 1;
end
mp.foot0 = [0 0];
end
