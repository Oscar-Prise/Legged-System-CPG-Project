function goal_table = make_A2_goals(mode, mp)
%MAKE_A2_GOALS Generate desired states for Assignment 2.
%
% Each row:
% [x_des, dx_des, y_des, dy_des, psi]
%
% x_des, y_des, dx_des, dy_des are expressed in the
% local walking frame. psi is the orientation of that
% frame in the world frame.

w = sqrt(mp.g / mp.z0);
C = cosh(w * mp.T_st);
S = sinh(w * mp.T_st);

switch lower(mode)
    case 'left_quarter'
        % 10 steps through exactly 90 degrees.
        % Effective radius ~ 0.5 / (pi/20) = 3.18 m.
        N = 10;
        dpsi = pi / (2*N);
        dpsi_seq = dpsi * ones(N,1);

    case 'right_quarter'
        N = 10;
        dpsi = -pi / (2*N);
        dpsi_seq = dpsi * ones(N,1);

    case 'figure8'
        % Tighter loops are used so the complete figure eight
        % fits inside the 10 m x 10 m demo area.
        Nstraight = 2;
        Nloop = 28;
        dpsi_loop = 2*pi / Nloop;

        dpsi_seq = [ ...
            zeros(Nstraight,1); ...
            dpsi_loop  * ones(Nloop,1); ...
           -dpsi_loop  * ones(Nloop,1); ...
            zeros(Nstraight,1)];

    otherwise
        error('Unknown walking pattern: %s', mode);
end

psi = cumsum(dpsi_seq);
N = length(psi);
goal_table = zeros(N,5);

for k = 1:N
    x_des  = mp.lstep / 2;
    dx_des = mp.vtgt;

    % Original m07 starts with RIGHT foot in stance
    % and LEFT foot in swing.
    if mod(k,2) == 1
        y_des = -mp.wstep / 2;
    else
        y_des =  mp.wstep / 2;
    end

    dy_des = y_des * w * (C - 1) / S;
    goal_table(k,:) = [x_des, dx_des, y_des, dy_des, psi(k)];
end

end
