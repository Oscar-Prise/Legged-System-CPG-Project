function [goal, psi, step_count] = fcn(t, goal_table, T_st)
% Goal Scheduler MATLAB Function block for Simulink.

N = size(goal_table,1);

step_count = floor(t / T_st) + 1;

if step_count < 1
    step_count = 1;
elseif step_count > N
    step_count = N;
end

goal = goal_table(step_count,1:4).';
psi  = goal_table(step_count,5);

end
