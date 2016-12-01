% This function calculates the maximum time given a set number of joints
% x = [x_1, x_1_dot,...,x_n,x_n_dot]
% @param x1 Initial state
% @param x2 Final state
% @param xi Intermediate state
function E = cal_energy(q1, q2, q_last)
    global T_best;
    T = getTime(q1,q2,q_last);
%      E = T + 1e4*sigmf(T-T_best,[10 0]);
%     E = log(T) + 1e4*sigmf(T-T_best,[10 0]);
    E = log(1+log(T)) + 1e4*sigmf(T-T_best,[20 0]);
end