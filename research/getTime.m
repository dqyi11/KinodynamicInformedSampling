% This function calculates the maximum time given a set number of joints
% x = [x_1, x_1_dot,...,x_n,x_n_dot]
% @param x1 Initial state
% @param x2 Final state
% @param xi Intermediate state
% @return T Maximum time
function T = getTime(x1, x2, xi)
    Ts = zeros(size(x1,2) / 2, 1);
    for i = 1:2:size(x1,2)
        Ts(i) = cal_min_time3(x1(i), x1(i+1), x2(i), x2(i+1),...
                              xi(i), xi(i+1));
    end
    T = max(Ts);
end
