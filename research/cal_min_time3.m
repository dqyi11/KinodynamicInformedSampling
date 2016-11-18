% returns the minimum time between 3 points ([x1,y1] -> [xi,yi] -> [x2,y2]) in the state space
% returns -1 if the solution does not exist
function T = cal_min_time3(x1, v1, x2, v2, xi, vi)

    T1 = cal_min_time2(x1, v1, xi, vi);
    T2 = cal_min_time2(xi, vi, x2, v2);
    if (T1 < 0 || T2 < 0)
        T = -1;
        return;
    end
    T = T1 + T2;
end