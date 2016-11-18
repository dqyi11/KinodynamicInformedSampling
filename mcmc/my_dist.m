%% Function for euclidean distance
function [d] = my_dist(x)
% Distance function 
    global x0;
    no_ele = size(x,1);
    d = sqrt(sum((x - repmat(x0, no_ele,1)).^2,2));
end