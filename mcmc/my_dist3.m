%% Function for euclidean distance
function [d] = my_dist3(x1, x2, x)
% Distance function 
    no_ele = size(x,1);
    d = sqrt(sum((x - repmat(x1, no_ele,1)).^2,2)) + sqrt(sum((x - repmat(x2, no_ele,1)).^2,2));
end