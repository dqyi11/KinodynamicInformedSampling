%% Function to estimate the gradient of a function
function [ grad ] = my_grad( fun, x, h )
% h default = 0.001
    grad = zeros(size(x));
    for dim = 1:size(x,2)
        x_plus = x; x_plus(dim) = x_plus(dim) + h;
        x_min = x; x_min(dim) = x_min(dim) - h;
        grad(dim) = (fun(x_plus) - fun(x_min)) / (2*h);
    end
end