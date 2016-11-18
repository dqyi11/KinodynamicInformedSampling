%% Function to calculate gradient descent
function [ results ] = grad_descent( epochs, x, alpha, T_best, fun )
    global h;
    results = [];
    for epoch = 1:epochs
        z = feval(fun, x);
        results = [results; [x,z]];
        if z <= T_best
            break
        end
        grad = my_grad(fun, x, h);
        x = x - alpha*grad;
    end
    results = [results; [x,z]];
end