%% Function to calculate gradient descent
function [ results ] = grad_descent( fun, epochs, x1, x2, x, alpha, T_best )
    global h;
    results = [];
    for epoch = 1:epochs
        z = feval(fun, x1, x2, x);
%         figure(10);
%         hold on;
%         plot(epoch,z,'r*');
        results = [results; [x,z]];
        if z <= T_best
            break
        end
        grad = my_grad(fun, x, x1, x2, h);
        x = x - alpha*grad;
    end
    results = [results; [x,z]];
end