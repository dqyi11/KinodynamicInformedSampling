%% Function to run random walk
function [ results ] = random_walk( x, max_steps, alpha )
    % Randomly choose the number of steps
    no_steps = round((max_steps).*rand(1,1) - 1);
    results = zeros(no_steps, 3);
    for step = 1:no_steps
        z = my_dist(x);
        results(step, :) = [x(1),x(2),z];
        % Randomly choose a direction
        rand_dir = 2*rand(1,2) - 1;
        x(1) = x(1) - alpha*rand_dir(1);
        x(2) = x(2) - alpha*rand_dir(2);
    end
end