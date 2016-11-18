%% Main variables
global x0 h x1 v1 x2 v2 a_max;
x1 = 0; v1=0; x2=1; v2=0; a_max=1;
num_dim = 2;
x0 = zeros(1,num_dim);
h = 0.001;
sigma = 0.05;
low_range = -2;
high_range = 2;
alpha = 0.01;
surf_step = 0.01;
no_epochs = 1000;
T_best = 5;
no_trials = 1;
last_of_run = [];
%% Running gradient descent to the level set 
% Generate a random starting point

for trial = 1:no_trials
    % Choose a random starting point
    start = (high_range - low_range).*rand(1,num_dim) + low_range;
    
    % Gradient descent to the level-curve
    results = grad_descent(no_epochs, start, alpha, T_best, @cal_min_time1);
    
    % Random walk
    % Parameters (may need to tune or randomize these)
    alpha = 0.1;
    max_steps = 2000;
    rand_step_prob = 0.2;
%     results = [results; random_walk([results(end,1), results(end,2)], max_steps, alpha)];
    results_after = results(end,1:end-1);
    results = [results; mcmc(results_after, max_steps, sigma, @cal_min_time1, T_best)];
    last_of_run = [last_of_run; results(end,:)];

    trial
end
%% Plot results
figure;
% Z = plot_surf(low_range:surf_step:high_range, low_range:surf_step:high_range, @cal_min_time3);
plot_kino_surf(low_range:surf_step:high_range, low_range:surf_step:high_range, @cal_min_time3, T);
plot3(results(:,1),results(:,2),results(:,3),'r');
% scatter3(results(:,1),results(:,2),results(:,3),'r');
hold off;

% figure;
% hold on;
% viscircles([0,0], 1);
% scatter(last_of_run(:,1), last_of_run(:,2));
% hold off;

% Print number in
% no_in = sum(last_of_run(:,end) < 1)
% percent_in = no_in / no_trials

% Find average distance
a = -1;
b = 1;
r = a + (b-a)*rand(no_trials, size(last_of_run, 2) - 1);
avg_dist_uniform = avg_distance_bw_points(r);
avg_dist_sampling = avg_distance_bw_points(last_of_run(:,1:2));
avg_dist_ratio = avg_dist_sampling / avg_dist_uniform