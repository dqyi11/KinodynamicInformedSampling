%% Main variables
global h a_max;
a_max=1;
num_dim = 8;
x1 = zeros(num_dim,1)';
x2 = ones(num_dim,1)';
h = 0.001;
sigma = 0.5;
max_steps = 20000;
low_range = -10;
high_range = 10;
alpha = 1;
surf_step = 0.1;
no_epochs = 1000;
T_best = 5;
no_trials = 100000;
last_of_run = [];
desired_no_samples = 1000;
%% Running gradient descent to the level set 
% Generate a random starting point

results = cell(no_trials,1);
actual_trials = 0;
per_in_total = 0;
mcmc_all_results = [];
total_t1 = 0;
total_t2 = 0;
for trial = 1:inf
    % Choose a random starting point
    start = (high_range - low_range).*rand(1,num_dim) + low_range;
    
    tic;
    % Gradient descent to the level-curve
    results{trial} = grad_descent(@getTime, no_epochs, x1, x2, start, alpha, T_best);
    total_t1 = total_t1 + toc;
    
    % MCMC
    tic;
    results_after = results{trial}(end,1:end-1);
    [mcmc_results, per_in] = mcmc(@getTime, x1, x2, results_after, max_steps, sigma, T_best);
    mcmc_all_results = [mcmc_all_results; mcmc_results];
    total_t2 = total_t2 + toc;
    
    per_in_total = per_in_total + per_in;
    actual_trials = actual_trials + 1;
    results{trial} = [results{trial}; mcmc_results];
    last_of_run = [last_of_run; results(end,:)];

    if(size(mcmc_all_results,1) >= desired_no_samples)
        break
    end
end
fprintf('Number of Samples \n')
size(mcmc_all_results)
fprintf('Gradient Time Ran \n');
total_t1
fprintf('MCMC Time Ran \n');
total_t2
fprintf('Total Time \n');
total_t1 + total_t2
fprintf('MCMC Percentage In \n');
per_in_total / actual_trials
%% Rejection sampling
% rejection_examples = [];
% rej_trials = 0;
% no_sampled_in = 0;
% tic;
% for trial = 1:inf
%     x = (high_range - low_range).*rand(1,num_dim) + low_range;
%     if(getTime(x1, x2, x) <= T_best)
%         rejection_examples = [rejection_examples; x];
%         no_sampled_in = no_sampled_in + 1;
%     end
%     if(size(rejection_examples, 1) >= desired_no_samples)
%         break
%     end
%     rej_trials = rej_trials + 1;
% end
% fprintf('Rejection Sampling ');
% toc;
% fprintf('Rejection Percent In \n');
% no_sampled_in / rej_trials
%% Plot results
% figure;
% Z = plot_surf(low_range:surf_step:high_range, low_range:surf_step:high_range, @cal_min_time3);
% colors = [hsv(100); jet(100)];
% shuffled_rows = randi(size(colors,1), [no_trials, 1]);
% colors = colors(shuffled_rows, :);
% 
% global v1 v2;
% v1 = x1(2);
% v2 = x2(2);
% x1 = x1(1);
% x2 = x2(2);
% plot_kino_surf(low_range:surf_step:high_range, low_range:surf_step:high_range, @cal_min_time3, T_best);
% 
% % for trial = 1:no_trials
% %     plot3(results{trial}(:,1),results{trial}(:,2),results{trial}(:,3), 'Color', colors(trial,:));
% % end
% scatter3(mcmc_all_results(:,1),mcmc_all_results(:,2),mcmc_all_results(:,3),'r');
% hold off;

% figure;
% hold on;
% viscircles([0,0], 1);
% scatter(last_of_run(:,1), last_of_run(:,2));
% hold off;

% Print number in
% no_in = sum(last_of_run(:,end) < 1)
% percent_in = no_in / no_trials

% Find average distance
% a = -1;
% b = 1;
% r = a + (b-a)*rand(no_trials, size(last_of_run, 2) - 1);
% avg_dist_uniform = avg_distance_bw_points(r);
% avg_dist_sampling = avg_distance_bw_points(last_of_run(:,1:2));
% avg_dist_ratio = avg_dist_sampling / avg_dist_uniform