function [ results ] = mcmc( x, max_steps, sigma, cstfxn, T_best )
    global h
    % Randomly choose the number of steps
%     no_steps = round((max_steps).*rand(1,1) - 1);
    no_steps = max_steps;
    z = 1;
    results = [];
    no_accepted = 0;
    for step = 1:no_steps
        % Randomly choose a direction or go with gradient
%         if( rand() > rand_step_prob )
%             rand_dir = 2*rand(1,size(x,2)) - 1;
%             posx = [];
%             posx = x - sigma*rand_dir;
%             posz = my_dist(posx);
%         else
%             grad = my_grad(@my_dist, x, h);
%             posx = [];
%             posx = x - sigma*grad;
%             posz = my_dist(posx);
%         end
%         
        posx = [];
        posx = x + normrnd(0, sigma, size(x));
        posz = feval(cstfxn, posx);
        % Go there if it is high enough
        if( (z / posz) > rand() && posz < T_best )
            x = posx;
            z = posz;
            results = [results; [x,z]];
            no_accepted = no_accepted + 1;
        else
            fprintf('Rejected\n');
        end
    end
    fprintf('Percent accepted\n');
    no_accepted / no_steps
end