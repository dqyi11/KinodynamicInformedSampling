function [ results, per_in] = mcmc( cstfxn, x1, x2, x, max_steps, sigma, T_best )
    % Randomly choose the number of steps
    no_steps = max_steps;
    z = 1;
    results = [];
    no_accepted = 0;
    extra_steps = 0;
    for step = 1:no_steps
        % Randomly choose a direction or go with gradient 
        posx = [];
        posx = x + normrnd(0, sigma, size(x));
        posz = feval(cstfxn, x1, x2, posx);
        % Go there if it is high enough
        if( (z / posz) > rand() && posz < T_best )
            x = posx;
            z = posz;
            extra_steps = extra_steps + 1;
            if(mod(extra_steps,100) == 0)
                results = [results; [x,z]];
            end
            no_accepted = no_accepted + 1;
        end
    end
    per_in = no_accepted / no_steps;
end