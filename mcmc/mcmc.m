function [ results, per_in] = mcmc( x, max_steps, sigma, cstfxn, T_best )
    global h
    % Randomly choose the number of steps
    no_steps = max_steps;
    z = 1;
    results = [];
    no_accepted = 0;
    for step = 1:no_steps
        % Randomly choose a direction or go with gradient 
        posx = [];
        posx = x + normrnd(0, sigma, size(x));
        posz = feval(cstfxn, posx);
        % Go there if it is high enough
        if( (z / posz) > rand() && posz < T_best )
            x = posx;
            z = posz;
            results = [results; [x,z]];
            no_accepted = no_accepted + 1;
        end
    end
    per_in = no_accepted / no_steps
end