% cstfxn, grad_cstfxn =  cost function and its gradient
% epsilon = time step for HMC integration
% L = no. of steps for integration
% q1, q2, q = x1, x2, x = start, goal, current_state (to start from)
% sigma = standard diviation of the normal distribution to sample momentum
% no_steps = no. of hmc steps to run
% T_best = cost of informed subset

% Ref : https://arxiv.org/pdf/1206.1901.pdf

function [ results, per_in] = hmc( cstfxn, grad_cstfxn, epsilon, L, q1, q2, q, sigma, no_steps,  T_best )
    global h; % step for taking the derivative
    results = [];
    per_in = 0;
    for i=1:no_steps
        q_last = q;
        p = normrnd(0, sigma, size(q));
        p_last = p;
        
        % Make a half step for momentum at the beginning
        p = p - epsilon * feval(grad_cstfxn, cstfxn, q, q1, q2, h)/2;
        % Alternate Full steps for q and p
        for j=1:L
            q = q + epsilon * p;
            if j ~= L; p = p - epsilon * feval(grad_cstfxn, cstfxn, q, q1, q2, h); end
            results = [results; q, feval(cstfxn, q1, q2, q)]; % Plot
%             hamiltonian trajectory
        end
        % Make a half step for momentum at the end
        p = p - epsilon * feval(grad_cstfxn, cstfxn, q, q1, q2, h)/2;
        % Negate the momentum at the end of the traj to make proposal
        % symmetric
        p = -p;
        % Evaluate potential and kinetic energies at start and end of traj
        U_last = feval(cstfxn, q1, q2, q_last);
        K_last = sum(p_last.^2)/2;
        U_proposed = feval(cstfxn, q1, q2, q);
        K_proposed = sum(p.^2)/2;
        
        % Accept or reject the state at the end of trajectory
        alpha = min(1,exp(U_last-U_proposed+K_last-K_proposed));
%         results = [results; q, feval(@getTime, q1, q2, q);];
        if (rand <= alpha)
            per_in = per_in + 1;
%             results = [results; q, feval(@getTime, q1, q2, q);];
        else
            q = q_last;
        end
        
    end
    per_in = per_in / no_steps;
end