function [optimal] = find_optimal(curr_th,acceptable_sol)
    optimal = [];
    diff = acceptable_sol - curr_th;
    dist = vecnorm(diff, 2, 2);
    [min_val, best_idx] = min(dist);
    optimal = acceptable_sol(best_idx,:);
end
