function [cost] = Astar_FN (n0, n1, ng, cost_old)
    cost = cost_old + norm(n1-n0)+norm(ng-n1);
end