function [path] = Astar_function (x_s, y_s, theta_s, x_g, y_g, vert, A)
    
    n_best = zeros(1,2);
    open(1,:) = vert(1,:);
    T(1,:) = vert(1,:);
    visited = zeros(size(vert,1),1);

    while ~isempty(open) && prod(n_best(1,1:2)~=[x_g y_g])
        i_best = find(open(:,4)==min(open(:,4)));
        n_best = open(i_best(1),:);
        temp = vert == n_best;
        [k, ~, ~] = find (prod(temp,2) == 1);
        row = A(k(1),:);
        i_adj = find(row==1);

        if ~isempty(i_adj)
            for i=1:size(i_adj,2)
                adj(i,:)=vert(i_adj(i),:);
                
                if visited(i_adj(i)) == 0
                    visited(i_adj(i))=1;
                    T(end+1,1:4) = adj(i,:);
                    T(end,5) = k(1);
                    open = [open; adj(i,:)];
                else
                    if sum(vert(i_adj(i),1:3) ~= [x_s y_s theta_s])==3
                        gN_best = n_best(1,4) - norm([x_g y_g] - n_best(1,1:2));
                        gN_i = adj(i,4) - norm([x_g y_g] - adj(i,1:2));
                        
                        if (gN_best + norm(adj(i,1:2)) - n_best(1,1:2))<gN_1
                            i_T = find(T == adj(i,:));
                            T(i_T,5) = k(1);
                            
                            if isempty(find(open==adj(i,:),1))
                                open = [open; adj(i,:)];
                            else
                                open(find(open==adj(i,:)),4)= Astar_FN (n_best(1:2), adj(i,1:2), [x_g y_g], n_best(4));
                            end
                            
                        end
                    end
                    
                end
            end
        end
        
        open(i_best(1),:) = [];
        adj = [];

    end

    path(1,:) = T(end,:);
    i = 2;
    while sum(temp(1,1:3) ~= [x_s y_s theta_s])>0
        temp = vert(path(i-1,5),:);
        temp_bool = T(:,1:4) == temp;
        [k, ~, ~] = find (prod(temp_bool,2) == 1);
        path(i,:)=T(k,:);
        i=i+1;
    end

    path(:,5)=[];
    path = flipud(path);
    
end