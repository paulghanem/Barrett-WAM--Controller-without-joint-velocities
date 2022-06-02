function [valid, q] = wam7_straightline_traj_gen(q0,tgoal, iterations, q3res)

dead = cell(0);
Q = Queue(q0,tgoal,[1 1 1]);

Visited = q0;
valid = 1;

T_start = fkWam7(q0);
Traj = ctraj(T_start,tgoal, iterations);

while Q.numcells>0
  
        [Q ~]= Q.sort();
        x = Q.getFirst();

        if x.Num == iterations
            q=x.minPath;
            return;
        end

        for a = 1:4
           trajLoc = x.Num; % position in trajectory of xprime
           vali = 0;
            switch(a)
                case 1
                    if(trajLoc < iterations)
                        [vali, xprime,~]= wam7ik_w_joint_limits(Traj(:,:,trajLoc+1),x.Pos);
                        trajLoc =trajLoc+1;
                    end

                case 2
                    if(trajLoc > 1)
                        [vali, xprime,~]= wam7ik_w_joint_limits(Traj(:,:,trajLoc-1),x.Pos);
                        trajLoc =trajLoc-1;
                    end

                case 3
                    qnear = x.Pos;
                    qnear(3)= qnear(3)+q3res;
                    [vali, xprime,~]= wam7ik_w_joint_limits(Traj(:,:,trajLoc),qnear);

                case 4
                    qnear = x.Pos;
                    qnear(3)= qnear(3)-q3res;
                    [vali, xprime,~]= wam7ik_w_joint_limits(Traj(:,:,trajLoc),qnear);
            end

            if(vali)
                if sum(ismember(Visited',xprime','rows'))==0
                    Visited = [Visited xprime];
                    Q = Q.insertNode(xprime,x,trajLoc);
                else
                    [node index] = Q.getNode(xprime);

                    if(index ==0)

                    else

                        if(x.minCostCome +1 < node.minCostCome)
                            Q.values{index}.minPath = [x.minPath xprime];
                            Q.values{index}.minCostCome = x.minCostCome +1;  
                        end
                    end

                end
            end
        end

        Q=Q.deleteNode(x);
        dead{length(dead)+1}=x;
end

valid = 0;
disp('No Path Found');

end

function node = getNode(dead,pos)
           for a = 1:length(dead)
              if sum(dead{a}.Pos==pos)
                  node = dead{a};
                  return;
              end
           end
end
