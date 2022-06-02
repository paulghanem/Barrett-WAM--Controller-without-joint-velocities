classdef Queue
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        values = cell(1);
        numcells = 0;
        numberInserted = 0;
        goal=[0,0];
    end
    
    methods
        function[obj] = Queue(q0,tg,valid_directions)
         obj.numberInserted = obj.numberInserted+1;
         obj.goal=tg;
         obj.values{1} = struct('Pos',q0,'Num',1,'minCostCome',0,'minPath',q0,'val',valid_directions,'vists',1);
         obj.numcells = obj.numcells + 1;
        end
        function obj = insertNode(obj,pos,current_node,TrajNum)
            obj.numcells = obj.numcells +1;
            obj.numberInserted = obj.numberInserted+1;
            obj.values{obj.numcells} = struct('Pos',pos,'Num',TrajNum,'minCostCome',current_node.minCostCome+1,'minPath',[current_node.minPath pos]);
        end
        function obj = deleteNode(obj,node)
            [n index]=getNode(obj,node.Pos);
            for a = index:obj.numcells-1
                obj.values{a}=obj.values{a+1};
            end
            obj.numcells = obj.numcells -1;
        end
        function [node index] = getNode(obj,pos)
           node=[];
           index = 0;
            for a = 1:obj.numcells
              if sum(obj.values{a}.Pos == pos) == 7
                  node = obj.values{a};
                  index = a;
                  return;
              end
           end
        end
        function dist = G(obj,node) 
            %Uncomment one of these two lines
            %dist = abs(obj.goal(1)-x) + abs(obj.goal(2)-y ); %18.1
            %dist = norm(obj.goal - x);  %18.2
            dist = 1000 - node.Num;
        end
        function [obj minG]= sort(obj)
           % Calculate G's element
           Gmatrix = zeros(obj.numcells,2);%pre allocation
            for a = 1:obj.numcells
                %loc = obj.values{a}.Pos;
                Gmatrix(a,:)=[obj.G(obj.values{a})+obj.values{a}.minCostCome,a];
            end
            Gmatrix = sortrows(Gmatrix,1);
            %Since only the first value of the queue is needed, the first
            %elemnent is pivoted
            element = Gmatrix(1,2);
            cell = obj.values{element};
            obj.values{element}=obj.values{1};
            obj.values{1}=cell;
            
            minG=Gmatrix(1,1);
        end
        function cell = getFirst(obj)
           cell = obj.values{1}; 
        end
        function cell = getRandom(obj)
           
           cell = obj.values{randi(obj.numcells)}; 
        end
    end
    
end

