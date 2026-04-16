function theta = findSolution(th, collision)

theta = [];

for i = 1:length(th)
    for j = 2:4
         if (th(i,j) > 2.168 || th(i,j) < -2.168) || collision == 1
             fprintf("the solution is out of reach or a collision occured.")
         else
             theta = th;
         end
    end
end

    