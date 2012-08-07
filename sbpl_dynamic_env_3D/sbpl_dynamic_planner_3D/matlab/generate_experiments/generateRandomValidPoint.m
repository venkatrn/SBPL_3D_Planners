function [x,y,z] = generateRandomValidPoint(map,fatFlag)
while true
    y = round(rand*(size(map,1)-1)+1);
    x = round(rand*(size(map,2)-1)+1);
    if fatFlag == 0
        z = round(rand*(size(map,3)-1)+1);
    else
        z = 1;
    end
    if(map(y,x,z) < 1)
        return;
    end
end
end

