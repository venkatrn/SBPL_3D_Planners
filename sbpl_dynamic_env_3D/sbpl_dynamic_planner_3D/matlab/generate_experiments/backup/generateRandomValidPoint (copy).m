function [x,y] = getRandomValidPoint(map)
while true
    y = round(rand*(size(map,1)-1)+1);
    x = round(rand*(size(map,2)-1)+1);
    if(map(y,x) < 1)
        return;
    end
end
end

