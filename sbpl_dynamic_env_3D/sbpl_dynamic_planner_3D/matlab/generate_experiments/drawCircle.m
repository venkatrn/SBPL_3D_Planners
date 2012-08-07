function [map, numDrawn] = drawCircle(map, minRad, maxRad)
numDrawn = 0;
radius = round(rand*(maxRad-minRad) + minRad);
x = round(rand*(size(map,1)-1)+1);
y = round(rand*(size(map,2)-1)+1);
z = round(rand*(size(map,3)-1)+1);
for i=x-radius:x+radius
    for j=y-radius:y+radius
        for k=z-radius:z+radius
            if( (i>0) && (i<=size(map,1)) && (j>0) && (j<=size(map,2)) && (k>0) && (k<=size(map,3)) && (map(i,j,k)==0) && (sqrt((x-i)^2 + (y-j)^2 + (z-k)^2)<=radius) )
                map(i,j,k) = 1;
                numDrawn = numDrawn + 1;
            end
        end
    end
end
