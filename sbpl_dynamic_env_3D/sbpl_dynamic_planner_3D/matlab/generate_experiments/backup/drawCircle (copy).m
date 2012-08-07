function [map, numDrawn] = drawCircle(map, minRad, maxRad)
numDrawn = 0;
radius = round(rand*(maxRad-minRad) + minRad);
x = round(rand*(size(map,1)-1)+1);
y = round(rand*(size(map,2)-1)+1);
for i=x-radius:x+radius
  for j=y-radius:y+radius
    if( (i>0) && (i<=size(map,1)) && (j>0) && (j<=size(map,2)) && (map(i,j)==0) && (sqrt((x-i)^2 + (y-j)^2)<=radius) )
      map(i,j) = 1;
      numDrawn = numDrawn + 1;
    end
  end
end
