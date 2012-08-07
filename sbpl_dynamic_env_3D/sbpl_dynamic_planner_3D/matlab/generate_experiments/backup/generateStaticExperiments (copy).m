function generateExperiments(numTests, indoor)
rand('twister',sum(100*clock))

%numTests = 50;

%length of each side of the map
side = 500;

%environment parameters
res = 0.025;

%INDOOR PARAMETERS
%hall parameters
hallWidth = 9;
minHalls = 4;
maxHalls = 10;

%room parameters
minRoomWidth = 3*hallWidth;
maxRoomWidth = 10*hallWidth;
pRoom = 0.4;

%OUTDOOR PARAMETERS
mapDensity = 0.2;
minObsCellRadius = round(3);
maxObsCellRadius = round(50);

!mkdir tests
for i=1:numTests
  fprintf('\ngenerating test %d\n',i);
  
  if(indoor)
    map = ones(side,side);
    %generate horizontal halls
    numHalls = round(rand*(maxHalls-minHalls)+minHalls);
    step = side/numHalls;
    for j=1:numHalls
      pos = round((j-1)*step + rand*(step-hallWidth)+1);
      map(pos:pos+hallWidth-1,:) = 0;
    end

    %generate vertical halls
    numHalls = round(rand*(maxHalls-minHalls)+minHalls);
    step = side/numHalls;
    for j=1:numHalls
      pos = round((j-1)*step + rand*(step-hallWidth)+1);
      map(:,pos:pos+hallWidth-1) = 0;
    end

    blobs = regionprops(bwlabel(map),'BoundingBox');

    %generate rooms
    for j=1:size(blobs,1)
      minX = ceil(blobs(j).BoundingBox(1))+1;
      maxX = floor(blobs(j).BoundingBox(1) + blobs(j).BoundingBox(3))-1;
      minY = ceil(blobs(j).BoundingBox(2))+1;
      maxY = floor(blobs(j).BoundingBox(2) + blobs(j).BoundingBox(4))-1;
      width = maxX-minX+1;
      height = maxY-minY+1;

      if(width < minRoomWidth || height < minRoomWidth || rand > pRoom)
        continue;
      end

      sideRoomX = round(rand*(min(maxRoomWidth,width)-minRoomWidth)+minRoomWidth);
      sideRoomY = round(rand*(min(maxRoomWidth,height)-minRoomWidth)+minRoomWidth);

      posX = round(rand*(width-sideRoomX)+minX);
      posY = round(rand*(height-sideRoomY)+minY);
      map(posY:posY+sideRoomY-1,posX:posX+sideRoomX-1) = 0;

      %left hall
      map(round(posY+sideRoomY/2-hallWidth/2):round(posY+sideRoomY/2-hallWidth/2)+hallWidth-1, minX-1:posX) = 0;

      %right hall
      map(round(posY+sideRoomY/2-hallWidth/2):round(posY+sideRoomY/2-hallWidth/2)+hallWidth-1, posX+sideRoomX-1:maxX+1) = 0;

      %bottom hall
      map(minY-1:posY, round(posX+sideRoomX/2-hallWidth/2):round(posX+sideRoomX/2-hallWidth/2)+hallWidth-1) = 0;

      %top hall
      map(posY+sideRoomY-1:maxY+1, round(posX+sideRoomX/2-hallWidth/2):round(posX+sideRoomX/2-hallWidth/2)+hallWidth-1) = 0;
      
    end
  else
    %for outdoor map generate random circular static obstacles
    map = zeros(side,side);
    numFilled = 0;
    while(numFilled/numel(map) < mapDensity)
      [map, temp] = drawCircle(map, minObsCellRadius, maxObsCellRadius);
      numFilled = numFilled + temp;
    end
  end

  figure(1);
  imagesc(map);
  axis xy;
  drawnow;

  %write environment to file
  filename = sprintf('tests/test%.4d.cfg',i-1);
  fout = fopen(filename, 'w');
  fprintf(fout, 'discretization(cells): %d %d\n', side, side);
  fprintf(fout, 'obsthresh: 1\n');
  fprintf(fout, 'cost_inscribed_thresh: 1\n');
  fprintf(fout, 'cost_possibly_circumscribed_thresh: 0\n');
  fprintf(fout, 'dynamic_obstacle_collision_cost_thresh: 254\n');
  fprintf(fout, 'cellsize(meters): %f\n', res);
  fprintf(fout, 'nominalvel(mpersecs): 1.0\n');
  fprintf(fout, 'timetoturn45degsinplace(secs): 2.0\n');

  %choose a random start and goal, such that a path exists between them
  while true
    [startX, startY] = generateRandomValidPoint(map);
    [goalX, goalY] = generateRandomValidPoint(map);
    path = a_star(map,[startX startY],[goalX goalY]);
    if(numel(path) > 0)
        break;
    end
  end

  fprintf(fout, 'start(meters,rads): %f %f %f\n', (startX-1)*res+res/2, (startY-1)*res+res/2, rand*2*pi);
  fprintf(fout, 'end(meters,rads): %f %f %f\n', (goalX-1)*res+res/2, (goalY-1)*res+res/2, rand*2*pi);
  fprintf(fout, 'environment:\n');
  %write map out to file
  out_map = map';
  out_map = out_map(:)';
  fprintf(fout,num2str(out_map));
  fclose(fout);

end
fprintf('Done Generating Experiments\n');
end


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
end


function [x,y] = getRandomValidPoint(map)
while true
    y = round(rand*(size(map,1)-1)+1);
    x = round(rand*(size(map,2)-1)+1);
    if(map(y,x) < 1)
        return;
    end
end
end

