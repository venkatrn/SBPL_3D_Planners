function path = generateDynamicObstacle(map, dynObsRadius, minPath, robotX, robotY)
map(robotY,robotX) = 1;
paddedMap = imdilate(im2bw(map, 0.5), strel('disk',dynObsRadius));
[startX, startY] = generateRandomValidPoint(paddedMap);
map(robotY,robotX) = 0;
paddedMap = imdilate(im2bw(map, 0.5), strel('disk',dynObsRadius));
path = [];
while true
    [goalX, goalY] = generateRandomValidPoint(paddedMap);
    temp = a_star(double(paddedMap),[startX startY],[goalX goalY]);
    if(numel(temp) == 0)
      continue;
    end
    
    path = [path; temp];
    startX = goalX;
    startY = goalY;
    if(numel(path) >= minPath)
        break;
    end
end

%draw stuff path on inflated map
figure(2);
imagesc(paddedMap);
axis xy;
hold on;
plot(path(:,1),path(:,2),'-g');
hold off;
drawnow;

