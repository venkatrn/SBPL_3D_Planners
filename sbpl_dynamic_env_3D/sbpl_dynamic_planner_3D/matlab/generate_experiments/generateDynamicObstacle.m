function path = generateDynamicObstacle(map, dynObsRadius, minPath, robotX, robotY, robotZ, fatFlag)
map(robotY,robotX,robotZ) = 1;
paddedMap = imdilate3D(map,dynObsRadius);
[startX, startY, startZ] = generateRandomValidPoint(paddedMap,fatFlag);
map(robotY,robotX,robotZ) = 0;
paddedMap = imdilate3D(map,dynObsRadius);
path = [];
while true
    [goalX, goalY, goalZ] = generateRandomValidPoint(paddedMap,fatFlag);
    temp = a_star3D(double(paddedMap),[startX startY startZ],[goalX goalY goalZ]);
    if(numel(temp) == 0)
        continue;
    end
    
    
    
    path = [path; temp];
    startX = goalX;
    startY = goalY;
    startZ = goalZ;
    if(numel(path) >= minPath)
        break;
    end
end

%draw stuff path on inflated map
figure(2);
%imagesc(paddedMap);
axis xy;
hold on;
plot3(path(:,1),path(:,2),path(:,3),'-g');
hold off;
drawnow;
end

function paddedMap = imdilate3D(map,dynObsRadius)
paddedMap = zeros(size(map));
for  i = 1:size(map,3)
    paddedMap(:,:,i) = imdilate(im2bw(map(:,:,i), 0.5), strel('disk',dynObsRadius));
end
end

