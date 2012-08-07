function generateExperiments(numTests, numDynObs, indoor)
rand('twister',sum(100*clock))


% TODO: Discretization, start, end, environment

%numTests = 50;

%length of each side of the map
side = 300;
depth = 5;


%environment parameters
res = 0.1;
timeRes = 0.1;

%dynamic obstacle parameters
%numDynObs = 200;
fatRadius = 2;
thinRadius = 1;
pFat = 0.1;
minPath = 60;
maxPath = 80;

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
minObsCellRadius = round(1);
maxObsCellRadius = round(10);


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
        %for outdoor map generate random spherical static obstacles
        map = zeros(side,side,depth);
        
        numFilled = 0;
        while(numFilled/numel(map) < mapDensity)
            [map, temp] = drawCircle(map, minObsCellRadius, maxObsCellRadius);
            numFilled = numFilled + temp;
        end
    end
    
    %     figure(1);
    %     imagesc(map);
    %     axis xy;
    %     drawnow;
    
    %write environment to file
    filename = sprintf('tests/test%.4d.exp',i-1);
    fout = fopen(filename, 'w');
    % TODO : Actual z !
    fprintf(fout, 'discretization(cells): %d %d %d\n', side, side, depth);
    fprintf(fout, 'obsthresh: 1\n');
    fprintf(fout, 'cost_inscribed_thresh: 1\n');
    fprintf(fout, 'cost_possibly_circumscribed_thresh: 0\n');
    fprintf(fout, 'dynamic_obstacle_collision_cost_thresh: 254\n');
    fprintf(fout, 'cellsize(meters): %f\n', res);
    fprintf(fout, 'timeResolution(seconds): %f\n', timeRes);
    fprintf(fout, 'temporal_padding(seconds): %f\n', timeRes);
    fprintf(fout, 'nominalvel(mpersecs): 1.0\n');
    fprintf(fout, 'timetoturn45degsinplace(secs): 0.1\n');
    
    %choose a random start and goal, such that a path exists between them
    while true
        [startX, startY] = generateRandomValidPoint(map);
        [goalX, goalY] = generateRandomValidPoint(map);
        startZ = 1;
        goalZ = 4;
        path = a_star(map(:,:,1),[startX startY],[goalX goalY]);
        if(numel(path) > 0)
            break;
        end
    end
    
    %TODO: Include z !
    fprintf(fout, 'start(meters,rads): %f %f %f %f %f\n', (startX-1)*res+res/2, (startY-1)*res+res/2, (startZ-1)*res+res/2, rand*2*pi, 0.0);
    fprintf(fout, 'end(meters,rads): %f %f %f %f\n', (goalX-1)*res+res/2, (goalY-1)*res+res/2, (goalZ-1)*res+res/2, rand*2*pi);
    fprintf(fout, 'environment:\n');
    %write map out to file
    out_map = zeros(size(map));
    size(map)
    size(map,3)
    for m = 1:size(map,3)
        out_map(:,:,m) = map(:,:,m)';
    end
    out_map = out_map(:)';
    fprintf(fout,num2str(out_map));
    fclose(fout);
    
    
    %generate dynamic obstacle trajectories
    %write a seperate file for creating a dynamic obstacle trajectory that
    %uses BFS to find a reasonable trajectory through the map...
    filename = sprintf('tests/dynObs%.4d.dob',i-1);
    fout = fopen(filename, 'w');
    fprintf(fout, 'NumberOfDynamicObstacles: %d\n', numDynObs);
    for j=1:numDynObs
        fprintf('Generate dynamic obstacle %d\n',j);
        fprintf(fout, 'DynamicObstacleID: %d\n', j-1);
        
        if(rand <= pFat)
            %fat obstacle
            radius = fatRadius;
        else
            %thin obstacle
            radius = thinRadius;
        end
        
        fprintf(fout, 'ObstacleRadius: %f\n', radius*res);
        fprintf(fout, 'NumberOfTrajectories: 1\n');
        fprintf(fout, 'TrajectoryID: 0\n');
        fprintf(fout, 'TrajectoryProbability: 1.0\n');
        
        path = generateDynamicObstacle(map(:,:,1), radius, minPath, startX, startY);
        
        if(size(path,1) > maxPath)
            s = round(size(path,1)/maxPath);
            path = path(1:s:size(path,1),:);
        end
        fprintf(fout, 'NumberOfPoints: %d\n', size(path,1));
        for k=1:size(path,1)
            % TODO : Introduce actual z coordinate
            fprintf(fout, '%f %f %f %f %f\n', (path(k,1)-1)*res, (path(k,2)-1)*res, 0, (k-1)*timeRes, 0);
        end
        fprintf(fout, 'ObstacleExistsAfterTrajectory: 0\n');
    end
    fclose(fout);
end
fprintf('Done Generating Experiments\n');

