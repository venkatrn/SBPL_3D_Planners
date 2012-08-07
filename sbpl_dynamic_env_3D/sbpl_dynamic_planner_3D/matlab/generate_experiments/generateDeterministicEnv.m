function generateDeterministicEnv()
rand('twister',sum(100*clock))






%environment parameters
res = 0.1;
timeRes = 0.1;

% robot parameters
robotRadius = 3;


% Customize Map here
% Map 1 - Crossroads
%%length of each side of the map
%side = 100;
%depth = 15;
% map = ones(side,side);
% map(4*side/7:5*side/7,:) = 0;
% map(:,4*side/7:5*side/7) = 0;
%Start and goal for robot
% robotStart = [97 64 5];
% robotGoal = [11 64 1];
% startX = robotStart(1); startY = robotStart(2); startZ = robotStart(3);
% goalX = robotGoal(1); goalY = robotGoal(2); goalZ = robotGoal(3);
% obsStart = [11 64 1;
%             64 92 1];
% obsGoal =  [97 64 1;
%             64 9 1];
% obsRad = [10 8];
% obsVel = [1 0.8];  % 1 means 1 cell per time resolution


% Map 2 -Quadwaves

% map = im2bw(imread('quadrotor_waves3.png'));
%%length of each side of the map
%side = 100;
%depth = 15;
% robotStart = [10 84 3];
% robotGoal = [37 18 14];
% startX = robotStart(1); startY = robotStart(2); startZ = robotStart(3);
% goalX = robotGoal(1); goalY = robotGoal(2); goalZ = robotGoal(3);
% obsStart = [ 
%             70 24 4;70 24 11;
%             72 7 1;72 7 8;72 7 15;68 7 1;68 7 8;68 7 15;
%             
%             35 18 15;35 18 14;35 18 13;35 18 12;35 18 11;35 18 10;35 18 9; 
%             35 22 15;35 22 14;35 22 13;35 22 12;35 22 11;35 22 10;35 22 9;
%             
%             20 18 1;20 18 2;20 18 3;20 18 4;20 18 5;20 18 6;20 18 7;
%             20 22 1;20 22 2;20 22 3;20 22 4;20 22 5;20 22 6;20 22 7;
%             
%             5 18 15;5 18 14;5 18 13;5 18 12;5 18 11;5 18 10;5 18 9;
%             5 22 15;5 22 14;5 22 13;5 22 12;5 22 11;5 22 10;5 22 9;
%             
%                   ];
% obsGoal =  [   
%             70 99 4;70 99 11;
%             72 99 1;72 99 8;72 99 15;68 99 1;68 99 8;68 99 15;
%             
%             68 99 15;68 99 14;68 99 13;68 99 12;68 99 11;68 99 10;68 99 9;
%             72 99 15;72 99 14;72 99 13;72 99 12;72 99 11;72 99 10;72 99 9;
%             
% 
%             68 99 1;68 99 2;68 99 3;68 99 4;68 99 5;68 99 6;68 99 7;
%             72 99 1;72 99 2;72 99 3;72 99 4;72 99 5;72 99 6;72 99 7;
%             
%             68 99 15;68 99 14;68 99 13;68 99 12;68 99 11;68 99 10;68 99 9;
%             72 99 15;72 99 14;72 99 13;72 99 12;72 99 11;72 99 10;72 99 9;
%             ];
% obsRad = 0.5*ones(1,size(obsStart,1));
% obsVel = [0.35*ones(1,8) 0.25*ones(1,42)];  % 1 means 1 cell per time resolution

% Map 3 - Outdoor
map = im2bw(imread('outdoor4.png'));
%length of each side of the map
side = 500;
depth = 10;



% robotStart = [239 414 10];
% robotGoal = [407 49 1];
% startX = robotStart(1); startY = robotStart(2); startZ = robotStart(3);
% goalX = robotGoal(1); goalY = robotGoal(2); goalZ = robotGoal(3);
% obsStart = [36 139 5;
%             454 102 1;
%            387 377 4;
%            33 282 5];
% obsGoal =  [466 450 1;
%             42 280 5;
%             213 44 1;
%             461 145 5];       
% obsRad = 20*ones(1,size(obsStart,1));
% obsVel = 1*ones(1,size(obsStart,1));;  % 1 means 1 cell per time resolution

robotStart = [304 478 10];
robotGoal = [407 49 1];
startX = robotStart(1); startY = robotStart(2); startZ = robotStart(3);
goalX = robotGoal(1); goalY = robotGoal(2); goalZ = robotGoal(3);
obsStart = [36 139 10;
            454 102 10;
           387 377 10;
           33 282 10;
           126 231 10];
obsGoal =  [466 450 1;
            42 280 5;
            213 44 1;
            461 145 5;
            156 465 5];       
obsRad = 20*ones(1,size(obsStart,1));
obsVel = 0.8*ones(1,size(obsStart,1));;  % 1 means 1 cell per time resolution



numDynObs = size(obsStart,1);

imshow(map)

for i=1:1
    fprintf('\ngenerating test %d\n',i);
    
    %Stack planes to form 3D world
    map3D = zeros(side,side,depth);
    size(map)
    for k = 1:depth
        map3D(:,:,k) = map;
    end
    map = map3D;
    
    
    %write environment to file
    filename = sprintf('tests/test%.4d.exp',i-1);
    fout = fopen(filename, 'w');
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
    
    
    
    
    fprintf(fout, 'start(meters,rads): %f %f %f %f %f\n', (startX-1)*res+res/2, (startY-1)*res+res/2, (startZ-1)*res+res/2, rand*2*pi, 0.0);
    fprintf(fout, 'end(meters,rads): %f %f %f %f\n', (goalX-1)*res+res/2, (goalY-1)*res+res/2, (goalZ-1)*res+res/2, rand*2*pi);
    fprintf(fout, 'environment:\n');
    %write map out to file
    %out_map = zeros(size(map));
  
    for m = 1:size(map,3)
       % out_map(:,:,m) = map(:,:,m)';
        out_map = map(:,:,m)';
        fprintf(fout,num2str(out_map(:)'));
        fprintf(fout,'\n');
    end
    %out_map = out_map(:)';
   % fprintf(fout,num2str(out_map));
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
        
        radius = obsRad(j);
        
        fprintf(fout, 'ObstacleRadius: %f\n', radius*res);
        fprintf(fout, 'NumberOfTrajectories: 1\n');
        fprintf(fout, 'TrajectoryID: 0\n');
        fprintf(fout, 'TrajectoryProbability: 1.0\n');
        
        path = a_star3D(map,obsStart(j,:),obsGoal(j,:));
        
        
        fprintf(fout, 'NumberOfPoints: %d\n', size(path,1));
        for k=1:size(path,1)
            fprintf(fout, '%f %f %f %f %f\n', (path(k,1)-1)*res, (path(k,2)-1)*res, (path(k,3)-1)*res, (k-1)*timeRes/obsVel(j), 0);
        end
        fprintf(fout, 'ObstacleExistsAfterTrajectory: 0\n');
    end
    fclose(fout);
end
fprintf('Done Generating Experiments\n');

end

function paddedMap = imdilate3D(map,robotRadius)
paddedMap = zeros(size(map));
for  i = 1:size(map,3)
    paddedMap(:,:,i) = imdilate(im2bw(map(:,:,i), 0.5), strel('disk',robotRadius));
end
end