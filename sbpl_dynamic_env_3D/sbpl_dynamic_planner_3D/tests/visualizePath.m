function visualizePath(envDirectory, solDirectory)
while(1)
  reply = input('\nEnter a test number to view a solution (or type q to quit).\n','s');
  if isempty(reply)
    continue;
  end
  if reply=='q'
    break;
  end
  testNum = str2num(reply);
  fprintf('Visualization loading...\n');
  filename = sprintf('%stest%.4d.exp',envDirectory,testNum);
  fin = fopen(filename, 'r');
  while 1
    a=fgets(fin);
    if(a == -1)
      fprintf('env file ended early\n');
      return;
    end
    if strncmp(a, 'environment:', 12)
      break;
    end
  end
  map = zeros(500,500);
  map_vals = fscanf(fin, '%d');
  fclose(fin);
  for y=1:500
      for x=1:500
          map(y,x) = map_vals((y-1)*500+x);
      end
  end

  robotPath = [];
  filename = sprintf('%ssol%.4d.txt',solDirectory,testNum);
  fin = fopen(filename, 'r');
  path = fscanf(fin, '%f');
  fclose(fin);

  oldX = -1; oldY = -1; oldTheta = -1;
  i = 1;
  waitCount=0;
  while i<=length(path)
      if(oldX == path(i) && oldY == path(i+1) && oldTheta == path(i+2))
        waitCount = waitCount + 1;
      end
      oldX = path(i); oldY = path(i+1); oldTheta = path(i+2);
      i=i+4;
  end

  i=1;
  while i<=length(path)
      robotPath = [robotPath; [floor(path(i)/0.025)+1,floor(path(i+1)/0.025)+1,path(i+2),path(i+3)]];
      i=i+4;
  end

  dynObsPath = [];
  obsCount = 0;
  fat = [];
  filename = sprintf('%sdynObs%.4d.dob',envDirectory,testNum);
  fin = fopen(filename, 'r');
  while 1
      a=fgets(fin);
      if(a == -1)
          break;
      end
      if strncmp(a, 'ObstacleRadius:', 15)
          rad = strread(a,'%s','delimiter',' ');
          rad = strread(cell2mat(rad(2)), '%f');
          if(rad > 0.05)
            fat = [fat; true];
          else
            fat = [fat; false];
          end
      end
      if strncmp(a, 'NumberOfPoints:', 15)
          obsCount = obsCount + 1;
          numPts = strread(a,'%s','delimiter',' ');
          numPts = strread(cell2mat(numPts(2)), '%d');
          
          for i=1:numPts
              point_str = fgets(fin);
              point = strread(point_str,'%f','delimiter',' ');
              dynObsPath(obsCount,i,:) = [round(point(1)/0.025)+1,round(point(2)/0.025)+1,point(3)];
          end
      end
  end

  robot_pts(1,:) = [-0.5 0.5 0.5 -0.5];
  robot_pts(2,:) = [0.5 0.5 -0.5 -0.5];
  goal_pts = robot_pts*3;
  t=0;
  robotPtr = 1;
  dynObsPtr = ones(obsCount,1);

  figure(1)
  imagesc(~map); colormap gray; axis image;
  hold on;
  goal1 = plot(robotPath(end,1),robotPath(end,2),'g*');
  robotHandle1 = plot(robotPath(1,1),robotPath(1,2),'bx');
  obsHandle1 = zeros(obsCount,1);
  for i=1:obsCount
    obsHandle1(i) = plot(dynObsPath(i,1,1),dynObsPath(i,1,2),'ro');
  end
  ax1 = gca;
  hold off;

  figure(2)
  imagesc(~map); colormap gray; axis image;
  hold on;
  goal2 = patch(robotPath(end,1)+goal_pts(1,:),robotPath(end,2)+goal_pts(2,:),'g');
  robotHandle2 = patch(robotPath(1,1)+robot_pts(1,:),robotPath(1,2)+robot_pts(2,:),'b');
  obsHandle2 = zeros(obsCount,1);
  for i=1:obsCount
    obsHandle2(i) = drawRobot(obsHandle2(i),dynObsPath(i,1,1),dynObsPath(i,1,2),fat(i),true);
  end
  w=40;
  axis([robotPath(robotPtr,1)-w,robotPath(robotPtr,1)+w,robotPath(robotPtr,2)-w,robotPath(robotPtr,2)+w]);
  ax2 = gca;
  hold off;

  fprintf('There are %d waits in this solution.\n',waitCount);
  fprintf('Experiment loaded! Press any key to begin visualization.\n');
  pause;
  while 1
      while(robotPtr <= length(robotPath) && t > robotPath(robotPtr,4))
          robotPtr = robotPtr + 1;
      end
      if(robotPtr > length(robotPath))
          break;
      end
      %{
      delete(robotHandle1);
      figure(1)
      hold on;
      robotHandle1 = plot(robotPath(robotPtr,1),robotPath(robotPtr,2),'bx');
      hold off;
      %}
      set(robotHandle1,'x',robotPath(robotPtr,1),'y',robotPath(robotPtr,2));

      %{
      delete(robotHandle2);
      figure(2)
      hold on;
      robotHandle2 = patch(robotPath(robotPtr,1)+robot_pts(1,:),robotPath(robotPtr,2)+robot_pts(2,:),'b');
      hold off;
      %}
      set(robotHandle2,'x',robotPath(robotPtr,1)+robot_pts(1,:),'y',robotPath(robotPtr,2)+robot_pts(2,:));
      if(map(robotPath(robotPtr,2),robotPath(robotPtr,1)) == 1)
        fprintf('whoa!\n');
      end

      %axis([robotPath(robotPtr,1)-w,robotPath(robotPtr,1)+w,robotPath(robotPtr,2)-w,robotPath(robotPtr,2)+w]);
      set(ax2,'XLim',[robotPath(robotPtr,1)-w,robotPath(robotPtr,1)+w],'YLim',[robotPath(robotPtr,2)-w,robotPath(robotPtr,2)+w]);
      
      for i=1:obsCount
          while(dynObsPtr(i) <= length(dynObsPath) && t > dynObsPath(i,dynObsPtr(i),3))
              dynObsPtr(i) = dynObsPtr(i) + 1;
          end
          if(dynObsPtr(i) > length(dynObsPath))
            if(obsHandle1(i) > 0)
              delete(obsHandle1(i));
              delete(obsHandle2(i));
              obsHandle1(i)=0;
            end
            continue;
          end

          %{
          delete(obsHandle1(i));
          figure(1)
          hold on;
          obsHandle1(i) = plot(dynObsPath(i,dynObsPtr(i),1),dynObsPath(i,dynObsPtr(i),2),'ro');
          hold off;
          %}
          set(obsHandle1(i),'x',dynObsPath(i,dynObsPtr(i),1),'y',dynObsPath(i,dynObsPtr(i),2));

          %{
          figure(2)
          hold on;
          obsHandle2(i) = drawRobot(obsHandle2(i),dynObsPath(i,dynObsPtr(i),1),dynObsPath(i,dynObsPtr(i),2),fat(i));
          hold off;
          %}
          drawRobot(obsHandle2(i),dynObsPath(i,dynObsPtr(i),1),dynObsPath(i,dynObsPtr(i),2),fat(i),false);
      end
      
      pause(0.1);
      t = t + 0.1;
  end
end
