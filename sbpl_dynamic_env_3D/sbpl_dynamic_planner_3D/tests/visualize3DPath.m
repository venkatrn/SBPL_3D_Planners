% Visualizes 3D path

fid = fopen('./test0000.exp');

map_size = fgets(fid);
map_size = map_size(24:end);
map_size = str2num(map_size);




for i = 1:12
    s = fgets(fid);
end
map = reshape(str2num(fscanf(fid,'%c')),map_size(1),map_size(2),map_size(3));
%for i = 1:map_size(3)
   %imshow((~map(:,:,1))');hold on;axis xy;
%end
[x y z] = ind2sub(size(map), find(map));
plot3(x, y, z, 'k.');
hold on;
fclose(fid);

path = dlmread('../sol.txt')./0.1;
path = path(:,1:3);
plot3(path(1,1),path(1,2),path(1,3),'rs');hold on;
plot3(path(:,1),path(:,2),path(:,3),'b.-');hold on;
plot3(path(end,1),path(end,2),path(end,3),'rs');hold on;



