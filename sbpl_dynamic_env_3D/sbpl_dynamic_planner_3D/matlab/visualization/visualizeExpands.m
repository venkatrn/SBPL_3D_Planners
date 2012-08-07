function data = visualizeExpands(name)
data = load(name);
sizeX = max(data(:,1))+1;
sizeY = max(data(:,2))+1;
map = zeros(sizeY,sizeX);
iter=0
for i=1:size(data,1)
  %map(data(i,2)+1,data(i,1)+1) = map(data(i,2)+1,data(i,1)+1) + 1;
  if iter~=data(i,5)
    iter = iter+1;
    figure(iter);
    imagesc(map);
  end
  map(data(i,2)+1,data(i,1)+1) = data(i,5)+1;
end
imagesc(map);
%data(data(:,1)==243 & data(:,2)==77,:)
