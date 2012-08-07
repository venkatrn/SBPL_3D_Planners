function h = drawRobot(h,x,y,fat,init)

persistent fat_pts thin_pts

if(isempty(fat_pts))
  fat_pts(1,:) = 5*cos(0:2*pi/100:2*pi);
  fat_pts(2,:) = 5*sin(0:2*pi/100:2*pi);

  thin_pts(1,:) = 2*cos(0:2*pi/100:2*pi);
  thin_pts(2,:) = 2*sin(0:2*pi/100:2*pi);
end

%{
delete(h);
hold on;
if(fat)
  h=patch(x+fat_pts(1,:),y+fat_pts(2,:),'r');
else
  h=patch(x+thin_pts(1,:),y+thin_pts(2,:),'r');
end
hold off;
%}
if(init)
  hold on;
  if(fat)
    h=patch(x+fat_pts(1,:),y+fat_pts(2,:),'r');
  else
    h=patch(x+thin_pts(1,:),y+thin_pts(2,:),'r');
  end
  hold off;
else
  if(fat)
    set(h,'x',x+fat_pts(1,:),'y',y+fat_pts(2,:));
  else
    set(h,'x',x+thin_pts(1,:),'y',y+thin_pts(2,:));
  end
end

