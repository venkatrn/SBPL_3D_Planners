function path = findPath(map, startX, startY, goalX, goalY)
    disp('find path');
    %wavefront grid init
    wavefront = inf(size(map,1),size(map,2));
    
    %fifo init
    fifo = FIFO_Init(map);
    if(map(startY,startX) == 0)
        fifo = FIFO_Insert(fifo,[startX, startY]);
        wavefront(startY,startX) = 0;
    end
    
    %BFS
        disp('begin');
        count = 0;
        percent = 0.1;
    while(~FIFO_IsEmpty(fifo) && wavefront(goalY,goalX)==inf)
        [fifo, x, y] = FIFO_Extract(fifo);
        [fifo, wavefront] = updateNeighbors(fifo, wavefront, map, x, y);
        count = count + 1;
        if(count/numel(map) > percent)
            fprintf('%f of map searched\n',percent);
            percent = percent + 0.1;
        end
    end
            disp('end');
    
    %if we were not able to find a path to the goal return an empty path
    if(wavefront(goalY,goalX) == inf)
        path = [];
        return;
    end
    
    %reconstruct the path by working backward through the wavefront
    path = zeros(wavefront(goalY,goalX)+1,2);
    path(size(path,1),:) = [goalX, goalY];
    for p=size(path,1)-1:-1:1
        found = false;
        x = path(p+1,1);
        y = path(p+1,2);
        for i=x-1:2:x+1
            if(i>0 && i<=size(wavefront,2) && wavefront(y,i) < wavefront(y,x))
                path(p,:) = [i,y];
                found = true;
            end
        end
        for j=y-1:2:y+1
            if(j>0 && j<=size(wavefront,1) && wavefront(j,x) < wavefront(y,x))
                path(p,:) = [x,j];
                found = true;
            end
        end
        if(~found)
            disp('Error: Could not reconstruct the path!');
        end
    end
end

function [fifo, wavefront] = updateNeighbors(fifo, wavefront, map, x, y)
for i=x-1:2:x+1
    if(i<=0 || i>size(wavefront,2) || wavefront(y,i) < inf || map(y,i) == 1)
        continue;
    end
    wavefront(y,i) = wavefront(y,x) + 1;
    fifo = FIFO_Insert(fifo, [i,y]);
end
for j=y-1:2:y+1
    if(j<=0 || j>size(wavefront,1) || wavefront(j,x) < inf || map(j,x) == 1)
        continue;
    end
    wavefront(j,x) = wavefront(y,x) + 1;
    fifo = FIFO_Insert(fifo, [x,j]);
end
end

function fifo = FIFO_Init(map)
fifo.queue = zeros(numel(map),2);
fifo.head=1;
fifo.tail=1;
end

function empty = FIFO_IsEmpty(fifo)
empty = fifo.head == fifo.tail;
end

function fifo = FIFO_Insert(fifo, pair)
fifo.queue(fifo.tail,:) = pair;
fifo.tail = fifo.tail + 1;
end

function [fifo,x,y] = FIFO_Extract(fifo)
if(FIFO_IsEmpty(fifo))
    disp('Error: tried to extract when FIFO was empty');
    x = -1;
    y = -1;
    return; 
end
x = fifo.queue(fifo.head,1);
y = fifo.queue(fifo.head,2);
fifo.head = fifo.head + 1;
end
