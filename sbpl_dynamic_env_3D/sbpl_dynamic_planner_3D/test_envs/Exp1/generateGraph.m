numEnvironments = 50;
count = 0;

for j = 0:numEnvironments-1
    
    path = sprintf('stats%.4d.txt',j);
    try                             % Ignore file if its empty
        file = dlmread(path);
    catch
        continue;
    end
    count = count+1;
    format long;
    unique_eps = unique(file(:,1));
    if j == 0
        x = cell(1,numel(unique_eps));
        plan_times = cell(1,numel(unique_eps)) ;
        num_expands = cell(1,numel(unique_eps)) ;
        solution_costs = cell(1,numel(unique_eps)) ;
    end
    
    types = {'r-.','g-*','b-o','m-s'};
    for i = 1:numel(unique_eps)
        data = file((file(:,1) == unique_eps(i)),:);
        x{i} = 0:size(plan_time,1)-1;
        if j == 0
            plan_times{i} = zeros(1,size(data,1));
            num_expands{i} = zeros(1,size(data,1));
            solution_costs{i} = zeros(1,size(data,1));
        end
        plan_times{i} = plan_times{i} + data(:,3)';
        num_expands{i} = num_expands{i} + data(:,4)';
        solution_costs{i} = solution_costs{i} + data(:,6)';
        
        
    end
    if count==40
        break;
    end
    
end

% Plot graph - Average Plan Time vs Time Horizon for Epsilon = 1,3,5,10
figure;
for i = 1:numel(unique_eps)
    hold on;
    plot(x{i},plan_times{i}/count,types{i});legend('\epsilon = 1','\epsilon = 3','\epsilon = 5','\epsilon = 10','Location','NorthWest');
    ylabel('Time for first solution upto time horizon (s)');
    xlabel('Time horizon (s)');
    axis tight;axis square;
    title('Average Plan Time vs Time Horizon for  \epsilon \in \{1, 3, 5, 10\}')
end

% Plot graph - Average Plan Time vs Time Horizon for Epsilon = 3,5,10
figure;
for i = 2:numel(unique_eps)
    hold on;
    plot(x{i},plan_times{i}/count,types{i});legend('\epsilon = 3','\epsilon = 5','\epsilon = 10','Location','NorthWest');
    ylabel('Time for first solution upto time horizon (s)');
    xlabel('Time Horizon (s)');
    axis tight;axis square;
    title('Average Plan Time vs Time Horizon for  \epsilon \in \{3, 5, 10\}')
end

% Plot graph - Average Number of Expansions of  vs Time Horizon for Epsilon = 1,3,5,10
figure;
for i = 1:numel(unique_eps)
    hold on;
    plot(x{i},num_expands{i}/count,types{i});legend('\epsilon = 1','\epsilon = 3','\epsilon = 5','\epsilon = 10','Location','NorthWest');
    ylabel('Average Number of Expansions');
    xlabel('Time Horizon (s)');
    axis tight;axis square;
    title('Average Expansions vs Time Horizon for  \epsilon \in \{1, 3, 5, 10\}')
end

% Plot graph - Average Solution Cost of  vs Time Horizon for Epsilon = 1,3,5,10
figure;
for i = 1:numel(unique_eps)
    hold on;
    plot(x{i},solution_costs{i}/count,types{i});legend('\epsilon = 1','\epsilon = 3','\epsilon = 5','\epsilon = 10','Location','NorthWest');
    ylabel('Average Solution Cost');
    xlabel('Time Horizon (s)');
    axis tight;axis square;
    title('Average Solution Cost vs Time Horizon for  \epsilon \in \{1, 3, 5, 10\}')
end
