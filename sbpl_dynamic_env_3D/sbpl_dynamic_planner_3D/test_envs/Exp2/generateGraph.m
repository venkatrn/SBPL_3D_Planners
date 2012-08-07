numEnvironments = 50;
count = 0;
data_count = 0;

for j = 0:numEnvironments-1
    
    path = sprintf('stats%.4d.txt',j);
    try                             % Ignore file if its empty
        file = dlmread(path);
    catch
        continue;
    end
    count = count+1;
    format long;
    unique_th = unique(file(:,1));
    unique_eps =unique(file(:,2));
    
    if j == 0
        x = unique_eps;
        plan_times = cell(1,numel(unique_th)) ;
        num_expands = cell(1,numel(unique_th)) ;
        solution_costs = cell(1,numel(unique_th)) ;
    end
    
    types = {'r-.','g-*','b-o','m-s'};
    for i = 1:numel(unique_th)
        data = file((file(:,1) == unique_th(i)),:);
        
        for k = 1:numel(unique_eps)
            data2 = data((data(:,2) == unique_eps(k)),:);
            if j == 0 
                plan_times{i}(k) = 0; %;zeros(1,numel(unique_eps));
                num_expands{i}(k) = 0;%zeros(1,numel(unique_eps));
                solution_costs{i}(k) = 0;%; zeros(1,numel(unique_eps));
            end
            plan_times{i}(k) = plan_times{i}(k) + sum(data2(:,3));
            num_expands{i}(k) = num_expands{i}(k) + sum(data2(:,4));
            solution_costs{i}(k) = solution_costs{i}(k) + sum(data2(:,6));
        end
       
        
    end
     
    if count==40
        break;
    end
   
end

% Make the plan times and number of expansions cumulative

for i = 1:numel(unique_th)

    plan_times{i} = cumsum(plan_times{i}(1,end:-1:1));
    plan_times{i} = plan_times{i}(1,end:-1:1);
    num_expands{i} = cumsum(num_expands{i}(1,end:-1:1));
    num_expands{i} = num_expands{i}(1,end:-1:1);
end


% Plot graph - Average Plan Time vs Time Horizon for Epsilon = 1,3,5,10
figure;
for i = 1:numel(unique_th)
    hold on;
    plot(plan_times{i}/count,x,types{i});legend('T_h = 5 s','T_h = 15 s','T_h = 25 s','T_h = 35 s','Location','NorthEast');
    xlabel('Time for solution upto time horizon (s)');
    ylabel('Epsilon (\epsilon)');
    axis tight;axis square;
    %title('Average Plan Time vs Epsilon (\epsilon) for T_h \in \{5, 15, 25, 35\}')
    title('Epsilon Reached vs Average Plan Time for T_h \in \{5, 15, 25, 35\}')
end


% Plot graph - Average Number of Expansions of  vs Time Horizon for Epsilon = 1,3,5,10
figure;
for i = 1:numel(unique_th)
    hold on;
    plot(x,num_expands{i}/count,types{i});legend('T_h = 5 s','T_h = 15 s','T_h = 25 s','T_h = 35 s','Location','NorthEast');
    ylabel('Average Number of Expansions');
    xlabel('Epsilon (\epsilon)');
    axis tight;axis square;
    title('Average Expansions vs Epsilon (\epsilon ) for T_h \in \{5, 15, 25, 35\}')
end

% Plot graph - Average Solution Cost vs Time Horizon for Epsilon = 1,3,5,10
figure;
for i = 1:numel(unique_th)
    hold on;
    plot(x,solution_costs{i}*1e-3/count,types{i});legend('T_h = 5 s','T_h = 15 s','T_h = 25 s','T_h = 35 s','Location','SouthEast');
    ylabel('Average Solution Cost (s)');
    xlabel('Epsilon (\epsilon)');
    axis tight;axis square;
    title('Average Solution Cost vs Epsilon (\epsilon) for T_h \in \{5, 15, 25, 35\}')
end

% Plot graph - Average Solution Cost vs Time Horizon for Epsilon = 1,3,5,10
figure;
for i = 1:numel(unique_th)
    hold on;
    plot(plan_times{i}/count,solution_costs{i}/count,types{i});legend('T_h = 5 s','T_h = 15 s','T_h = 25 s','T_h = 35 s','Location','NorthEast');
    ylabel('Average Solution Cost');
    xlabel('Time for solution upto time horizon (s)');
    axis tight;axis square;
   % title('Solution Cost vs Planning Time for T_h \in \{5, 15, 25, 35\}')
end