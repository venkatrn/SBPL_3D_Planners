numEnvironments = 50;
count = 0;
types = {'r-.','g-*','b-o','m-s'};
bins = [0:.02:0.1 1:2:10 inf];
format short;
for j = 0:numEnvironments-1
    
    path = sprintf('stats%.4d.txt',j);
    try                             % Ignore file if its empty
        file = dlmread(path);
    catch
        continue;
    end
    count = count+1;
    
    unique_eps = unique(file(:,1));
    if j == 0
        x = cell(1,numel(unique_eps));
        plan_times = cell(1,numel(unique_eps)) ;
        num_expands = cell(1,numel(unique_eps)) ;
        solution_costs = cell(1,numel(unique_eps)) ;
    end
    

    for i = 1:numel(unique_eps)
        data = file((file(:,1) == unique_eps(i)),:);
        x{i} = 0:size(plan_time,1)-1;
        if j == 0
            plan_times{i} = zeros(1,numel(bins));
            num_expands{i} = zeros(1,numel(bins));
            solution_costs{i} = zeros(1,numel(bins));
        end
        plan_times{i} = plan_times{i} + histc(data(:,3)',bins);
        num_expands{i} = num_expands{i} + histc(data(:,4)',bins);
        solution_costs{i} = solution_costs{i} + histc(data(:,6)',bins);
            
    end
    if count==40
        break;
      end
    
end

table = zeros(numel(bins),numel(unique_eps));
for i = 1:numel(unique_eps)
    table(:,i) = plan_times{i}*100/sum(plan_times{i});
end
[bins(2:end)' table(1:end-1,:)]



% for i = 1:numel(unique_eps)
%     figure;
%     hold on;
%     %stem(plan_times{i})
%     bar(plan_times{i})
%     ylabel('Time for first solution upto time horizon (s)');
%     xlabel('Time(s)');
%     axis tight;axis square;
%  
% end

