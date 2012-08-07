function compute_stats(path)
eps_dec = 1.0;

% Begin Parse Raw Data
fname = dir([path '/data_*.csv']);
for i=1:length(fname)
  data(i,:,:) = csvread([path '/' fname(i).name]);
  names{i} = fname(i).name(6:end-4);
end

expands=-ones(size(data,1),size(data,2),size(data,3)/3);
t=-ones(size(data,1),size(data,2),size(data,3)/3);
g=-ones(size(data,1),size(data,2),size(data,3)/3);

error_idx=[];
solFoundSkips=[];
for j=1:size(data,2) %loop over trials
  if any(data(:,j,1)<0) %skip a trial if any of the planners didn't find a solution
    solFoundSkips(end+1,:)=[j (data(:,j,1)>=0)']; %record which planners did find a solution though
  end
  for i=1:size(data,1) %loop over planners
    for k=1:3:size(data,3) %loop over epsilon
      expands(i,j,(k-1)/3+1)=data(i,j,k);
      t(i,j,(k-1)/3+1)=data(i,j,k+1);
      g(i,j,(k-1)/3+1)=data(i,j,k+2);
    end
  end
end
if ~isempty(solFoundSkips)
  fprintf('%d trials had no solution for at least one planner\n',size(solFoundSkips,1));
  error_idx = [error_idx solFoundSkips(:,1)'];
end
% End Parse Raw Data

% Compute epsilon vector
epsilon = 1.0:eps_dec:(size(expands,3)-1)*eps_dec+1;

% Begin Error Checking
optSolsNotEqual=[];
optSolsNotMin=[];
for i=1:size(g,2)
  if any(i==error_idx)
    continue;
  end
  if any(g(1,i,1)~=g(:,i,1))
    optSolsNotEqual(end+1) = i;
  end
  for j=1:size(g,1)
    if min(g(j,i,:))~=g(j,i,1)
      optSolsNotMin(end+1,:)=[j i];
    end
  end
end

if ~isempty(optSolsNotEqual)
  fprintf('Error: %d trials had planners with different optimal solution costs\n',length(optSolsNotEqual));
  error_idx = [error_idx optSolsNotEqual];
end
if ~isempty(optSolsNotMin)
  fprintf('Error: %d trials had planners with the optimal solution cost not being minimal\n',length(optSolsNotEqual));
  error_idx = [error_idx optSolsNotMin(:,2)'];
end

error_idx = unique(error_idx);
good_idx = setdiff(1:size(expands,2),error_idx);
% End Error Checking

% Begin Compute Stats
for i=1:size(expands,1)
  for j=1:size(expands,3)
    temp = expands(i,:,j);
    avg_expands(i,j) = mean(temp(good_idx));
    std_expands(i,j) = std(temp(good_idx));
    temp = t(i,:,j);
    avg_time(i,j) = mean(temp(good_idx));
    std_time(i,j) = std(temp(good_idx));
    temp = g(i,:,j);
    avg_cost(i,j) = mean(temp(good_idx));
    std_cost(i,j) = std(temp(good_idx));
  end
end
if size(expands,1)==2
  for i=1:size(expands,3)
    temp = expands(2,:,i)./expands(1,:,i);
    avg_expand_ratio(i) = median(temp(good_idx));
  end
end
% End Compute Stats

% Make Stat Directories
!mkdir stats
!mkdir stats/averages
!mkdir stats/expands
!mkdir stats/time
!mkdir stats/cost

% Begin Drawing Plots

h=figure(1);
% draw average expands
clf; plot(epsilon,avg_expands); xlabel('epsilon'); ylabel('expands'); legend(names); print(h,'-dpdf','stats/averages/average_expands.pdf');
clf; plot(epsilon,log10(avg_expands)); xlabel('epsilon'); ylabel('log10(expands)'); legend(names); print(h,'-dpdf','stats/averages/log_average_expands.pdf');
% draw average time
clf; plot(epsilon,avg_time); xlabel('epsilon'); ylabel('time (s)'); legend(names); print(h,'-dpdf','stats/averages/average_time.pdf');
clf; plot(epsilon,log10(avg_time)); xlabel('epsilon'); ylabel('log10(time (s))'); legend(names); print(h,'-dpdf','stats/averages/log_average_time.pdf');
% draw average cost
clf; plot(epsilon,avg_cost); xlabel('epsilon'); ylabel('cost'); legend(names); print(h,'-dpdf','stats/averages/average_cost.pdf');
% draw average expand ratio
if size(expands,1)==2
  clf; plot(epsilon,avg_expand_ratio); xlabel('epsilon'); ylabel('median expand ratio'); print(h,'-dpdf','stats/averages/average_expand_ratio.pdf');
end

% draw detailed plots showing all trials
for i=1:size(expands,3)
  clf; plot(expands(:,:,i)'); xlabel('trial'); ylabel('expands'); legend(names); fout=sprintf('stats/expands/expandsEps%.1f',epsilon(i)); fout(fout=='.')='_'; fout=[fout '.pdf']; print(h,'-dpdf',fout);
  clf; plot(t(:,:,i)'); xlabel('trial'); ylabel('time (s)'); legend(names); fout=sprintf('stats/time/timeEps%.1f',epsilon(i)); fout(fout=='.')='_'; fout=[fout '.pdf']; print(h,'-dpdf',fout);
  clf; plot(g(:,:,i)'); xlabel('trial'); ylabel('cost'); legend(names); fout=sprintf('stats/cost/costEps%.1f',epsilon(i)); fout(fout=='.')='_'; fout=[fout '.pdf']; print(h,'-dpdf',fout);
end

% End Drawing Plots

% Begin Saving Data
stats.raw = data;
stats.epsilon = epsilon;
stats.names = names;
stats.expands = expands;
stats.time = t;
stats.cost = g;
stats.avg_expands = avg_expands;
stats.avg_time = avg_time;
stats.avg_cost = avg_cost;
if size(expands,1)==2
  stats.avg_expand_ratio = avg_expand_ratio;
end
stats.std_expands = std_expands;
stats.std_time = std_time;
stats.std_cost = std_cost;
stats.solFoundSkips = solFoundSkips;
stats.optimalSolsNotEqual = optSolsNotEqual;
stats.optimalSolsNotMin = optSolsNotMin;

save stats/stats.mat stats;
% End Saving Data

