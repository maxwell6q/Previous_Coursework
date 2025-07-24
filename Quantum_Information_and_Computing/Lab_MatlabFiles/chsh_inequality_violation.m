% Script to compute the CHSH parameter and its variance from raw
% measurement data. The data is assumed to be stored in file called 
% "chsh_dataset.mat" in a subdirectory "data" to the root directory of this
% file.
clear

%% User Inputs

% Window to consider counts [s]
t_window = 1;

% Deltas to consider for gaussian fit [ns] (reduce effect of outliers)
t_gauss = 5;

% Device sampling time
ts_ps = 80.955;     % [ps]
ts_ns = ts_ps*1e-3; % [ns]

% Choice to generate Plots (1:generate, 0:not)
generate_plots = 0;


%% Load Data
% Load dataset into a struct called "meas"
load("data\chsh_dataset.mat")
fns = fieldnames(meas);


%% Computation of Coincidences
% Preallocate
coincidences = zeros(4,4);
sample_mean = zeros(4,4);


for run=1:length(fns)
    dat = meas.(fns{run});

    % Take middle one second of data
    samples_per_second = 1e12/ts_ps;
    middle = dat(1,1) + (dat(end,1)-dat(1,1))/2;
    
    window = dat((dat(:,1) >= middle-samples_per_second*t_window/2 &...
                  dat(:,1) <= middle+samples_per_second*t_window/2),:);
    
    % Split the channels (only save timestamps)
    ch1 = window(window(:,2) == 1, 1);
    ch4 = window(window(:,2) == 4, 1);
    
    % Split by long and short vectors
    if length(ch1) < length(ch4)
        short = ch1;
        long = ch4;
        short_tag = "Channel 1";
        long_tag = "Channel 4";
    else
        short = ch4;
        long = ch1;
        short_tag = "Channel 4";
        long_tag = "Channel 1";
    end
    
    % Preallocate
    nearest_idx = zeros(length(short),1);
    delta = zeros(length(short),1);
    
    % For each short(i), find nearest long(j)
    for i = 1:length(short)
        [delta(i), nearest_idx(i)] = min(abs(long - short(i)));
    end
    
    % For each chosen long(j), keep only the closest short(i)
    unique_long = unique(nearest_idx);
    keep_idx = false(length(short),1);
    
    for i = 1:length(unique_long)
        idx = unique_long(i);
        
        % Find all short events that chose this long event
        matching_shorts = find(nearest_idx == idx);
        
        % Keep only the one with the smallest delta
        [~, min_pos] = min(delta(matching_shorts));
        keep_idx(matching_shorts(min_pos)) = true;
    end
    
    % Final matched pairs
    ch_short = short(keep_idx);
    ch_long = long(nearest_idx(keep_idx));
    
    % Deltas in [ns]
    deltas = (ch_short - ch_long)*ts_ns;
    
    % Filter all events with delta more than 5 ns
    idx_gauss = find(abs(deltas) <= t_gauss);
    deltas = deltas(idx_gauss);
    ch_short = ch_short(idx_gauss);
    ch_long = ch_long(idx_gauss);
    
    % Histogram
    width = ts_ns;
    [counts, edges] = histcounts(deltas, BinWidth=width);
    bin_means = edges(1:end-1) + width/2;
    
    % Sample mean and variance to determine cut off for coincidence window
    sample_mean(run) = sum(counts.*bin_means)/sum(counts);
    sample_var = sum(counts.*(bin_means-sample_mean(run)).^2)...
                 /(length(deltas)-1);
    sigma = sqrt(sample_var);
    
    % Gaussian fit to the data (plot up to scale factor for visibility)
    x = -t_gauss:ts_ns:t_gauss;
    y = normpdf(x,sample_mean(run),sigma);
    
    % Cut-off for coincidence window
    cut_off = 3*sigma;
    
    % Coincidence count
    idx_cutoff = find(abs(deltas) <= cut_off);
    coincidence = length(idx_cutoff);
    coincidences(run) = coincidence;

    % Generate Plots
    if generate_plots == 1
        % Create figure
        fig_num = "f"+run;
        fig.(fig_num) = figure(run);
        
        % Plot the histogram
        histogram(deltas, BinWidth=ts_ns);
    
        % Scale the gaussian
        cdf = normcdf([-t_gauss,t_gauss],sample_mean(run),sigma);
        norm_fact = width*sum(counts)*(cdf(2)-cdf(1));
        hold on
        plot(x,y*norm_fact, LineWidth=1.5)
        
        % Plot the cut-off values as vertical lines
        bar = max(counts)/4;
        plot([1,1]*cut_off+sample_mean(run),[0,bar],'k',LineWidth=1.5)
        plot([1,1]*-cut_off+sample_mean(run),[0,bar],'k',LineWidth=1.5)
        hold off

        grid on
        xlabel("Time difference "+short_tag+" - "+long_tag+" [ns]");
        ylabel("Coincidence count")
        legend("Measured Counts","Gaussian Fit (scaled)","3\sigma cut-off")
    end
end

% Mean of means and standard deviation
sample_mean = reshape(sample_mean,[16,1]);
mean_of_means = mean(sample_mean);
std_of_means = sqrt(var(sample_mean));


%% Compute Probabilities and Correlations
% Total coincidence counts for each combination of bases
total = zeros(4,4);
total(1:2,1:2) = sum(coincidences(1:2,1:2), "all");
total(3:4,1:2) = sum(coincidences(3:4,1:2), "all");
total(1:2,3:4) = sum(coincidences(1:2,3:4), "all");
total(3:4,3:4) = sum(coincidences(3:4,3:4), "all");

% Probabilities
probabilities = coincidences./total;

% Correlations
eval_comb = zeros(4,4); % Combinations of all eigenvalues
for i=1:4
    for j=1:4
        eval_comb(i,j) = (-1)^(i+j);
    end
end
correlations = zeros(2,2);
prob_times_evals = probabilities.*eval_comb;
correlations(1,1) = sum(prob_times_evals(1:2,1:2), "all");
correlations(2,1) = sum(prob_times_evals(3:4,1:2), "all");
correlations(1,2) = sum(prob_times_evals(1:2,3:4), "all");
correlations(2,2) = sum(prob_times_evals(3:4,3:4), "all");

% CHSH Paramter
chsh = abs(sum(correlations(1:3))-correlations(4));


%% Gaussian Method of Linear Error Propagation
% Compute the partial derivatives
for i=1:2
    for j=1:2
        % expand the correlations matrix to be 4x4 for easier computation
        correlations_4x4(i*2-1:i*2,j*2-1:j*2)=ones(2,2)*correlations(i,j);
    end
end
partials = 1./total.*(eval_comb-correlations_4x4);

% Summands for method
summands = partials.^2 .*coincidences;

% Variance of chsh parameter
var_chsh = sum(summands, "all") - 2*sum(summands(3:4,3:4), "all");

% Standard deviation
sigma_chsh = sqrt(var_chsh);