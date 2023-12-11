% Load the TSV files
numFiles = 5;
files = cell(1, numFiles);
for i = 1:numFiles
    files{i} = dlmread(['../data/scope_data_stefano_' num2str(i-1) '.tsv'], '\t');
end

% Extract the 4th column from each file
cols = cell(1, numFiles);
for i = 1:numFiles
    cols{i} = files{i}(:, 4);
end

% Create a single plot
figure;
colors = {'b', 'r', 'g', 'c', 'm'};
LineWidth_ = 0.2;
min_plot = 100;
max_plot = 1000;

hold on;
for i = 1:numFiles
    plot(min_plot:max_plot, cols{i}(min_plot:max_plot), colors{i}, 'LineWidth', LineWidth_);
end
hold off;

legend('PI_Kp=10,ki=1', 'PI_Kp=15,ki=1', 'PI_Kp=10,ki=0.1', 'PI_Kp=10,ki=0.1,kd=1', 'PI_Kp=10,ki=0.1,kd=0.1');
xlabel('t [s]');
ylabel('Value');
title('Plot of 4th Column from Multiple TSV Files');
grid on;

% Save the plot as a file
print('output.png', '-dpng');

