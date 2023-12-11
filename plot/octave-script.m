% load textfile
arg_list=argv();
fin=arg_list{1};

data=load(fin);

% plot data
stairs(data(:,1), data(:,2:end), 'LineWidth', 2);
grid('on');
xlim([data(1,1) data(end,1)]);
xlabel('t [s]');
legend({'reference', 'plant reference', 'plant output'});

% save plot as file
% saveas(gcf,'result_stefano.png');



% generate output plot file name
[~, name, ~] = fileparts(fin);
output_file = strcat(name, '_plot.png');

% save plot as file
saveas(gcf, output_file);
