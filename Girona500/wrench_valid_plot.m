close all
% load data
data = load("Girona500/wrench_validation.txt");
n = size(data,1);
tau_est = data(:,1:6)';
tau_est_scaled = data(:,7:12)';
wrench_mea = data(:,13:18)';
wrench_mea_filtered = 0*wrench_mea;
alpha = 0.95;
for i=1:n-1
wrench_mea_filtered(:,i+1) = alpha* wrench_mea_filtered(:,i) + (1-alpha)*wrench_mea(:,i);
end
norm_nu = data(:,19)';
t = 1:n;
t = t/50;
fig_name_list = "Prediction_comparison_in_" + ["X","Y","Z","Rx","Ry","Rz"];
for i=1:6
    figure
    plot(t,tau_est(i,:), ...
        t,tau_est_scaled(i,:), ...
        t,wrench_mea_filtered(i,:));
    legend(["pre","pre scaled","mea"],"FontSize",8)
    ax = gcf;
    ax.Units = "centimeters";
    ax.Position = [20,10,8,6];
    print(ax, '-dpng', '-r300', fig_name_list(i)+'.png')

end