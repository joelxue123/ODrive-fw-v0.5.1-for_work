
% 定义 x 轴数据
x = 1:1:250;

% 调用函数，传入不同的文件名和相位角
analyze_and_plot_data("current_B_C_0.csv", 0);
analyze_and_plot_data("current_B_C_1.csv", 1);
analyze_and_plot_data("current_B_C_1_6.csv", 1.6);

