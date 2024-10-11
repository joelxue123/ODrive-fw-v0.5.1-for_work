    current_b_c_data = importdata("velocity2_plot.csv");
    n = size(current_b_c_data.data, 1);

    x = current_b_c_data.data(:, 1);
    y = current_b_c_data.data(:, 2)/16*60;


    % 创建图形窗口
    figure;
    plot(x,y); 
