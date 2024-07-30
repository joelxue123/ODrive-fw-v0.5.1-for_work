    current_b_c_data = importdata("speed.csv");
    n = size(current_b_c_data.data, 1);

    x = current_b_c_data.data(:, 1);
    y = current_b_c_data.data(:, 2);


    % 创建图形窗口
    figure;
    plot(x,y); 
