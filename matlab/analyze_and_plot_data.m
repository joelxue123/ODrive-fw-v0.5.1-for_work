function analyze_and_plot_data(file_name, phase_angle)
    % 导入数据
    current_b_c_data = importdata(file_name);
    n = size(current_b_c_data.data, 1);

    current_b = current_b_c_data.data(:, 2);
    current_c = current_b_c_data.data(:, 3);

    % 计算 Ialpha 和 Ibeta
    Ialpha = -current_b - current_c;
    Ibeta = sqrt(3)/3 * (current_b - current_c);

    % 计算 Id 和 Iq
    C_I = cos(phase_angle);
    S_I = sin(phase_angle);
    Id = C_I * Ialpha + S_I * Ibeta;
    Iq = C_I * Ibeta - S_I * Ialpha;

    % 创建图形窗口
    figure;
    subplot(2, 1, 1); % 第一个子图
    plot(1:n, current_b, 1:n, current_c); 
    ylim([-1.5 1.5]);
    title('Currents b and c');

    subplot(2, 1, 2); % 第二个子图
    plot(1:n, Iq, 1:n, Id);
    ylim([-0.5 1.5]);
    title('Id and Iq');
end
