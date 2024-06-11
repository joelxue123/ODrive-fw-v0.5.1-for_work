    current_iq = importdata("output_actual_velocity.csv");

    y = current_iq(1, 1:1:4096);
    x = 1:1:4096;
    
    % 创建图形窗口
    figure;
     plot(x,y); 

current_iq_setpoint = importdata("output_set_velocity_2.csv");
    z = current_iq_setpoint(1, 1:1:4096);
    
    % 创建图形窗口
    hold on 
 %    plot(x,z); 

 Fs = 2000;               % 采样频率（Hz）
T = 4096*0.0005;                   % 总时间（秒）
f_start = 10;            % 起始频率（Hz）
f_end = 100;             % 结束频率（Hz）
t = 0/Fs:1/Fs:T-1/Fs;       % 时间向量

 k = (f_end - f_start) / T;
f = f_start + 1/2*k * t;
sweep_signal_0 = 5*sin(2 * pi * f .* t);
hold on
 plot(x,sweep_signal_0); 
 z = sweep_signal_0(1, 1:1:4096);
% 
% k = (f_end - f_start) / T;
phase = 0;
sweep_signal = zeros(1, 4096); % 初始化信号数组
t = 0:1/Fs:T-1/Fs;       % 时间向量
for i=0:1:2000
    f = f_start + k * (i * (1/Fs)) + 1/2*k*(1/Fs) ; % 计算当前频率
    phase = phase + 2 * pi * f * (1/Fs); % 更新相位
    if phase > 2 * pi
        phase = phase - 2 * pi; % 确保相位在0到2*pi的范围内
    end
    
 sweep_signal(i+1) = sin(phase); % 计算正弦值并赋值给sweep_signal
end    


hold on
 %plot(x,sweep_signal); 

N = 4096;
frequencies = (0:N/2 -1) * (Fs / N);

inputFFT = fft(z, N);
outputFFT = fft(y, N);
H = outputFFT ./ inputFFT;
magnitudeResponse = abs(H);
angle_response_in = angle(inputFFT);
angle_response_out = angle(outputFFT);
angle_response = angle_response_out - angle_response_in;
for i=1:1:2048
if angle_response(i) > pi
    angle_response(i) = angle_response(i) - 2*pi;
end
if angle_response(i) < -pi
    angle_response(i) = angle_response(i) +   2*pi;
end
end

figure
 plot(frequencies(1:1:300),20*log10(magnitudeResponse(1:1:300))); 
 ylim([-10,1])
figure
plot(frequencies(1:1:300), 180*angle_response(1:1:300)/pi);

