data = importdata("BoosterMotorEncosTestPos_0_1hz_2pi.csv");

x = data.data(:,13);
y = data.data(:,3)*180/3.1415;
z = data.data(:,6)*180/3.1415;

plot(x,y,x,z,x,(y-z));
legend('Actual Position', 'Target Position', 'Position Error');
title('QDD - 0.1HZ');



    


