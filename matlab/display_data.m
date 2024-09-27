data = importdata("BoosterMotorEncosTestencos-241102-5hz.csv");

x = data.data(:,13);
y = data.data(:,3)*180/3.1415;
z = data.data(:,6)*180/3.1415;

plot(x,y,x,z,x,(y-z));
legend('Actual Position', 'Target Position', 'Position Error');
title('encos241102 - 5HZ');



    


