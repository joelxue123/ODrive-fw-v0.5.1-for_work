    motor_gear_two_posiotn = importdata("first_sencond_pos.csv");

    motor_position = motor_gear_two_posiotn.data(:, 2) - 30685;
    gear_position = motor_gear_two_posiotn.data(:, 3) - 216274;

     x = 1:1:250;
        
     plot(x,motor_position, x ,gear_position); 
     