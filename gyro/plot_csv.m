csvdata = csvread('/home/linux/Universitaet/master/seminar-projekt/quadrocopter/screenlog.0');

figure;
subplot(2,3,1);
plot(csvdata(:,1));
title("ACC-X");

subplot(2,3,2);
plot(csvdata(:,2));
title("ACC-Y");

subplot(2,3,3);
plot(csvdata(:,3));
title("ACC-Z");

subplot(2,3,4);
plot(csvdata(:,4));
title("GYRO-X");

subplot(2,3,5);
plot(csvdata(:,5));
title("GYRO-Y");

subplot(2,3,6);
plot(csvdata(:,6));
title("GYRO-Z");