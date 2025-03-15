close all

start = 0;
goal = 0.5;
goal1 = 0.75;

w = pi/0.4;

pos = [];
vel = [];
acc = [];

pos1 = [];
vel1 = [];
acc1 = [];

for t = 0:0.01:0.4
    pos = [pos;.5 * goal * (1 - cos(w * t)) + .5 * (1 + cos(w * t)) * start];
    vel = [vel;.5 * goal * w * (sin(w * t)) + .5 * w * (-sin(w * t)) * start];
    acc = [acc;.5 * goal * w * w * (cos(w * t)) + .5 * w * w * (-cos(w * t)) * start];

    pos1 = [pos1;.5 * goal1 * (1 - cos(w * t)) + .5 * (1 + cos(w * t)) * start];
    vel1 = [vel1;.5 * goal1 * w * (sin(w * t)) + .5 * w * (-sin(w * t)) * start];
    acc1 = [acc1;.5 * goal1 * w * w * (cos(w * t)) + .5 * w * w * (-cos(w * t)) * start];
end

figure
plot(pos);
hold on
plot(pos1);

figure
plot(vel);
hold on
plot(vel1);

figure
plot(acc);
hold on
plot(acc1);
