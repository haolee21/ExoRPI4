%%
% this script will calculate the momentum of human and the exoskeleton from
% either the opensim *.mot file or the *.csv from the exoskeleton

%% load opensim *.mot
%q6 = -ankle_angle_l-pelvis_tilt
%q5 = -knee_angle_l-pelvis_tilt
%q4 = -hip_flexion_l-pelvis_tilt
%q3 = hip_flexion_r-pelvis_tilt
%q2 = knee_angle_r-pelvis_tilt
%q1 = ankle_angle_r-pelvis_tilt
close all;
file_name = 'Hao_human Walk01_IK.mot';


data = readmatrix(file_name ,'FileType','text');
time = data(:,1);
pelvis_tilt = data(:,2)/180*pi;
hip_flexion_r = data(:,8)/180*pi;
knee_angle_r = data(:,11)/180*pi;
ankle_angle_r = data(:,12)/180*pi;
hip_flexion_l = data(:,15)/180*pi;
knee_angle_l = data(:,18)/180*pi;
ankle_angle_l = data(:,19)/180*pi;


q1 = ankle_angle_r+pi/2+pelvis_tilt;
q2 = knee_angle_r;%-pelvis_tilt;
q3 = hip_flexion_r;%-pelvis_tilt;
q4 = -hip_flexion_l-pi;%+pelvis_tilt;
q5 = -knee_angle_l;%-pelvis_tilt;
q6 = -ankle_angle_l-pi/2;%-pelvis_tilt;

sampT = time(2)-time(1);
dq1 = gradient(q1)/sampT;
dq2 = gradient(q2)/sampT;
dq3 = gradient(q3)/sampT;
dq4 = gradient(q4)/sampT;
dq5 = gradient(q5)/sampT;
dq6 = gradient(q6)/sampT;

p_h = zeros(2,length(q1));
p_h_segment = zeros(2*6,length(q1));
% calculate human momentum
for i=1:length(q1)
    q = [q1(i),q2(i),q3(i),q4(i),q5(i),q6(i)];
    dq = [dq1(i),dq2(i),dq3(i),dq4(i),dq5(i),dq6(i)];
    cur_p = human_load_momentum(q,dq);
    p_h(:,i) = sum(cur_p,2);
    p_h_segment(:,i) = [cur_p(:,1);cur_p(:,2);cur_p(:,3);cur_p(:,4);cur_p(:,5);cur_p(:,6)];
end
figure(1);
plot(time,p_h(1,:));
xlim([3.251,3.724]);
title('Horizontal Momentum');
figure(2);
plot(time,p_h(2,:));
% xlim([3.251,3.724]);
title('Vertical Momentum');
figure(3);
plot(time,p_h_segment(1,:));
xlim([3.251,3.724]);


figure();
plot(time,p_h_segment(1,:));
title('Seg 1 momentum');
