close all;
clear all;
%%
mocap_file_name = 'Hao_exo Act01_IK.mot';
mocap_data = readmatrix(mocap_file_name,'FileType','text');
l_enc_data = readmatrix('EncodersL_0.csv');
r_enc_data = readmatrix('EncodersR_0.csv');

q1_e = r_enc_data(:,4)/180*pi+pi/2;
q2_e = r_enc_data(:,3)/180*pi;
q3_e = -r_enc_data(:,2)/180*pi;
q4_e = -l_enc_data(:,2)/180*pi+pi;
q5_e = -l_enc_data(:,3)/180*pi;
q6_e = -l_enc_data(:,4)-pi/2;
enc_time = l_enc_data(:,1)/100;
dq1_e = gradient(q1_e)*100;
dq2_e = gradient(q2_e)*100;
dq3_e = gradient(q3_e)*100;
dq4_e = gradient(q4_e)*100;
dq5_e = gradient(q5_e)*100;
dq6_e = gradient(q6_e)*100;

test = exo_com_pos([q1_e(1),q2_e(1),q3_e(1),q4_e(1),q5_e(1),q6_e(1)]);
joint_pos = exo_joint_pos([q1_e(1),q2_e(1),q3_e(1),q4_e(1),q5_e(1),q6_e(1)]);
figure();
hold on;
axis equal;
plot(test(1,1),test(2,1),'r*');
plot(test(1,2),test(2,2),'r*');
plot(test(1,3),test(2,3),'r*');
plot(test(1,4),test(2,4),'r*');
plot(test(1,5),test(2,5),'r*');
plot(test(1,6),test(2,6),'r*');


plot(joint_pos(1,1),joint_pos(2,1),'go')
plot(joint_pos(1,2),joint_pos(2,2),'go')
plot(joint_pos(1,3),joint_pos(2,3),'go')
plot(joint_pos(1,4),joint_pos(2,4),'go')
plot(joint_pos(1,5),joint_pos(2,5),'go')
plot(joint_pos(1,6),joint_pos(2,6),'go')


hold off;


mocap_lkne_ang = mocap_data(:,26)/180*pi;
mocap_rkne_ang = mocap_data(:,28)/180*pi;
mocap_time = mocap_data(:,1);

pelvis_tilt = mocap_data(:,2)/180*pi;
hip_flexion_r = mocap_data(:,8)/180*pi;
knee_angle_r = mocap_data(:,11)/180*pi;
ankle_angle_r = mocap_data(:,12)/180*pi;
hip_flexion_l = mocap_data(:,15)/180*pi;
knee_angle_l = mocap_data(:,18)/180*pi;
ankle_angle_l = mocap_data(:,19)/180*pi;

figure();
plot(mocap_time,pelvis_tilt*180/pi,'b');
hold on
plot(mocap_time,hip_flexion_r*180/pi,'g');
plot(mocap_time,knee_angle_r*180/pi,'r');
plot(mocap_time,ankle_angle_r*180/pi,'c');
plot(mocap_time,hip_flexion_l*180/pi,'m');
plot(mocap_time,knee_angle_l*180/pi,'y');
plot(mocap_time,ankle_angle_l*180/pi,'k');
legend('pelvis','hip_r','knee_r','ankle_r','hip_l','knee_l','ankle_l');
title('Mocap Angles');
hold off

q1_h = ankle_angle_r;%-pelvis_tilt;
q2_h = knee_angle_r;%-pelvis_tilt;
q3_h = hip_flexion_r;%-pelvis_tilt;
q4_h = -hip_flexion_l-pi;%+pelvis_tilt;
q5_h = -knee_angle_l;%-pelvis_tilt;
q6_h = -ankle_angle_l-pi/2;%-pelvis_tilt;

figure();
plot(mocap_time,q3_h*180/pi,'g');
hold on
plot(mocap_time,q2_h*180/pi,'r');
plot(mocap_time,q1_h*180/pi,'c');
plot(mocap_time,-q4_h*180/pi,'m');
plot(mocap_time,-q5_h*180/pi,'y');
plot(mocap_time,-q6_h*180/pi,'k');
legend('hip_r','knee_r','ankle_r','hip_l','knee_l','ankle_l');
title('Mocap Angles Normalized to Pelvis');
hold off

sampT = mocap_time(2)-mocap_time(1);
dq1_h = gradient(q1_h)/sampT;
dq2_h = gradient(q2_h)/sampT;
dq3_h = gradient(q3_h)/sampT;
dq4_h = gradient(q4_h)/sampT;
dq5_h = gradient(q5_h)/sampT;
dq6_h = gradient(q6_h)/sampT;

test = human_no_load_com_pos([q1_h(1),q2_h(1),q3_h(1),q4_h(1),q5_h(1),q6_h(1)]);
% test_q = [45,0,0,0,0,0]/180*pi;
% test = human_no_load_com_pos(test_q);
figure();
hold on;
axis equal;
plot(test(1,1),test(2,1),'r*');
plot(test(1,2),test(2,2),'r*');
plot(test(1,3),test(2,3),'r*');
plot(test(1,4),test(2,4),'r*');
plot(test(1,5),test(2,5),'r*');
plot(test(1,6),test(2,6),'r*');

hold off;


l_enc_time = l_enc_data(:,1);
enc_lhip_ang = l_enc_data(:,2)/180*pi;
enc_lkne_ang = l_enc_data(:,3)/180*pi;
enc_lank_ang = l_enc_data(:,4)/180*pi;
r_enc_time = r_enc_data(:,1);
enc_rhip_ang = r_enc_data(:,2)/180*pi;
enc_rkne_ang = r_enc_data(:,3)/180*pi;
enc_rank_ang = r_enc_data(:,4)/180*pi;

time_offset = 6.06;
figure();
plot(mocap_time,mocap_lkne_ang);
figure();
plot(l_enc_time/100,enc_lkne_ang);


figure();
hold on;
plot(mocap_time,mocap_lkne_ang);
plot(l_enc_time/100 - time_offset,enc_lkne_ang);
xlim([5,10]);
legend('mocap','enc');
hold off;

figure();
hold on;
plot(mocap_time,mocap_rkne_ang);
plot(r_enc_time/100-time_offset,enc_rkne_ang);
xlim([5,10]);
legend('mocap','enc');
hold off;

%% momentum of human
p_h = zeros(2,length(q1_h));
p_h_segment = zeros(2*6,length(q1_h));
% calculate human momentum
for i=1:length(q1_h)
    q = [q1_h(i),q2_h(i),q3_h(i),q4_h(i),q5_h(i),q6_h(i)];
    dq = [dq1_h(i),dq2_h(i),dq3_h(i),dq4_h(i),dq5_h(i),dq6_h(i)];
    cur_p = human_no_load_momentum(q,dq);
    p_h(:,i) = sum(cur_p,2);
    p_h_segment(:,i) = [cur_p(:,1);cur_p(:,2);cur_p(:,3);cur_p(:,4);cur_p(:,5);cur_p(:,6)];
end

figure();
plot(mocap_time,p_h(1,:))
title('horizontal momentum');
% xlim([7.3,8.2]);
figure();
plot(mocap_time,p_h(2,:));
title('Vertical Momentum');
xlim([7.3,8.2]);


% momentum of exo
p_e = zeros(2,length(q1_e));
p_e_segment = zeros(2*6,length(q1_e));
% calculate human momentum
for i=1:length(q1_e)
    q = [q1_e(i),q2_e(i),q3_e(i),q4_e(i),q5_e(i),q6_e(i)];
    dq = [dq1_e(i),dq2_e(i),dq3_e(i),dq4_e(i),dq5_e(i),dq6_e(i)];
    cur_p = exo_momentum(q,dq);
    p_e(:,i) = sum(cur_p,2);
    p_e_segment(:,i) = [cur_p(:,1);cur_p(:,2);cur_p(:,3);cur_p(:,4);cur_p(:,5);cur_p(:,6)];
end

figure(1);
plot(enc_time,p_e(1,:));
title('exo horizontal momentum');
xlim([7.3+time_offset,8.2+time_offset]);
figure(2);
plot(enc_time,p_e(2,:));
title('exo vertical momentum');
xlim([7.3+time_offset,8.2+time_offset]);
% 
% figure();
% plot(mocap_time,p_h_segment(1,:));
% title('seg 1 momentum');
% xlim([7.3,8.2]);
% 
% figure();
% plot(mocap_time,p_h_segment(3,:));
% title('seg 2 momentum');
% xlim([7.3,8.2]);
% 
% figure();
% plot(mocap_time,p_h_segment(5,:));
% title('seg 3 momentum');
% xlim([7.3,8.2]);
% 
% figure();
% plot(mocap_time,p_h_segment(7,:));
% title('seg 4 momentum');
% xlim([7.3,8.2]);
% 
% figure();
% plot(mocap_time,p_h_segment(9,:));
% title('seg 5 momentum');
% xlim([7.3,8.2]);
% 
% figure();
% plot(mocap_time,p_h_segment(11,:));
% title('seg 6 momentum');
% xlim([7.3,8.2]);






