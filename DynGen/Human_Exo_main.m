function Human_Exo_main(modelName,modelType,extra_load,exo_enable,totH,totM,gen_shared)
% I use function since we need to use a few subfunctions
%% Generate robot with discrete Lagrangian
% the dynamics of forward/backward knee

% clear;

% modelName='human_10';
% totM=65;
% extra_load = 20;
% totH=1.83;
numJ=6;


%% Mass and length of human

% for here I will redefine the foot length, the total foot length is
% 0.1476*totH, yet, the ankle joint is not at the end of it, I divide foot
% into l_foot, l_heel horizontally, and the height is h_heel

% base on my own body measurement, l_foot is 0.5556 of the whole foot, and
% l_heel is 0.14815
l_foot = 0.152*totH*0.5556; %use de Leva number to get the percentage, 0.7143 is if I exclude toe
l_heel = 0.152*totH*0.14815;
h_heel = 0.039*totH;  

l_calf = 0.245*totH; % on paper it should be 0.246, but I use 0.245 to make it the same length as thigh, which can help us solve backward knee easily
l_thigh =0.245*totH;
l_torso = 0.34*totH; %head + torso, will have to adjust lc_torso later


DH = [0, 0, 0, 0, 0, 0;...
      0, l_calf, 0, 0, 0, 0;...
      0, l_thigh, 0, 0, 0, 0;...
      0,  0, 0, 0, 0, 0;...
      0,l_thigh, 0, 0, 0, 0;...
      0,l_calf,0,0,0,0];
  
%m_hand = 0.006*totM;
%m_upper_arm = 0.028*totM;
%m_fore_arm = 0.016*totM;
m_foot = 0.0145*totM;
m_calf =0.0465*totM;
m_thigh = 0.1*totM;
m_head = 0.081*totM;
%m_trunk = 0.497*totM;
m_torso = 0.678*totM+extra_load; % the mass of load is directly added on the torso

if exo_enable==1
    m_exo_calf = 1.7;
    m_exo_thigh=1.7;
    m_exo_foot = 0.8;
else
    m_exo_calf=0;
    m_exo_thigh=0;
    m_exo_foot=0;
end


model.totM = (m_foot+m_calf+m_thigh+m_exo_calf+m_exo_thigh+m_exo_foot)*2+m_torso; %it should be the same as totM, just in case there are some roundoff errors
model.h_heel = h_heel;
model.l_heel = l_heel;
model.l_foot = l_foot;
model.l_calf = l_calf;
model.l_thigh = l_thigh;
model.m_foot = m_foot;
model.m_calf = m_calf;
model.m_thigh = m_thigh;
model.m_torso = m_torso;
model.totH = totH;
model.m_exo_calf = m_exo_calf;
model.m_exo_thigh = m_exo_thigh;
model.m_exo_foot = m_exo_foot;

if ~exist(['../',modelName,'/',modelType],'dir')
    mkdir(['../',modelName,'/',modelType]);
end
save(['../',modelName,'/',modelType,'/model.mat'],'model');

%CoM pos
lc_thigh2_h = 0.433*l_thigh;
lc_calf2_h = 0.433*l_calf;
lc_calf1_h = l_calf-lc_calf2_h;
lc_thigh1_h = l_thigh-lc_thigh2_h;

lc_foot_h = 0.5*l_foot-l_heel; % de Leva, shift by l_heel since the joint is not at the end of foot 
% include head when calculating, suppose neck is fixed

lc_thigh_e = 0.5*l_thigh;
lc_calf_e = 0.5*l_calf;
lc_foot_e = 0.5*l_foot-l_heel;


lc_torso = (0.5*l_torso*m_torso+(0.1166*totH*(1-0.5976)+l_torso)*m_head)/(m_head+m_torso); % recalculate since we need to consider the load

I_calf_h = [0,0,0;     %ROG data from "Comparison of direct collocation optimal control", same as what I got from David A. Winter's 
          0,0,0;
          0,0,m_calf*l_calf^2*0.302^2];
I_calf_e = [0,0,0;
            0,0,0;
            0,0,m_exo_calf*l_calf^2/12];
I_thigh_h = [0,0,0;
              0,0,0;
              0,0,m_thigh*l_thigh^2*0.323^2];
I_thigh_e = [0,0,0;
             0,0,0;
             0,0,m_exo_thigh*l_thigh^2/12];
I_torso = [0,0,0;
           0,0,0;
           0,0,m_torso*l_torso^2*0.496^2];

I_foot_h = [0,0,0;
          0,0,0;
          0,0,m_foot*(0.152*totH*0.475)^2];% we need to use the original foot length, not the one removed toe
      
I_foot_e = [0,0,0;
            0,0,0;
            0,0,m_exo_foot*(0.152*totH)^2/12];% we need to use the original foot length, not the one removed toe

m_h = [m_calf,m_thigh,m_torso,m_thigh,m_calf,m_foot];
m_exo=[m_exo_calf,m_exo_thigh,0,m_exo_thigh,m_exo_calf,m_exo_foot]; %m_exo_torso =0 since it is already included in m_torso
I_h = {I_calf_h,I_thigh_h,I_torso,I_thigh_h,I_calf_h,I_foot_h};
I_exo={I_calf_e,I_thigh_e,zeros(3),I_thigh_e,I_calf_e,I_foot_e};

l=[l_calf,l_thigh,l_torso,l_thigh,l_calf,l_foot,-l_heel; % when calculating dynamics, only the first 5 will be used, so don't worry about the last 2
        0,      0,      0,      0,     0,h_heel, h_heel;
        0,      0,      0,      0,     0,     0,     0];


lc_h=[lc_calf1_h,lc_thigh1_h,lc_torso,lc_thigh2_h,lc_calf2_h,lc_foot_h;
               0,          0,       0,          0,         0,   h_heel;
               0,          0,       0,          0,         0,        0];
lc_exo=[lc_calf_e,lc_thigh_e,0,lc_thigh_e,lc_calf_e,lc_foot_e;
                0,         0,0,         0,        0,   h_heel;
                0,         0,0,         0,        0,       0];

[q,dq,dLq_F,dLdq_F,dLq_B,dLdq_B,rotM,rotM_hb]=GenHuman_Exo(DH,m_h,m_exo,I_h,I_exo,l,lc_h,lc_exo);
 

sampT = sym('sampT');

dL1_F = 0.5*dLq_F-dLdq_F/sampT;
dL2_F = 0.5*dLq_F+dLdq_F/sampT;

dL1_B = 0.5*dLq_B-dLdq_B/sampT;
dL2_B = 0.5*dLq_B+dLdq_B/sampT;


% the return values are the Discrete Lagrangian
% dL1 means dL(x_k,x_k+1)/dx_k
% dL2 means dL(x_k-1,x_k)/dx_k  (always takes derivative at x_k, 1,2 just
% menas the order of x_k)

% in other words, for dL1, it is taking derivative on q1, but dL2 is on q2
if ~exist(['../',modelName,'/',modelType,'/dyn/forward'],'dir')
    mkdir(['../',modelName,'/',modelType,'/dyn/forward']);
end
if exo_enable
    if ~exist(['../',modelName,'/',modelType,'/dyn/backward'],'dir')
        mkdir(['../',modelName,'/',modelType,'/dyn/backward']);
    end
end


tasks{1,1} =@()matlabFunction(dL1_F(1,1),'file',['../',modelName,'/',modelType,'/dyn/forward/dL1_1'],'vars',{q,dq,sampT}); 
tasks{1,2} =@()matlabFunction(dL1_F(2,1),'file',['../',modelName,'/',modelType,'/dyn/forward/dL1_2'],'vars',{q,dq,sampT}); 
tasks{1,3} =@()matlabFunction(dL1_F(3,1),'file',['../',modelName,'/',modelType,'/dyn/forward/dL1_3'],'vars',{q,dq,sampT}); 
tasks{1,4} =@()matlabFunction(dL1_F(4,1),'file',['../',modelName,'/',modelType,'/dyn/forward/dL1_4'],'vars',{q,dq,sampT});
tasks{1,5} =@()matlabFunction(dL1_F(5,1),'file',['../',modelName,'/',modelType,'/dyn/forward/dL1_5'],'vars',{q,dq,sampT}); 
tasks{1,6} =@()matlabFunction(dL1_F(6,1),'file',['../',modelName,'/',modelType,'/dyn/forward/dL1_6'],'vars',{q,dq,sampT}); 


tasks{1,7} =@()matlabFunction(dL2_F(1,1),'file',['../',modelName,'/',modelType,'/dyn/forward/dL2_1'],'vars',{q,dq,sampT});
tasks{1,8} =@()matlabFunction(dL2_F(2,1),'file',['../',modelName,'/',modelType,'/dyn/forward/dL2_2'],'vars',{q,dq,sampT}); 
tasks{1,9} =@()matlabFunction(dL2_F(3,1),'file',['../',modelName,'/',modelType,'/dyn/forward/dL2_3'],'vars',{q,dq,sampT}); 
tasks{1,10} =@()matlabFunction(dL2_F(4,1),'file',['../',modelName,'/',modelType,'/dyn/forward/dL2_4'],'vars',{q,dq,sampT});
tasks{1,11} =@()matlabFunction(dL2_F(5,1),'file',['../',modelName,'/',modelType,'/dyn/forward/dL2_5'],'vars',{q,dq,sampT});
tasks{1,12} =@()matlabFunction(dL2_F(6,1),'file',['../',modelName,'/',modelType,'/dyn/forward/dL2_6'],'vars',{q,dq,sampT}); 

if exo_enable
    tasks{1,13} =@()matlabFunction(dL1_B(1,1),'file',['../',modelName,'/',modelType,'/dyn/backward/dL1_1'],'vars',{q,dq,sampT});
    tasks{1,14} =@()matlabFunction(dL1_B(2,1),'file',['../',modelName,'/',modelType,'/dyn/backward/dL1_2'],'vars',{q,dq,sampT});
    tasks{1,15} =@()matlabFunction(dL1_B(3,1),'file',['../',modelName,'/',modelType,'/dyn/backward/dL1_3'],'vars',{q,dq,sampT});
    tasks{1,16} =@()matlabFunction(dL1_B(4,1),'file',['../',modelName,'/',modelType,'/dyn/backward/dL1_4'],'vars',{q,dq,sampT});
    tasks{1,17} =@()matlabFunction(dL1_B(5,1),'file',['../',modelName,'/',modelType,'/dyn/backward/dL1_5'],'vars',{q,dq,sampT});
    tasks{1,18} =@()matlabFunction(dL1_B(6,1),'file',['../',modelName,'/',modelType,'/dyn/backward/dL1_6'],'vars',{q,dq,sampT});
    
    
    tasks{1,19} =@()matlabFunction(dL2_B(1,1),'file',['../',modelName,'/',modelType,'/dyn/backward/dL2_1'],'vars',{q,dq,sampT});
    tasks{1,20} =@()matlabFunction(dL2_B(2,1),'file',['../',modelName,'/',modelType,'/dyn/backward/dL2_2'],'vars',{q,dq,sampT});
    tasks{1,21} =@()matlabFunction(dL2_B(3,1),'file',['../',modelName,'/',modelType,'/dyn/backward/dL2_3'],'vars',{q,dq,sampT});
    tasks{1,22} =@()matlabFunction(dL2_B(4,1),'file',['../',modelName,'/',modelType,'/dyn/backward/dL2_4'],'vars',{q,dq,sampT});
    tasks{1,23} =@()matlabFunction(dL2_B(5,1),'file',['../',modelName,'/',modelType,'/dyn/backward/dL2_5'],'vars',{q,dq,sampT});
    tasks{1,24} =@()matlabFunction(dL2_B(6,1),'file',['../',modelName,'/',modelType,'/dyn/backward/dL2_6'],'vars',{q,dq,sampT});
end

% now we need to generate the derivative of it

tasks2 = Gen_dL_dq(dL1_F,dL2_F,q,dq,sampT,modelName,modelType,'forward');
if exo_enable
    tasks3 = Gen_dL_dq(dL1_B,dL2_B,q,dq,sampT,modelName,modelType,'backward');
end


%% 
if gen_shared==1


%% position data
% l=[l_calf,l_thigh,l_torso,l_thigh,l_calf,l_foot,-l_heel; % when calculating dynamics, only the first 5 will be used, so don't worry about the last 2
%         0,      0,      0,      0,     0,h_heel, h_heel;
%         0,      0,      0,      0,     0,     0,     0];

posMat = [1,0,0,0;
          0,1,0,0;
          0,0,1,0]; % the result is always 4x1 vector, we exclude the last 1

knee_front = posMat*rotM{1,1}*[l(:,1);1];
hip_front = posMat*rotM{1,2}*[l(:,2);1];
head = posMat*rotM{1,3}*[l(:,3);1];
knee_back = posMat*rotM{1,4}*[l(:,4);1];
ankle_back =posMat*rotM{1,5}*[l(:,5);1];
toe_back = posMat*rotM{1,6}*[l(:,6);1];
heel_back = posMat*rotM{1,6}*[l(:,7);1];

knee_front_hb = posMat*rotM_hb{1,1}*[l(:,1);1];
hip_front_hb = posMat*rotM_hb{1,2}*[l(:,2);1];
head_hb = posMat*rotM_hb{1,3}*[l(:,3);1];
knee_back_hb = posMat*rotM_hb{1,4}*[l(:,4);1];
ankle_back_hb =posMat*rotM_hb{1,5}*[l(:,5);1];

% Com
% lc_h=[lc_calf1_h,lc_thigh1_h,lc_torso,lc_thigh2_h,lc_calf2_h,lc_foot_h;
%                0,          0,       0,          0,         0,   h_heel;
%                0,          0,       0,          0,         0,        0];
% lc_exo=[lc_calf_e,lc_thigh_e,0,lc_thigh_e,lc_calf_e,lc_foot_e;
%                 0,         0,0,         0,        0,   h_heel;
%                 0,         0,0,         0,        0,       0];


calf_front_hf = posMat*rotM{1,1}*[lc_h(:,1);1];
thigh_front_hf =posMat*rotM{1,2}*[lc_h(:,2);1];
torso = posMat*rotM{1,3}*[lc_h(:,3);1];
thigh_back_hf = posMat*rotM{1,4}*[lc_h(:,4);1];
calf_back_hf = posMat*rotM{1,5}*[lc_h(:,5);1];
foot_back_hf = posMat*rotM{1,6}*[lc_h(:,6);1];

calf_front_e = posMat*rotM{1,1}*[lc_exo(:,1);1];
thigh_front_e =posMat*rotM{1,2}*[lc_exo(:,2);1];
thigh_back_e = posMat*rotM{1,4}*[lc_exo(:,4);1];
calf_back_e = posMat*rotM{1,5}*[lc_exo(:,5);1];
foot_back_e = posMat*rotM{1,6}*[lc_exo(:,6);1];

calf_front_hb = posMat*rotM_hb{1,1}*[lc_h(:,1);1];
thigh_front_hb =posMat*rotM_hb{1,2}*[lc_h(:,2);1];
torso_hb =      posMat*rotM_hb{1,3}*[lc_h(:,3);1];
thigh_back_hb = posMat*rotM_hb{1,4}*[lc_h(:,4);1];
calf_back_hb =  posMat*rotM_hb{1,5}*[lc_h(:,5);1];
foot_back_hb =  posMat*rotM_hb{1,6}*[lc_h(:,6);1];

jointPos_f = [knee_front,hip_front,head,knee_back,ankle_back,toe_back,heel_back];

jointPos_hb=[knee_front_hb,hip_front_hb,head_hb,knee_back_hb,ankle_back_hb];

com_hf = [calf_front_hf,thigh_front_hf,torso,thigh_back_hf,calf_back_hf,foot_back_hf];

com_hb = [calf_front_hb,thigh_front_hb,torso_hb,thigh_back_hb,calf_back_hb,foot_back_hb];

com_e = [calf_front_e,thigh_front_e,thigh_back_e,calf_back_e,foot_back_e];

if ~exist(['../',modelName,'/graph'],'dir')
    mkdir(['../',modelName,'/graph']);
end


tasks4{1,1} = @()matlabFunction(jointPos_f,'file',['../',modelName,'/graph/jointPos_f'],'vars',{q});
tasks4{1,2} = @()matlabFunction(jointPos_hb,'file',['../',modelName,'/graph/jointPos_hb'],'vars',{q});
tasks4{1,3} = @()matlabFunction(com_hf,'file',['../',modelName,'/graph/com_hf'],'vars',{q});
tasks4{1,4} = @()matlabFunction(com_hb,'file',['../',modelName,'/graph/com_hb'],'vars',{q});
tasks4{1,5} = @()matlabFunction(com_e,'file',['../',modelName,'/graph/com_e'],'vars',{q});


%% calculate Jacobian
% Jacobian at the hip, this is for making sure it won't go backward
if ~exist(['../',modelName,'/pos/'],'dir')
    mkdir(['../',modelName,'/pos/']);
end

hipPos = posMat*rotM{1,3}*[0;0;0;1];
[J_hip,tasks5_1]=Cal_jacob(hipPos,q,numJ,modelName,'hip');
[J_toe,tasks5_2]=Cal_jacob(toe_back,q,numJ,modelName,'toe');
[J_heel,tasks5_3]=Cal_jacob(heel_back,q,numJ,modelName,'heel');






%% calculate position constraints and it's gradient

tasks5_4 = PosGrad(toe_back(1,1),q,modelName,'ToePos_x');
tasks5_5 = PosGrad(toe_back(2,1),q,modelName,'ToePos_y');

tasks5_6 = PosGrad(heel_back(1,1),q,modelName,'HeelPos_x');
tasks5_7 = PosGrad(heel_back(2,1),q,modelName,'HeelPos_y');

tasks5 = {tasks5_1,tasks5_2,tasks5_3,tasks5_4,tasks5_5,tasks5_6,tasks5_7};
tasks5 = cat(2,tasks5{:});


%% calculate external force

% conditions for ground reaction force
% to make this symbolic calculation simplier, we use dq, although it is
% actually (q2-q1)/sampT

toe_vel = J_toe(1:2,:)*dq.';
heel_vel = J_heel(1:2,:)*dq.';

tasks6=cell(1,44);

toe_vel_x = toe_vel(1);
toe_vel_y = toe_vel(2);
heel_vel_x =heel_vel(1);
heel_vel_y = heel_vel(2);

if ~exist(['../',modelName,'/grf'],'dir')
    mkdir(['../',modelName,'/grf']);
end

% save toe_vel_x/heel_vel_x for sliding constraints
tasks6{1,1} = @()matlabFunction(toe_vel_x,'file',['../',modelName,'/grf/toeVel_x'],'vars',{q,dq,sampT});
tasks6{1,2} = @()matlabFunction(heel_vel_x,'file',['../',modelName,'/grf/heelVel_x'],'vars',{q,dq,sampT});

dToe_vel_xdq = [diff(toe_vel_x,q(1));
                diff(toe_vel_x,q(2));
                diff(toe_vel_x,q(3));
                diff(toe_vel_x,q(4));
                diff(toe_vel_x,q(5));
                diff(toe_vel_x,q(6))];
dToe_vel_xddq = [diff(toe_vel_x,dq(1));
                 diff(toe_vel_x,dq(2));
                 diff(toe_vel_x,dq(3));
                 diff(toe_vel_x,dq(4));
                 diff(toe_vel_x,dq(5));
                 diff(toe_vel_x,dq(6))];
dToe_vel_xdq1 = 0.5*dToe_vel_xdq-dToe_vel_xddq/sampT;
dToe_vel_xdq2 = 0.5*dToe_vel_xdq+dToe_vel_xddq/sampT;

tasks6{1,3} = @()matlabFunction(dToe_vel_xdq1,'file',['../',modelName,'/grf/dToe_vel_dq1'],'vars',{q,dq,sampT});
tasks6{1,4} = @()matlabFunction(dToe_vel_xdq2,'file',['../',modelName,'/grf/dToe_vel_dq2'],'vars',{q,dq,sampT});

dHeel_vel_xdq = [diff(heel_vel_x,q(1));
                 diff(heel_vel_x,q(2));
                 diff(heel_vel_x,q(3));
                 diff(heel_vel_x,q(4));
                 diff(heel_vel_x,q(5));
                 diff(heel_vel_x,q(6))];
dHeel_vel_xddq = [diff(heel_vel_x,dq(1));
                  diff(heel_vel_x,dq(2));
                  diff(heel_vel_x,dq(3));
                  diff(heel_vel_x,dq(4));
                  diff(heel_vel_x,dq(5));
                  diff(heel_vel_x,dq(6))];
dHeel_vel_xdq1 = 0.5*dHeel_vel_xdq-dHeel_vel_xddq/sampT;
dHeel_vel_xdq2 = 0.5*dHeel_vel_xdq+dHeel_vel_xddq/sampT;
tasks6{1,5} = @()matlabFunction(dHeel_vel_xdq1,'file',['../',modelName,'/grf/dHeel_vel_dq1'],'vars',{q,dq,sampT});
tasks6{1,6} = @()matlabFunction(dHeel_vel_xdq2,'file',['../',modelName,'/grf/dHeel_vel_dq2'],'vars',{q,dq,sampT});




syms Fx Fy real
syms k cmax dmax us H real;
ks =2;
Fy_toe = k*(H-toe_back(2,1))^ks -cmax*(H-toe_back(2,1))/dmax*toe_vel_y;
Fy_heel = k*(H-heel_back(2,1))^ks -cmax*(H-heel_back(2,1))/dmax*heel_vel_y;


tasks6{1,7} =@()matlabFunction(Fy_toe,'file',['../',modelName,'/grf/Fy_toe'],'vars',{q,dq,H,k,cmax,dmax,sampT}); 
tasks6{1,8} =@()matlabFunction(Fy_heel,'file',['../',modelName,'/grf/Fy_heel'],'vars',{q,dq,H,k,cmax,dmax,sampT}); 

dFy_toe_dq = [diff(Fy_toe,q(1));
              diff(Fy_toe,q(2));
              diff(Fy_toe,q(3));
              diff(Fy_toe,q(4));
              diff(Fy_toe,q(5));
              diff(Fy_toe,q(6))];
dFy_toe_ddq = [diff(Fy_toe,dq(1));
               diff(Fy_toe,dq(2));
               diff(Fy_toe,dq(3));
               diff(Fy_toe,dq(4));
               diff(Fy_toe,dq(5));
               diff(Fy_toe,dq(6))];
dFy_toe_q1 = 0.5*dFy_toe_dq-dFy_toe_ddq/sampT;
dFy_toe_q2 = 0.5*dFy_toe_dq+dFy_toe_ddq/sampT;

tasks6{1,9} =@()matlabFunction(dFy_toe_q1,'file',['../',modelName,'/grf/dFy_toe1'],'vars',{q,dq,H,k,cmax,dmax,sampT}); 
tasks6{1,10} =@()matlabFunction(dFy_toe_q2,'file',['../',modelName,'/grf/dFy_toe2'],'vars',{q,dq,H,k,cmax,dmax,sampT}); 


dFy_heel_dq = [diff(Fy_heel,q(1));
              diff(Fy_heel,q(2));
              diff(Fy_heel,q(3));
              diff(Fy_heel,q(4));
              diff(Fy_heel,q(5));
              diff(Fy_heel,q(6))];
dFy_heel_ddq = [diff(Fy_heel,dq(1));
               diff(Fy_heel,dq(2));
               diff(Fy_heel,dq(3));
               diff(Fy_heel,dq(4));
               diff(Fy_heel,dq(5));
               diff(Fy_heel,dq(6))];
dFy_heel_q1 = 0.5*dFy_heel_dq-dFy_heel_ddq/sampT;
dFy_heel_q2 = 0.5*dFy_heel_dq+dFy_heel_ddq/sampT;
tasks6{1,11} =@()matlabFunction(dFy_heel_q1,'file',['../',modelName,'/grf/dFy_heel1'],'vars',{q,dq,H,k,cmax,dmax,sampT}); 
tasks6{1,12} =@()matlabFunction(dFy_heel_q2,'file',['../',modelName,'/grf/dFy_heel2'],'vars',{q,dq,H,k,cmax,dmax,sampT}); 





% external ground reaction Tau for ankle push-off phase
Tau_toe_pushoff = J_toe(1:2,:).'*[Fx;Fy];
dTau_toe_pushoff_dq = [diff(Tau_toe_pushoff.',q(1));
                       diff(Tau_toe_pushoff.',q(2));
                       diff(Tau_toe_pushoff.',q(3));
                       diff(Tau_toe_pushoff.',q(4));
                       diff(Tau_toe_pushoff.',q(5));
                       diff(Tau_toe_pushoff.',q(6))];
dTau_toe_pushoff_ddq = [diff(Tau_toe_pushoff.',dq(1));
                        diff(Tau_toe_pushoff.',dq(2));
                        diff(Tau_toe_pushoff.',dq(3));
                        diff(Tau_toe_pushoff.',dq(4));
                        diff(Tau_toe_pushoff.',dq(5));
                        diff(Tau_toe_pushoff.',dq(6))];
dTau_toe_pushoff_dq1 =0.5*dTau_toe_pushoff_dq-dTau_toe_pushoff_ddq/sampT;
dTau_toe_pushoff_dq2 = 0.5*dTau_toe_pushoff_dq+dTau_toe_pushoff_ddq/sampT;
dTau_toe_pushoff_dfx = diff(Tau_toe_pushoff.',Fx);
dTau_toe_pushoff_dfy = diff(Tau_toe_pushoff.',Fy);
tasks6{1,13} =@()matlabFunction(Tau_toe_pushoff,'file',['../',modelName,'/grf/Tau_toe_pushoff'],'vars',{q,dq,Fx,Fy,H,k,cmax,dmax,sampT}); 
tasks6{1,14} =@()matlabFunction(dTau_toe_pushoff_dq1,'file',['../',modelName,'/grf/dTau_toe_pushoff_dq1'],'vars',{q,dq,Fx,Fy,H,k,cmax,dmax,sampT}); 
tasks6{1,15} =@()matlabFunction(dTau_toe_pushoff_dq2,'file',['../',modelName,'/grf/dTau_toe_pushoff_dq2'],'vars',{q,dq,Fx,Fy,H,k,cmax,dmax,sampT}); 
tasks6{1,16} =@()matlabFunction(dTau_toe_pushoff_dfx,'file',['../',modelName,'/grf/dTau_toe_pushoff_dfx'],'vars',{q,dq,Fx,Fy,H,k,cmax,dmax,sampT});
tasks6{1,17} =@()matlabFunction(dTau_toe_pushoff_dfy,'file',['../',modelName,'/grf/dTau_toe_pushoff_dfy'],'vars',{q,dq,Fx,Fy,H,k,cmax,dmax,sampT});



% constraint 1 and 2: Fx<us*Fy, -Fx<us*Fy
Grf_toe_c1 = Fx-us*Fy_toe;
Grf_toe_c2 = -Fx-us*Fy_toe;

Grf_heel_c1 = Fx-us*Fy_heel;
Grf_heel_c2 = -Fx-us*Fy_heel;


dGrf_toe_c1_dq = [diff(Grf_toe_c1,q(1));
                  diff(Grf_toe_c1,q(2));
                  diff(Grf_toe_c1,q(3));
                  diff(Grf_toe_c1,q(4));
                  diff(Grf_toe_c1,q(5));
                  diff(Grf_toe_c1,q(6))];
dGrf_toe_c1_ddq = [diff(Grf_toe_c1,dq(1));
                  diff(Grf_toe_c1,dq(2));
                  diff(Grf_toe_c1,dq(3));
                  diff(Grf_toe_c1,dq(4));
                  diff(Grf_toe_c1,dq(5));
                  diff(Grf_toe_c1,dq(6))];               
dGrf_toe_c1_dq1 = 0.5*dGrf_toe_c1_dq-dGrf_toe_c1_ddq/sampT;
dGrf_toe_c1_dq2 = 0.5*dGrf_toe_c1_dq+dGrf_toe_c1_ddq/sampT;
dGrf_toe_c1_dfx = diff(Grf_toe_c1,Fx);

dGrf_heel_c1_dq = [diff(Grf_heel_c1,q(1));
                  diff(Grf_heel_c1,q(2));
                  diff(Grf_heel_c1,q(3));
                  diff(Grf_heel_c1,q(4));
                  diff(Grf_heel_c1,q(5));
                  diff(Grf_heel_c1,q(6))];
dGrf_heel_c1_ddq = [diff(Grf_heel_c1,dq(1));
                  diff(Grf_heel_c1,dq(2));
                  diff(Grf_heel_c1,dq(3));
                  diff(Grf_heel_c1,dq(4));
                  diff(Grf_heel_c1,dq(5));
                  diff(Grf_heel_c1,dq(6))]; 
dGrf_heel_c1_dq1 = 0.5*dGrf_heel_c1_dq-dGrf_heel_c1_ddq/sampT;
dGrf_heel_c1_dq2 = 0.5*dGrf_heel_c1_dq+dGrf_heel_c1_ddq/sampT;
dGrf_heel_c1_dfx = diff(Grf_heel_c1,Fx);





dGrf_toe_c2_dq = [diff(Grf_toe_c2,q(1));
                  diff(Grf_toe_c2,q(2));
                  diff(Grf_toe_c2,q(3));
                  diff(Grf_toe_c2,q(4));
                  diff(Grf_toe_c2,q(5));
                  diff(Grf_toe_c2,q(6))];
dGrf_toe_c2_ddq = [diff(Grf_toe_c2,dq(1));
                  diff(Grf_toe_c2,dq(2));
                  diff(Grf_toe_c2,dq(3));
                  diff(Grf_toe_c2,dq(4));
                  diff(Grf_toe_c2,dq(5));
                  diff(Grf_toe_c2,dq(6))];               
dGrf_toe_c2_dq1 = 0.5*dGrf_toe_c2_dq-dGrf_toe_c2_ddq/sampT;
dGrf_toe_c2_dq2 = 0.5*dGrf_toe_c2_dq+dGrf_toe_c2_ddq/sampT;
dGrf_toe_c2_dfx = diff(Grf_toe_c2,Fx);

dGrf_heel_c2_dq = [diff(Grf_heel_c2,q(1));
                  diff(Grf_heel_c2,q(2));
                  diff(Grf_heel_c2,q(3));
                  diff(Grf_heel_c2,q(4));
                  diff(Grf_heel_c2,q(5));
                  diff(Grf_heel_c2,q(6))];
dGrf_heel_c2_ddq = [diff(Grf_heel_c2,dq(1));
                  diff(Grf_heel_c2,dq(2));
                  diff(Grf_heel_c2,dq(3));
                  diff(Grf_heel_c2,dq(4));
                  diff(Grf_heel_c2,dq(5));
                  diff(Grf_heel_c2,dq(6))];               
dGrf_heel_c2_dq1 = 0.5*dGrf_heel_c2_dq-dGrf_heel_c2_ddq/sampT;
dGrf_heel_c2_dq2 = 0.5*dGrf_heel_c2_dq+dGrf_heel_c2_ddq/sampT;
dGrf_heel_c2_dfx = diff(Grf_heel_c2,Fx);

tasks6{1,18} =@()matlabFunction(Grf_toe_c1,'file',['../',modelName,'/grf/Grf_toe_c1'],'vars',{q,dq,Fx,H,k,cmax,dmax,us}); 
tasks6{1,19} =@()matlabFunction(Grf_heel_c1,'file',['../',modelName,'/grf/Grf_heel_c1'],'vars',{q,dq,Fx,H,k,cmax,dmax,us}); 

tasks6{1,20} =@()matlabFunction(dGrf_toe_c1_dq1,'file',['../',modelName,'/grf/dGrf_toe_c1_dq1'],'vars',{q,dq,H,k,cmax,dmax,us,sampT}); 
tasks6{1,21} =@()matlabFunction(dGrf_toe_c1_dq2,'file',['../',modelName,'/grf/dGrf_toe_c1_dq2'],'vars',{q,dq,H,k,cmax,dmax,us,sampT}); 
tasks6{1,22} =@()matlabFunction(dGrf_toe_c1_dfx,'file',['../',modelName,'/grf/dGrf_toe_c1_dfx'],'vars',Fx); 

tasks6{1,23} =@()matlabFunction(dGrf_heel_c1_dq1,'file',['../',modelName,'/grf/dGrf_heel_c1_dq1'],'vars',{q,dq,H,k,cmax,dmax,us,sampT}); 
tasks6{1,24} =@()matlabFunction(dGrf_heel_c1_dq2,'file',['../',modelName,'/grf/dGrf_heel_c1_dq2'],'vars',{q,dq,H,k,cmax,dmax,us,sampT}); 
tasks6{1,25} =@()matlabFunction(dGrf_heel_c1_dfx,'file',['../',modelName,'/grf/dGrf_heel_c1_dfx'],'vars',Fx); 

tasks6{1,26} =@()matlabFunction(Grf_toe_c2,'file',['../',modelName,'/grf/Grf_toe_c2'],'vars',{q,dq,Fx,H,k,cmax,dmax,us}); 
tasks6{1,27} =@()matlabFunction(Grf_heel_c2,'file',['../',modelName,'/grf/Grf_heel_c2'],'vars',{q,dq,Fx,H,k,cmax,dmax,us}); 

tasks6{1,28} =@()matlabFunction(dGrf_toe_c2_dq1,'file',['../',modelName,'/grf/dGrf_toe_c2_dq1'],'vars',{q,dq,H,k,cmax,dmax,us,sampT}); 
tasks6{1,29} =@()matlabFunction(dGrf_toe_c2_dq2,'file',['../',modelName,'/grf/dGrf_toe_c2_dq2'],'vars',{q,dq,H,k,cmax,dmax,us,sampT}); 
tasks6{1,30} =@()matlabFunction(dGrf_toe_c2_dfx,'file',['../',modelName,'/grf/dGrf_toe_c2_dfx'],'vars',Fx); 

tasks6{1,31} =@()matlabFunction(dGrf_heel_c2_dq1,'file',['../',modelName,'/grf/dGrf_heel_c2_dq1'],'vars',{q,dq,H,k,cmax,dmax,us,sampT}); 
tasks6{1,32} =@()matlabFunction(dGrf_heel_c2_dq2,'file',['../',modelName,'/grf/dGrf_heel_c2_dq2'],'vars',{q,dq,H,k,cmax,dmax,us,sampT}); 
tasks6{1,33} =@()matlabFunction(dGrf_heel_c2_dfx,'file',['../',modelName,'/grf/dGrf_heel_c2_dfx'],'vars',Fx); 

% constraint 4
% xvel*Fx <=0  (Fx is always acting on the opposite direction of xvel)
Grf_toe_c4 = toe_vel_x*Fx;
Grf_heel_c4 = heel_vel_x*Fx;

tasks6{1,34}=@()matlabFunction(Grf_toe_c4,'file',['../',modelName,'/grf/Grf_toe_c4'],'vars',{q,dq,Fx}); 
tasks6{1,35}=@()matlabFunction(Grf_heel_c4,'file',['../',modelName,'/grf/Grf_heel_c4'],'vars',{q,dq,Fx}); 

dGrf_toe_c4q = [diff(Grf_toe_c4,q(1));
                diff(Grf_toe_c4,q(2));
                diff(Grf_toe_c4,q(3));
                diff(Grf_toe_c4,q(4));
                diff(Grf_toe_c4,q(5));
                diff(Grf_toe_c4,q(6))];

dGrf_toe_c4qd=[diff(Grf_toe_c4,dq(1));
               diff(Grf_toe_c4,dq(2));
               diff(Grf_toe_c4,dq(3));
               diff(Grf_toe_c4,dq(4));
               diff(Grf_toe_c4,dq(5));
               diff(Grf_toe_c4,dq(6))];
dGrf_toe_c4_q1 = 0.5*dGrf_toe_c4q-dGrf_toe_c4qd/sampT;
dGrf_toe_c4_q2 = 0.5*dGrf_toe_c4q+dGrf_toe_c4qd/sampT;

dGrf_heel_c4q = [diff(Grf_heel_c4,q(1));
                diff(Grf_heel_c4,q(2));
                diff(Grf_heel_c4,q(3));
                diff(Grf_heel_c4,q(4));
                diff(Grf_heel_c4,q(5));
                diff(Grf_heel_c4,q(6))];

dGrf_heel_c4qd=[diff(Grf_heel_c4,dq(1));
               diff(Grf_heel_c4,dq(2));
               diff(Grf_heel_c4,dq(3));
               diff(Grf_heel_c4,dq(4));
               diff(Grf_heel_c4,dq(5));
               diff(Grf_heel_c4,dq(6))];
dGrf_heel_c4_q1 = 0.5*dGrf_heel_c4q-dGrf_heel_c4qd/sampT;
dGrf_heel_c4_q2 = 0.5*dGrf_heel_c4q+dGrf_heel_c4qd/sampT;



tasks6{1,36}=@()matlabFunction(dGrf_toe_c4_q1,'file',['../',modelName,'/grf/dGrf_toe_c4_q1'],'vars',{q,dq,Fx,sampT});
tasks6{1,37}=@()matlabFunction(dGrf_toe_c4_q2,'file',['../',modelName,'/grf/dGrf_toe_c4_q2'],'vars',{q,dq,Fx,sampT}); 

tasks6{1,38}=@()matlabFunction(dGrf_heel_c4_q1,'file',['../',modelName,'/grf/dGrf_heel_c4_q1'],'vars',{q,dq,Fx,sampT}); 
tasks6{1,39}=@()matlabFunction(dGrf_heel_c4_q2,'file',['../',modelName,'/grf/dGrf_heel_c4_q2'],'vars',{q,dq,Fx,sampT}); 

dGrf_toe_c4_Fx = diff(Grf_toe_c4,Fx);
                 
dGrf_heel_c4_Fx = diff(Grf_heel_c4,Fx);
              
tasks6{1,40}=@()matlabFunction(dGrf_toe_c4_Fx,'file',['../',modelName,'/grf/dGrf_toe_c4_F'],'vars',{q,dq}); 
tasks6{1,41}=@()matlabFunction(dGrf_heel_c4_Fx,'file',['../',modelName,'/grf/dGrf_heel_c4_F'],'vars',{q,dq}); 



%generate hip velocity constraint
hip_vel_x = J_hip*dq.';
hip_vel_x = hip_vel_x(1);
dHip_vel_xq = [diff(hip_vel_x,q(1));
               diff(hip_vel_x,q(2));
               diff(hip_vel_x,q(3));
               diff(hip_vel_x,q(4));
               diff(hip_vel_x,q(5));
               diff(hip_vel_x,q(6))];
dHip_vel_xdq =[diff(hip_vel_x,dq(1));
               diff(hip_vel_x,dq(2));
               diff(hip_vel_x,dq(3));
               diff(hip_vel_x,dq(4));
               diff(hip_vel_x,dq(5));
               diff(hip_vel_x,dq(6))];
dHip_vel_q1 =0.5*dHip_vel_xq-dHip_vel_xdq/sampT; %in the form q2-q1
dHip_vel_q2 =0.5*dHip_vel_xq+dHip_vel_xdq/sampT;   

tasks6{1,42}=@()matlabFunction(hip_vel_x ,'file',['../',modelName,'/pos/hip_vel_x'],'vars',{q,dq,sampT}); 
tasks6{1,43}=@()matlabFunction(dHip_vel_q1,'file',['../',modelName,'/pos/dHip_vel_q1'],'vars',{q,dq,sampT}); 
tasks6{1,44}=@()matlabFunction(dHip_vel_q2,'file',['../',modelName,'/pos/dHip_vel_q2'],'vars',{q,dq,sampT}); 



if exo_enable
    allTasks = {tasks,tasks2,tasks3,tasks4,tasks5,tasks6};
else
    allTasks = {tasks,tasks2,tasks4,tasks5,tasks6};
end






else
    if exo_enable
        allTasks = {tasks,tasks2,tasks3};
    else
        allTasks = {tasks,tasks2};
    end
end
allTasks = cat(2,allTasks{:});
parfor i=1:length(allTasks)
    allTasks{1,i}();
end
end
function tasks=Gen_dL_dq(dL1,dL2,q,dq,sampT,modelName,modelType,knee_config)

dL11_dq = [diff(dL1(1,1),q(1));diff(dL1(1,1),q(2));diff(dL1(1,1),q(3));diff(dL1(1,1),q(4));diff(dL1(1,1),q(5));diff(dL1(1,1),q(6))];
dL12_dq = [diff(dL1(2,1),q(1));diff(dL1(2,1),q(2));diff(dL1(2,1),q(3));diff(dL1(2,1),q(4));diff(dL1(2,1),q(5));diff(dL1(2,1),q(6))];
dL13_dq = [diff(dL1(3,1),q(1));diff(dL1(3,1),q(2));diff(dL1(3,1),q(3));diff(dL1(3,1),q(4));diff(dL1(3,1),q(5));diff(dL1(3,1),q(6))];
dL14_dq = [diff(dL1(4,1),q(1));diff(dL1(4,1),q(2));diff(dL1(4,1),q(3));diff(dL1(4,1),q(4));diff(dL1(4,1),q(5));diff(dL1(4,1),q(6))];
dL15_dq = [diff(dL1(5,1),q(1));diff(dL1(5,1),q(2));diff(dL1(5,1),q(3));diff(dL1(5,1),q(4));diff(dL1(5,1),q(5));diff(dL1(5,1),q(6))];
dL16_dq = [diff(dL1(6,1),q(1));diff(dL1(6,1),q(2));diff(dL1(6,1),q(3));diff(dL1(6,1),q(4));diff(dL1(6,1),q(5));diff(dL1(6,1),q(6))];

dL11_ddq = [diff(dL1(1,1),dq(1));diff(dL1(1,1),dq(2));diff(dL1(1,1),dq(3));diff(dL1(1,1),dq(4));diff(dL1(1,1),dq(5));diff(dL1(1,1),dq(6))];
dL12_ddq = [diff(dL1(2,1),dq(1));diff(dL1(2,1),dq(2));diff(dL1(2,1),dq(3));diff(dL1(2,1),dq(4));diff(dL1(2,1),dq(5));diff(dL1(2,1),dq(6))];
dL13_ddq = [diff(dL1(3,1),dq(1));diff(dL1(3,1),dq(2));diff(dL1(3,1),dq(3));diff(dL1(3,1),dq(4));diff(dL1(3,1),dq(5));diff(dL1(3,1),dq(6))];
dL14_ddq = [diff(dL1(4,1),dq(1));diff(dL1(4,1),dq(2));diff(dL1(4,1),dq(3));diff(dL1(4,1),dq(4));diff(dL1(4,1),dq(5));diff(dL1(4,1),dq(6))];
dL15_ddq = [diff(dL1(5,1),dq(1));diff(dL1(5,1),dq(2));diff(dL1(5,1),dq(3));diff(dL1(5,1),dq(4));diff(dL1(5,1),dq(5));diff(dL1(5,1),dq(6))];
dL16_ddq = [diff(dL1(6,1),dq(1));diff(dL1(6,1),dq(2));diff(dL1(6,1),dq(3));diff(dL1(6,1),dq(4));diff(dL1(6,1),dq(5));diff(dL1(6,1),dq(6))];

dL11_dq1 = dL11_dq*0.5-dL11_ddq/sampT;
dL12_dq1 = dL12_dq*0.5-dL12_ddq/sampT;
dL13_dq1 = dL13_dq*0.5-dL13_ddq/sampT;
dL14_dq1 = dL14_dq*0.5-dL14_ddq/sampT;
dL15_dq1 = dL15_dq*0.5-dL15_ddq/sampT;
dL16_dq1 = dL16_dq*0.5-dL16_ddq/sampT;

dL11_dq2 = dL11_dq*0.5+dL11_ddq/sampT;
dL12_dq2 = dL12_dq*0.5+dL12_ddq/sampT;
dL13_dq2 = dL13_dq*0.5+dL13_ddq/sampT;
dL14_dq2 = dL14_dq*0.5+dL14_ddq/sampT;
dL15_dq2 = dL15_dq*0.5+dL15_ddq/sampT;
dL16_dq2 = dL16_dq*0.5+dL16_ddq/sampT;

dL21_dq = [diff(dL2(1,1),q(1));diff(dL2(1,1),q(2));diff(dL2(1,1),q(3));diff(dL2(1,1),q(4));diff(dL2(1,1),q(5));diff(dL2(1,1),q(6))];
dL22_dq = [diff(dL2(2,1),q(1));diff(dL2(2,1),q(2));diff(dL2(2,1),q(3));diff(dL2(2,1),q(4));diff(dL2(2,1),q(5));diff(dL2(2,1),q(6))];
dL23_dq = [diff(dL2(3,1),q(1));diff(dL2(3,1),q(2));diff(dL2(3,1),q(3));diff(dL2(3,1),q(4));diff(dL2(3,1),q(5));diff(dL2(3,1),q(6))];
dL24_dq = [diff(dL2(4,1),q(1));diff(dL2(4,1),q(2));diff(dL2(4,1),q(3));diff(dL2(4,1),q(4));diff(dL2(4,1),q(5));diff(dL2(4,1),q(6))];
dL25_dq = [diff(dL2(5,1),q(1));diff(dL2(5,1),q(2));diff(dL2(5,1),q(3));diff(dL2(5,1),q(4));diff(dL2(5,1),q(5));diff(dL2(5,1),q(6))];
dL26_dq = [diff(dL2(6,1),q(1));diff(dL2(6,1),q(2));diff(dL2(6,1),q(3));diff(dL2(6,1),q(4));diff(dL2(6,1),q(5));diff(dL2(6,1),q(6))];

dL21_ddq = [diff(dL2(1,1),dq(1));diff(dL2(1,1),dq(2));diff(dL2(1,1),dq(3));diff(dL2(1,1),dq(4));diff(dL2(1,1),dq(5));diff(dL2(1,1),dq(6))];
dL22_ddq = [diff(dL2(2,1),dq(1));diff(dL2(2,1),dq(2));diff(dL2(2,1),dq(3));diff(dL2(2,1),dq(4));diff(dL2(2,1),dq(5));diff(dL2(2,1),dq(6))];
dL23_ddq = [diff(dL2(3,1),dq(1));diff(dL2(3,1),dq(2));diff(dL2(3,1),dq(3));diff(dL2(3,1),dq(4));diff(dL2(3,1),dq(5));diff(dL2(3,1),dq(6))];
dL24_ddq = [diff(dL2(4,1),dq(1));diff(dL2(4,1),dq(2));diff(dL2(4,1),dq(3));diff(dL2(4,1),dq(4));diff(dL2(4,1),dq(5));diff(dL2(4,1),dq(6))];
dL25_ddq = [diff(dL2(5,1),dq(1));diff(dL2(5,1),dq(2));diff(dL2(5,1),dq(3));diff(dL2(5,1),dq(4));diff(dL2(5,1),dq(5));diff(dL2(5,1),dq(6))];
dL26_ddq = [diff(dL2(6,1),dq(1));diff(dL2(6,1),dq(2));diff(dL2(6,1),dq(3));diff(dL2(6,1),dq(4));diff(dL2(6,1),dq(5));diff(dL2(6,1),dq(6))];

dL21_dq1 = dL21_dq*0.5-dL21_ddq/sampT;
dL22_dq1 = dL22_dq*0.5-dL22_ddq/sampT;
dL23_dq1 = dL23_dq*0.5-dL23_ddq/sampT;
dL24_dq1 = dL24_dq*0.5-dL24_ddq/sampT;
dL25_dq1 = dL25_dq*0.5-dL25_ddq/sampT;
dL26_dq1 = dL26_dq*0.5-dL26_ddq/sampT;

dL21_dq2 = dL21_dq*0.5+dL21_ddq/sampT;
dL22_dq2 = dL22_dq*0.5+dL22_ddq/sampT;
dL23_dq2 = dL23_dq*0.5+dL23_ddq/sampT;
dL24_dq2 = dL24_dq*0.5+dL24_ddq/sampT;
dL25_dq2 = dL25_dq*0.5+dL25_ddq/sampT;
dL26_dq2 = dL26_dq*0.5+dL26_ddq/sampT;

tasks = cell(1,24);
tasks{1,1} =@()matlabFunction(dL11_dq1,'file',['../',modelName,'/',modelType,'/dyn/',knee_config,'/dL11_dq1'],'vars',{q,dq,sampT}); 
tasks{1,2} =@()matlabFunction(dL12_dq1,'file',['../',modelName,'/',modelType,'/dyn/',knee_config,'/dL12_dq1'],'vars',{q,dq,sampT});
tasks{1,3} =@()matlabFunction(dL13_dq1,'file',['../',modelName,'/',modelType,'/dyn/',knee_config,'/dL13_dq1'],'vars',{q,dq,sampT});
tasks{1,4} =@()matlabFunction(dL14_dq1,'file',['../',modelName,'/',modelType,'/dyn/',knee_config,'/dL14_dq1'],'vars',{q,dq,sampT}); 
tasks{1,5} =@()matlabFunction(dL15_dq1,'file',['../',modelName,'/',modelType,'/dyn/',knee_config,'/dL15_dq1'],'vars',{q,dq,sampT});
tasks{1,6} =@()matlabFunction(dL16_dq1,'file',['../',modelName,'/',modelType,'/dyn/',knee_config,'/dL16_dq1'],'vars',{q,dq,sampT}); 

tasks{1,7} =@()matlabFunction(dL11_dq2,'file',['../',modelName,'/',modelType,'/dyn/',knee_config,'/dL11_dq2'],'vars',{q,dq,sampT});
tasks{1,8} =@()matlabFunction(dL12_dq2,'file',['../',modelName,'/',modelType,'/dyn/',knee_config,'/dL12_dq2'],'vars',{q,dq,sampT}); 
tasks{1,9} =@()matlabFunction(dL13_dq2,'file',['../',modelName,'/',modelType,'/dyn/',knee_config,'/dL13_dq2'],'vars',{q,dq,sampT}); 
tasks{1,10} =@()matlabFunction(dL14_dq2,'file',['../',modelName,'/',modelType,'/dyn/',knee_config,'/dL14_dq2'],'vars',{q,dq,sampT});
tasks{1,11} =@()matlabFunction(dL15_dq2,'file',['../',modelName,'/',modelType,'/dyn/',knee_config,'/dL15_dq2'],'vars',{q,dq,sampT}); 
tasks{1,12} =@()matlabFunction(dL16_dq2,'file',['../',modelName,'/',modelType,'/dyn/',knee_config,'/dL16_dq2'],'vars',{q,dq,sampT}); 

tasks{1,13} =@()matlabFunction(dL21_dq1,'file',['../',modelName,'/',modelType,'/dyn/',knee_config,'/dL21_dq1'],'vars',{q,dq,sampT});
tasks{1,14} =@()matlabFunction(dL22_dq1,'file',['../',modelName,'/',modelType,'/dyn/',knee_config,'/dL22_dq1'],'vars',{q,dq,sampT}); 
tasks{1,15} =@()matlabFunction(dL23_dq1,'file',['../',modelName,'/',modelType,'/dyn/',knee_config,'/dL23_dq1'],'vars',{q,dq,sampT});
tasks{1,16} =@()matlabFunction(dL24_dq1,'file',['../',modelName,'/',modelType,'/dyn/',knee_config,'/dL24_dq1'],'vars',{q,dq,sampT}); 
tasks{1,17} =@()matlabFunction(dL25_dq1,'file',['../',modelName,'/',modelType,'/dyn/',knee_config,'/dL25_dq1'],'vars',{q,dq,sampT});
tasks{1,18} =@()matlabFunction(dL26_dq1,'file',['../',modelName,'/',modelType,'/dyn/',knee_config,'/dL26_dq1'],'vars',{q,dq,sampT});

tasks{1,19} =@()matlabFunction(dL21_dq2,'file',['../',modelName,'/',modelType,'/dyn/',knee_config,'/dL21_dq2'],'vars',{q,dq,sampT}); 
tasks{1,20} =@()matlabFunction(dL22_dq2,'file',['../',modelName,'/',modelType,'/dyn/',knee_config,'/dL22_dq2'],'vars',{q,dq,sampT}); 
tasks{1,21} =@()matlabFunction(dL23_dq2,'file',['../',modelName,'/',modelType,'/dyn/',knee_config,'/dL23_dq2'],'vars',{q,dq,sampT}); 
tasks{1,22} =@()matlabFunction(dL24_dq2,'file',['../',modelName,'/',modelType,'/dyn/',knee_config,'/dL24_dq2'],'vars',{q,dq,sampT}); 
tasks{1,23} =@()matlabFunction(dL25_dq2,'file',['../',modelName,'/',modelType,'/dyn/',knee_config,'/dL25_dq2'],'vars',{q,dq,sampT});
tasks{1,24} =@()matlabFunction(dL26_dq2,'file',['../',modelName,'/',modelType,'/dyn/',knee_config,'/dL26_dq2'],'vars',{q,dq,sampT}); 
end
function [J,tasks]=Cal_jacob(pos,q,numJ,modelName,name)
J = sym(zeros(2,numJ));
for i=1:numJ
    for k=1:2
        J(k,i) = simplify(diff(pos(k,1),q(i)));
    end
end

dJ_q1 = diff(J,q(1));
dJ_q2 = diff(J,q(2));
dJ_q3 = diff(J,q(3));
dJ_q4 = diff(J,q(4));
dJ_q5 = diff(J,q(5));
dJ_q6 = diff(J,q(6));

tasks = cell(1,7);
tasks{1,1} =@()matlabFunction(J,'file',['../',modelName,'/pos/J_',name],'vars',{q}); 

tasks{1,2} =@()matlabFunction(dJ_q1,'file',['../',modelName,'/pos/dJ_',name,'1'],'vars',{q}); 
tasks{1,3} =@()matlabFunction(dJ_q2,'file',['../',modelName,'/pos/dJ_',name,'2'],'vars',{q}); 
tasks{1,4} =@()matlabFunction(dJ_q3,'file',['../',modelName,'/pos/dJ_',name,'3'],'vars',{q}); 
tasks{1,5} =@()matlabFunction(dJ_q4,'file',['../',modelName,'/pos/dJ_',name,'4'],'vars',{q}); 
tasks{1,6} =@()matlabFunction(dJ_q5,'file',['../',modelName,'/pos/dJ_',name,'5'],'vars',{q}); 
tasks{1,7} =@()matlabFunction(dJ_q6,'file',['../',modelName,'/pos/dJ_',name,'6'],'vars',{q}); 

end
function tasks = PosGrad(pos,q,modelName,name)
dPos_q = [diff(pos,q(1));diff(pos,q(2));diff(pos,q(3));diff(pos,q(4));diff(pos,q(5)); diff(pos,q(6))];


tasks = cell(1,2);
tasks{1,1} =@()matlabFunction(pos,'file',['../',modelName,'/pos/',name],'vars',{q});

tasks{1,2} =@()matlabFunction(dPos_q,'file',['../',modelName,'/pos/d',name],'vars',{q});




end


