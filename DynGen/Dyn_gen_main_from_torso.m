%% Generate the model for PERK only
%all units are in SI units (kg, m)

modelName='PERK_Exo';
task_i = 1;
all_tasks = {};
%% kinematic parameters
numJ=6;

foot_l_e = 0.24;
foot_h_e = 0.114;
calf_l_e = 0.4505;
thigh_l_e =0.438;


% DH = [0, 0, 0, 0, 0, 0;...
%       0, calf_l_e, 0, 0, 0, 0;...
%       0, thigh_l_e, 0, 0, 0, 0;...
%       0,  0, 0, 0, 0, 0;...
%       0,thigh_l_e, 0, 0, 0, 0;...
%       0,calf_l_e,0,0,0,0];

%% Dynamic parameter
% define the mass, CoM(2D) of each segments

% issue, should I consider the mass of human+exo or just exo? 
% if consider human, everyone has different foot......
% luckily, the speed is too slow to cause significant difference



m_calf_e = 1; % ori: 0.9352, just add some weight for the cables
m_thigh_e = 1.2; % ori: 1.1374
m_backpack = 10.20583; %22.5 lb
m_foot_e = 0.8242665;
% all CoM are expressed in their local frame (x,y)


% calf and thigh will be different for front and rear leg since we define
% all z is coming out of the plane

% f: front, r: rear
f_calf_com_e = [0.2557,-0.0236];% calculated from solidwork 
r_calf_com_e = [0.1968,0.0392];

f_thigh_com_e = [0.2105,0.0026];
r_thigh_com_e = [0.2274,-0.0026];

r_foot_com_e = [0.0630,-0.0642];

box_com = [0.3053,-0.2858];
% moment of inertia
calf_I_e = 0.0202;
thigh_I_e = 0.0237;
foot_I_e = 0.0086;
box_I = (0.15^2*0.37^2)*m_backpack/12;


%% build model for momentum

% we assume human and exo share the same front foot, the rear one might not
% be align due to numerical errors........
q =sym('q',[1,numJ]);
dq =sym('dq',[1,numJ]);

% exoskeleton CoM velocity amd momentum
vc_e = sym(zeros(2,numJ));
p_e = sym(zeros(2,numJ)); %momentum of exoskeleton
rcm_pos = sym(zeros(2,numJ));

l_e = [0,calf_l_e,thigh_l_e,0,thigh_l_e,calf_l_e];
rc_e  = [f_calf_com_e;
         f_thigh_com_e;
         box_com;
         r_thigh_com_e;
         r_calf_com_e;
         r_foot_com_e].';
m_e = [m_calf_e,m_thigh_e,m_backpack,m_thigh_e,m_calf_e,m_foot_e];
w = 0;
T = eye(3);
for i = 1:numJ
    T_cur =[cos(q(i)), -sin(q(i)), l_e(i);
            sin(q(i)), cos(q(i)), 0;
            0,           0,            1];
        
    T = T*T_cur;
    w = w+dq(i);
    
    rcm_pos_extend = T*[rc_e(:,i);1];
    rcm_pos(:,i)=rcm_pos_extend(1:2);
    v_extend = simplify([-1,0,0;0,1,0;0,0,1]*w*T*[rc_e(:,i);1]);
    vc_e(:,i) = v_extend(1:2,1);
    p_e(:,i) = m_e(i)*vc_e(:,i);
end
all_tasks{1,task_i} = @()matlabFunction(p_e,'file','exo_momentum_torso','vars',{q,dq});
task_i=task_i+1;
all_tasks{1,task_i} = @()matlabFunction(rcm_pos,'file','exo_com_pos','vars',q);
task_i = task_i+1;

%% Human without load
% for here I will redefine the foot length, the total foot length is
% 0.1476*totH, yet, the ankle joint is not at the end of it, I divide foot
% into l_foot, l_heel horizontally, and the height is h_heel

% base on my own body measurement, l_foot is 0.5556 of the whole foot, and
% l_heel is 0.14815

totM = 70;
totH = 0.183;


foot_l_h = 0.152*totH*0.5556; %use de Leva number to get the percentage, 0.7143 is if I exclude toe
heel_l_h = 0.152*totH*0.14815;
heel_h_h = 0.039*totH;  
calf_l_h = 0.245*totH; % on paper it should be 0.246, but I use 0.245 to make it the same length as thigh, which can help us solve backward knee easily
thigh_l_h =0.245*totH;
torso_l = 0.34*totH; %head + torso, will have to adjust lc_torso later




m_foot_h = 0.0145*totM;
m_calf_h =0.0465*totM;
m_thigh_h = 0.1*totM;
m_head_h = 0.081*totM;
m_torso_h = 0.678*totM;
m_load_h = m_backpack;

f_thigh_com_h = [0.433*thigh_l_h,0];
r_thigh_com_h = [thigh_l_h-f_thigh_com_h(1),0];
f_calf_com_h = [0.433*calf_l_h,0];
r_calf_com_h = [calf_l_h-f_calf_com_h(1),0];
r_foot_com_h = [0.5*foot_l_h-heel_l_h,0]; % de Leva, shift by l_heel since the joint is not at the end of foot 

torso_com = [(0.5*torso_l*m_torso_h+(0.1166*totH*(1-0.5976)+torso_l)*m_head_h)/(m_head_h+m_torso_h),0];
load_com = [torso_com(1),-0.05];
%combine torso and load and head
% m_torso_h = m_head_h+m_torso_h+m_backpack;
% torso_com = (torso_com*(m_head_h+m_torso_h)+load_com*m_backpack)/(m_head_h+m_torso_h+m_backpack);


calf_I_h = m_calf_h*calf_l_h^2*0.302^2;
thigh_I_h = m_thigh_h*thigh_l_h^2*0.323^2;
foot_I_h = m_foot_h*(0.152*totH*0.475)^2;



% human CoM velocity amd momentum
vc_h = sym(zeros(2,numJ));
p_h = sym(zeros(2,numJ)); %momentum of human
rcm_pos = sym(zeros(2,numJ));

l_h = [0,calf_l_h,thigh_l_h,0,thigh_l_h,calf_l_h];
rc_h = [r_calf_com_h;
        r_thigh_com_h;
        torso_com;
        r_thigh_com_h;
        r_calf_com_h;
        r_foot_com_h].';
m_h = [m_calf_h,m_thigh_h,m_torso_h,m_thigh_h,m_calf_h,m_foot_h];
w=0;
T=eye(3);
for i = 1:numJ
    T_cur =[cos(q(i)), -sin(q(i)), l_h(i);
            sin(q(i)), cos(q(i)), 0;
            0,           0,            1];
        
    T = T*T_cur;
    rcm_pos_extend = T*[rc_h(:,i);1];
    rcm_pos(:,i)=rcm_pos_extend(1:2);
    w = w+dq(i);
    v_extend = [-1,0,0;0,1,0;0,0,1]*w*T*[rc_h(:,i);1];
    vc_h(:,i) = v_extend(1:2,1);
    p_h(:,i) = m_h(i)*vc_h(:,i);
end

all_tasks{1,task_i} = @()matlabFunction(p_h,'file','human_no_load_momentum_torso','vars',{q,dq});
task_i=task_i+1;   
all_tasks{1,task_i} = @()matlabFunction(rcm_pos,'file','human_no_load_com_pos','vars',q);
task_i = task_i+1; 

%% human with load

%combine torso and load and head
m_torso_h = m_head_h+m_torso_h+m_backpack;
torso_com = (torso_com*(m_head_h+m_torso_h)+load_com*m_backpack)/(m_head_h+m_torso_h+m_backpack);


calf_I_h = m_calf_h*calf_l_h^2*0.302^2;
thigh_I_h = m_thigh_h*thigh_l_h^2*0.323^2;
foot_I_h = m_foot_h*(0.152*totH*0.475)^2;



% human CoM velocity amd momentum
vc_h = sym(zeros(2,numJ));
p_h = sym(zeros(2,numJ)); %momentum of human
rcm_pos = sym(zeros(2,numJ));
l_h = [0,calf_l_h,thigh_l_h,0,thigh_l_h,calf_l_h];
rc_h = [f_calf_com_h;
        f_thigh_com_h;
        torso_com;
        r_thigh_com_h;
        r_calf_com_h;
        r_foot_com_h].';
m_h = [m_calf_h,m_thigh_h,m_torso_h,m_thigh_h,m_calf_h,m_foot_h];
w=0;
T=eye(3);
for i = 1:numJ
    T_cur =[cos(q(i)), -sin(q(i)), l_h(i);
            sin(q(i)), cos(q(i)), 0;
            0,           0,            1];
        
    T = T*T_cur;
    rcm_pos_extend = T*[rc_h(:,i);1];
    rcm_pos(:,i)=rcm_pos_extend(1:2);
    w = w+dq(i);
    v_extend = [-1,0,0;0,1,0;0,0,1]*w*T*[rc_e(:,i);1];
    vc_h(:,i) = v_extend(1:2,1);
    p_h(:,i) = m_h(i)*vc_h(:,i);
end

all_tasks{1,task_i} = @()matlabFunction(p_h,'file','human_load_momentum_torso','vars',{q,dq});
task_i=task_i+1;   
all_tasks{1,task_i} = @()matlabFunction(rcm_pos,'file','human_load_com_pos','vars',q);
task_i = task_i+1; 


parfor i=1:length(all_tasks)
    all_tasks{1,i}();
end

























