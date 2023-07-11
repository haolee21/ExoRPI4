function [q,dq,dLq_F,dLdq_F,dLq_B,dLdq_B,rotM,rotM_hb]=GenHuman_Exo(DH,m_h,m_exo,I_h,I_exo,l,lc_h,lc_exo)
%% calculate the forward dynamics of a human_exo model, for both forward
% knee and backward knee, return the symbolic variable for joints

numJ = size(DH,1);
g = [0,9.81,0];
%% generate symbolic variables
q =sym('q',[1,numJ]);

dq = sym('dq',[1,numJ]);

assume(q,'real');
assume(dq,'real');
q_hb = [q(1)+q(2),-q(2),q(2)+q(3),q(4)+q(5),-q(5),q(5)+q(6)]; % this is used when we need to generate human model for backward knee
dq_hb = [dq(1)+dq(2),-dq(2),dq(2)+dq(3),dq(4)+dq(5),-dq(5),dq(5)+dq(6)];


%% calculate each joint's translational matrix, joint velocity, CoM velocity
% wrt the base frame
curRot = eye(4);
curRot_hb=eye(4);
cur_w = sym(zeros(3,1));
cur_w_hb=sym(zeros(3,1));
cur_JV = sym(zeros(3,1)); % velocity of joint origin wrt base
cur_JV_hb = sym(zeros(3,1));% velocity of joint origin wrt base, human backward knee


rotM = cell(1,numJ);
rotM_hb=cell(1,numJ);
w =cell(1,numJ);
w_hb = cell(1,numJ);
vc_hF = cell(1,numJ);
vc_hB=cell(1,numJ);
vc_e = cell(1,numJ);
for i=1:numJ
    a = DH(i,2);
    d = DH(i,3);
    al = DH(i,1);
    curRot = curRot*T_matrix(a,al,d,q(1,i));
    curRot_hb =curRot_hb*T_matrix(a,al,d,q_hb(1,i));
    cur_w = curRot(1:3,1:3)*[0;0;dq(1,i)]+cur_w;
    cur_w_hb = curRot_hb(1:3,1:3)*[0;0;dq_hb(1,i)]+cur_w_hb;
    
    vc_hF{1,i} = simplify(cur_JV+curRot(1:3,1:3)*cross(cur_w,lc_h(:,i))); %the joint origin velocity 
    vc_hB{1,i} = simplify(cur_JV_hb+curRot_hb(1:3,1:3)*cross(cur_w_hb,lc_h(:,i)));
    vc_e{1,i} = simplify(cur_JV+curRot(1:3,1:3)*cross(cur_w,lc_exo(:,i))); % we calculate from exoskeleton pov, so exo shares the same cur_JV as forward knee
    
    
    rotM{1,i} = curRot;
    rotM_hb{1,i}=curRot_hb;
    w{1,i}=cur_w;
    w_hb{1,i} = cur_w_hb;
    
    cur_JV = cur_JV+simplify(curRot(1:3,1:3)*cross(cur_w,l(:,i))); %the joint origin velocity is update here since at the base, the vel =0
    cur_JV_hb = cur_JV_hb + simplify(curRot_hb(1:3,1:3)*cross(cur_w_hb,l(:,i)));
end


%% calculate the lagrangian 
dLq_F = sym(zeros(numJ,numJ));
dLq_B = sym(zeros(numJ,numJ));
dLdq_F = sym(zeros(numJ,numJ));
dLdq_B = sym(zeros(numJ,numJ));

for i=1:numJ
    %potential energy
    cur_lc_hF = rotM{1,i}*[lc_h(:,i);1];  % center of mass location of human limb, forward knee
    cur_lc_hB = rotM_hb{1,i}*[lc_h(:,i);1]; % center of mass location of human limb, backward knee
    cur_lc_e = rotM{1,i}*[lc_exo(:,i);1]; % center of mass location of exo link, exo does not need to consider forward/backward knee since exo's config is always the base
    
    V_cur_F = m_h(1,i)*g*cur_lc_hF(1:3,1)+m_exo(1,i)*g*cur_lc_e(1:3,1);
    V_cur_B = m_h(1,i)*g*cur_lc_hB(1:3,1)+m_exo(1,i)*g*cur_lc_e(1:3,1);
    
    %kinetic energy
    %rotational energy
    T_cur_F1 = 0.5*simplify(w{1,i}.'*(I_h{1,i}+I_exo{1,i})*w{1,i});
    T_cur_B1 = 0.5*simplify(w_hb{1,i}.'*I_h{1,i}*w_hb{1,i}+w{1,i}.'*I_exo{1,i}*w{1,i});
    
    %translational energy
    T_cur_F2 = 0.5*simplify((m_h(1,i)*vc_hF{1,i}.'*vc_hF{1,i}+m_exo(1,i)*vc_e{1,i}.'*vc_e{1,i}));
    T_cur_B2 = 0.5*simplify((m_h(1,i)*vc_hB{1,i}.'*vc_hB{1,i}+m_exo(1,i)*vc_e{1,i}.'*vc_e{1,i}));
    
    T_cur_F = T_cur_F1+T_cur_F2;
    T_cur_B = T_cur_B1+T_cur_B2;
    
    
    L_F = T_cur_F-V_cur_F;
    L_B = T_cur_B-V_cur_B;
    
    
    for c=1:numJ %since we have backward knee, the gradient of knee will be related to the ankle (q2 related to q1)
        dLq_F(c,i) = simplify(diff(L_F,q(1,c)));
        dLdq_F(c,i) = simplify(diff(L_F,dq(1,c)));
        
        dLq_B(c,i) = simplify(diff(L_B,q(1,c)));
        dLdq_B(c,i) = simplify(diff(L_B,dq(1,c)));
    end
    
end
dLq_F = sum(dLq_F,2);
dLdq_F = sum(dLdq_F,2);

dLq_B = sum(dLq_B,2);
dLdq_B = sum(dLdq_B,2);







end

function T = T_matrix(a,alpha,d,theta)

T = [cos(theta)               -sin(theta)                 0                  a;
     sin(theta)*cos(alpha)    cos(theta)*cos(alpha)       -sin(alpha)        -d*sin(alpha);
     sin(theta)*sin(alpha)    cos(theta)*sin(alpha)       cos(alpha)         d*cos(alpha);
     0                        0                           0                  1];

end