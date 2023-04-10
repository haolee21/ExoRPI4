function p_h = human_load_momentum(in1,in2)
%HUMAN_LOAD_MOMENTUM
%    P_H = HUMAN_LOAD_MOMENTUM(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    06-Mar-2023 12:39:40

dq1 = in2(:,1);
dq2 = in2(:,2);
dq3 = in2(:,3);
dq4 = in2(:,4);
dq5 = in2(:,5);
dq6 = in2(:,6);
q1 = in1(:,1);
q2 = in1(:,2);
q3 = in1(:,3);
q4 = in1(:,4);
q5 = in1(:,5);
q6 = in1(:,6);
t2 = cos(q1);
t3 = cos(q2);
t4 = cos(q3);
t5 = cos(q4);
t6 = cos(q5);
t7 = cos(q6);
t8 = sin(q1);
t9 = sin(q2);
t10 = sin(q3);
t11 = sin(q4);
t12 = sin(q5);
t13 = sin(q6);
t14 = dq1+dq2;
t15 = dq3+t14;
t16 = t2.*t3;
t17 = t2.*t9;
t18 = t3.*t8;
t19 = t8.*t9;
t38 = t2.*4.4835e-2;
t39 = t8.*4.4835e-2;
t20 = dq4+t15;
t22 = -t19;
t24 = t17+t18;
t40 = t16.*4.4835e-2;
t41 = t17.*4.4835e-2;
t42 = t18.*4.4835e-2;
t43 = t19.*4.4835e-2;
t21 = dq5+t20;
t25 = t16+t22;
t26 = t4.*t24;
t27 = t10.*t24;
t44 = -t43;
t57 = t39+t41+t42;
t23 = dq6+t21;
t28 = t4.*t25;
t29 = t10.*t25;
t30 = -t27;
t58 = t38+t40+t44;
t31 = t26+t29;
t32 = t28+t30;
t35 = -t5.*(t27-t28);
t36 = -t11.*(t27-t28);
t48 = t5.*(t27-t28).*(-4.4835e-2);
t49 = t11.*(t27-t28).*(-4.4835e-2);
t33 = t5.*t31;
t34 = t11.*t31;
t37 = -t34;
t45 = t33.*4.4835e-2;
t46 = t34.*4.4835e-2;
t50 = t33+t36;
t54 = -t12.*(t34+t5.*(t27-t28));
t55 = -t6.*(t34+t5.*(t27-t28));
t56 = t6.*(t34+t5.*(t27-t28));
t47 = -t46;
t51 = t35+t37;
t52 = t12.*t50;
t53 = t6.*t50;
t59 = t53+t54;
t60 = t52+t56;
p_h = reshape([dq1.*t2.*(-6.3191121525e-2),dq1.*t8.*6.3191121525e-2,t2.*t14.*(-3.13845e-1)-t14.*t25.*1.35894885e-1,t8.*t14.*3.13845e-1+t14.*t24.*1.35894885e-1,t15.*t31.*(-4.080174016608792e-1)-t15.*t58.*6.333583e+1+t15.*(t27-t28).*2.23869151106622,t15.*t31.*2.23869151106622+t15.*t57.*6.333583e+1+t15.*(t27-t28).*4.080174016608792e-1,t20.*(t34+t5.*(t27-t28)).*1.77950115e-1-t20.*t58.*7.0,t20.*t50.*1.77950115e-1+t20.*t57.*7.0,t21.*t60.*8.2746803475e-2+t21.*(-t38-t40+t43+t46+t5.*(t27-t28).*4.4835e-2).*(6.51e+2./2.0e+2),t21.*t59.*8.2746803475e-2+t21.*(t45+t49+t57).*(6.51e+2./2.0e+2),t23.*(-t38-t40+t43+t46+t52.*4.4835e-2+t56.*4.4835e-2+t5.*(t27-t28).*4.4835e-2).*(2.03e+2./2.0e+2)+t23.*(t7.*t60+t13.*t59).*3.660439565999999e-3,t23.*(t45+t49+t53.*4.4835e-2+t57-t12.*(t34+t5.*(t27-t28)).*4.4835e-2).*(2.03e+2./2.0e+2)+t23.*(t7.*t59-t13.*t60).*3.660439565999999e-3],[2,6]);
