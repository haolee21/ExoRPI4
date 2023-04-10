function rcm_pos = human_load_com_pos(in1)
%HUMAN_LOAD_COM_POS
%    RCM_POS = HUMAN_LOAD_COM_POS(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    06-Mar-2023 12:39:19

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
t14 = t2.*t3;
t15 = t2.*t9;
t16 = t3.*t8;
t17 = t8.*t9;
t33 = t2.*4.4835e-2;
t34 = t8.*4.4835e-2;
t18 = -t17;
t19 = t15+t16;
t35 = t14.*4.4835e-2;
t36 = t15.*4.4835e-2;
t37 = t16.*4.4835e-2;
t38 = t17.*4.4835e-2;
t20 = t14+t18;
t21 = t4.*t19;
t22 = t10.*t19;
t39 = -t38;
t23 = t4.*t20;
t24 = t10.*t20;
t25 = -t22;
t26 = t21+t24;
t27 = t23+t25;
t30 = -t5.*(t22-t23);
t31 = -t11.*(t22-t23);
t43 = t5.*(t22-t23).*(-4.4835e-2);
t44 = t11.*(t22-t23).*(-4.4835e-2);
t28 = t5.*t26;
t29 = t11.*t26;
t32 = -t29;
t40 = t28.*4.4835e-2;
t41 = t29.*4.4835e-2;
t45 = t28+t31;
t49 = -t12.*(t29+t5.*(t22-t23));
t50 = -t6.*(t29+t5.*(t22-t23));
t51 = t6.*(t29+t5.*(t22-t23));
t42 = -t41;
t46 = t30+t32;
t47 = t12.*t45;
t48 = t6.*t45;
t52 = t48+t49;
t53 = t47+t51;
rcm_pos = reshape([t2.*1.9413555e-2,t8.*1.9413555e-2,t14.*1.9413555e-2-t17.*1.9413555e-2+t33,t15.*1.9413555e-2+t16.*1.9413555e-2+t34,t21.*6.44212607083351e-3-t22.*3.534636731003952e-2+t23.*3.534636731003952e-2+t24.*6.44212607083351e-3+t33+t35+t39,t21.*3.534636731003952e-2+t22.*6.44212607083351e-3-t23.*6.44212607083351e-3+t24.*3.534636731003952e-2+t34+t36+t37,t29.*(-2.5421445e-2)+t33+t35+t39-t5.*(t22-t23).*2.5421445e-2,t28.*2.5421445e-2+t34+t36+t37-t11.*(t22-t23).*2.5421445e-2,t33+t35+t39+t42+t43-t47.*2.5421445e-2-t51.*2.5421445e-2,t34+t36+t37+t40+t44+t48.*2.5421445e-2-t12.*(t29+t5.*(t22-t23)).*2.5421445e-2,t33+t35+t39+t42+t43-t47.*4.4835e-2-t51.*4.4835e-2-t7.*t53.*3.606344399999999e-3-t13.*t52.*3.606344399999999e-3,t34+t36+t37+t40+t44+t48.*4.4835e-2-t12.*(t29+t5.*(t22-t23)).*4.4835e-2+t7.*t52.*3.606344399999999e-3-t13.*t53.*3.606344399999999e-3],[2,6]);
