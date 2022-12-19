import math

'''Calibrating the equation for Cylinder_length(joint_angle)'''
# it is defined as l = beta0+beta*angle
def GetJointLenEqn(theta_list,l_list):
    if(len(theta_list)!=l_list):
        print('theta and l have different length')
        return -1
    if(len(theta_list)!=3):
        print('please just input 3 points')
        return -1
    l1 = l_list[0]
    l2 = l_list[1]
    l3 = l_list[2]
    theta1=math.radians(theta_list[0])
    theta2=math.radians(theta_list[1])
    theta3=math.radians(theta_list[2])

    k1 = (l2**2-l1**2)/(l3**2-l2**2)
    k2 = (l3**2-l1**2)/(l3**2-l2**2)
    p1 = k1*math.sin((theta1-theta3)/2)
    p2 = k2*math.sin((theta1-theta2)/2)

    m = (theta1+theta2)/2
    n = (theta1+theta3)/2
    
    alpha = math.atan2(p1*math.sin(n)-p2*math.sin(m),p1*math.cos(n)-p2*math.cos(m))
    
    alpha1 = theta1-alpha
    alpha2 = theta2-alpha
    alpha3 = theta3-alpha
    beta1_0 = (l2^2-l1^2)/(math.cos(alpha1)-math.cos(alpha2))
    beta1_1 = (l3^2-l2^2)/(math.cos(alpha2)-math.cos(alpha3))
    beta1 = (beta1_0+beta1_1)/2
    
    beta0_0 = beta1*math.cos(alpha1)+l1^2
    beta0_1 = beta1*math.cos(alpha2)+l2^2
    beta0_2 = beta1*math.cos(alpha3)+l3^2

    beta0 = (beta0_0+beta0_1+beta0_2)/3

    return beta0,beta1/math.pi*180 #transform it back to degrees

