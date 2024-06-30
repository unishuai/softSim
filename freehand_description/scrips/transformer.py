#! /usr/bin/env python3
import rospy
import math as m
from sensor_msgs.msg import JointState
from std_msgs.msg import String 

States=JointState()
States.name=['fi1-1joint', 'fi1-2joint', 'fi1-3joint', 'fi1-4joint', 'fi2-1joint',\
             'fi2-2joint', 'fi2-3joint', 'fi2-4joint', 'fi3-1joint', 'fi3-2joint',\
             'fi3-3joint', 'fi3-4joint', 'fi4-1joint', 'fi4-2joint', 'fi4-3joint',\
             'fi4-4joint', 'fi5-1joint', 'fi5-2joint', 'fi5-3joint', 'fi5-4joint', ]
States.position=[0.0, 0.0, 0.0 ,0.0, 0.0,0.0, 0.0, 0.0 ,0.0, 0.0,\
                 0.0, 0.0, 0.0 ,0.0, 0.0,0.0, 0.0, 0.0 ,0.0, 0.0]
States.velocity=[]
States.effort=[]
def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.position)
    global States
    States=data

def talker(): 
    rospy.init_node('talker', anonymous=True) 
    rospy.Subscriber("/joint_states", JointState, callback)
    pub = rospy.Publisher('/joint_states_1', JointState, queue_size=1) 
    #想只处理最新的消息，
    #实际上只需要把两个queue_size都设置成1那么系统不会缓存数据，自然处理的就是最新的消息
    rate = rospy.Rate(10)#10Hz 处理速度要小于0.1s
    global States
      
    while not rospy.is_shutdown():
        States.header.stamp = rospy.Time.now()
        #角度转换关系
        #States.position是元组不能直接写入，需要转换成list写入
        list1=list(States.position)
        list1[4-1]=trans1_3_4(list1[3-1])
        list1[8-1]=trans1_3_4(list1[7-1])
        list1[11-1],list1[12-1]=trans2_2_3_4(list1[10-1])
        list1[15-1],list1[16-1]=trans3_2_3_4(list1[14-1])
        list1[20-1]=trans1_3_4(list1[19-1])
        States.position=tuple(list1)
        #
        pub.publish(States)
        rospy.loginfo("Realstates is %s",States.position)
        rate.sleep()

def trans1_3_4(degree_3):#模组1 关节3转4
    A=9.5;
    B=32.35;
    C=29.867;
    D=7.3;
    initial_degree1=m.radians(61.4845);
    initial_degree2=m.radians(118.1266);
    L1=m.sqrt(A**2+C**2-2*A*C*m.cos(degree_3+initial_degree1));
    parameter1=(C-A*m.cos(degree_3+initial_degree1))/L1;
    parameter2=(L1**2+D**2-B**2)/(2*L1*D);
    degree_4=initial_degree2-(m.acos(parameter2)-m.acos(parameter1));
    return degree_4

def trans2_2_3_4(theta):#模组2 关节2转3转4
    A=7;
    B=46;
    C=45;
    D=6;
    initial_degree1=m.radians(66.3557);
    initial_degree2=m.radians(111.6954);
    L1=m.sqrt(A**2+C**2-2*A*C*m.cos(theta+initial_degree1));
    parameter1=(C-A*m.cos(theta+initial_degree1))/L1;
    parameter2=(L1**2+D**2-B**2)/(2*L1*D);
    degree_3=initial_degree2-(m.acos(parameter2)-m.acos(parameter1));
    
    A1=4.5;
    B1=26.5;
    C1=25;
    D1=6;
    initial_degree3=m.radians(45.0619);
    initial_degree4=m.radians(124.5317);
    L1_=m.sqrt(A1**2+C1**2-2*A1*C1*m.cos(degree_3+initial_degree3));
    parameter1=(C1-A1*m.cos(degree_3+initial_degree3))/L1_;
    parameter2=(L1_**2+D1**2-B1**2)/(2*L1_*D1);
    degree_4=initial_degree4-(m.acos(parameter2)-m.acos(parameter1));
    return degree_3,degree_4

def trans3_2_3_4(theta):#模组3 关节2转3转4
    A=7;
    B=36.85;
    C=36;
    D=6;
    initial_degree1=m.radians(67.63534);
    initial_degree2=m.radians(103.46481);
    L1=m.sqrt(A**2+C**2-2*A*C*m.cos(theta+initial_degree1));
    parameter1=(C-A*m.cos(theta+initial_degree1))/L1;
    parameter2=(L1**2+D**2-B**2)/(2*L1*D);
    degree_3=initial_degree2-(m.acos(parameter2)-m.acos(parameter1));
    
    A1=4.5;
    B1=26.5;
    C1=25;
    D1=6;
    initial_degree3=m.radians(45.0619);
    initial_degree4=m.radians(124.5317);
    L1_=m.sqrt(A1**2+C1**2-2*A1*C1*m.cos(degree_3+initial_degree3));
    parameter1=(C1-A1*m.cos(degree_3+initial_degree3))/L1_;
    parameter2=(L1_**2+D1**2-B1**2)/(2*L1_*D1);
    degree_4=initial_degree4-(m.acos(parameter2)-m.acos(parameter1));
    return degree_3,degree_4

def trans4_3_4(theta):#模组4 关节3转4
    A=7;
    B=37.610;
    C=36;
    D=5.5;
    initial_degree1=m.radians(71.98912);
    initial_degree2=m.radians(109.82193);
    L1=m.sqrt(A**2+C**2-2*A*C*m.cos(theta+initial_degree1));
    parameter1=(C-A*m.cos(theta+initial_degree1))/L1;
    parameter2=(L1**2+D**2-B**2)/(2*L1*D);
    degree_4=initial_degree2-(m.acos(parameter2)-m.acos(parameter1));
    return degree_4    
    


if __name__ == '__main__': 
    try: 
        talker() 
    except rospy.ROSInterruptException:
        pass
