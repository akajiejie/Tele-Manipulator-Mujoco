import time
import os
import math
from scipy.spatial.transform import Rotation as R
import numpy as np
import aloha_robot as robot # 忽略这里的报错
l_p = 150 # 工具参考点到电机输出轴表面的距离，单位mm（所有尺寸参数皆为mm）
l_p_mass_center = 55 # 工具（负载）质心到 6 号关节输出面的距离
G_p = 0.396 # 负载重量，单位kg，所有重量单位皆为kg
uart_baudrate = 115200 # 串口波特率，与CAN模块的串口波特率一致，（出厂默认为 115200，最高460800）

# com = 'COM9' # 在这里输入 COM 端口号
com='/dev/ttyACM0' # 在 jetson nano（ubuntu）下控制机器人，相应的输入连接的串口
# com='/dev/ttyAMA0' # 在树莓派（raspbian）下控制机器人，相应的输入连接的串口
# com='/dev/cu.usbserial-110' # 在苹果电脑 mac 下控制机器人，相应地输入串口
# # 机械臂对象初始化函数函数


dr = robot.robot(L_p=l_p, L_p_mass_center=l_p_mass_center, G_p=G_p, com=com, uart_baudrate=uart_baudrate)


'''设置机器人参数'''
dr.L = [152, 152, 70, 62]  # Aloha 机械臂尺寸参数列表：[l1, l2, l3, d3]，详见库函数说明
dr.G_L = [45, 30, 60, 106, 43, 62]
''' G_L 是计算重力补偿时用到的参数
    G_L[0] 是杆件 3 质心到关节 3 轴线在机械臂伸长方向上的距离 G_L_1
    G_L[1] 是关节 4 质心到关节 3 轴线在机械臂伸长方向上的距离 G_L_2
    G_L[2] 是杆件 4 质心到关节 4 端面的距离 G_L_3
    G_L[3] 是关节 5 轴线到关节 4 端面的距离 G_L_4
    G_L[4] 是杆件 5 质心到关节 5 轴线的距离 G_L_5
    G_L[5] 是关节 6 质到心关节 5 轴线的距离 G_L_6
'''
dr.G = [0.1005, 0.054, 0.057, 0.057, 0.329, 0.183, 0.253, 0.183] + [G_p] # + [G_p] 必不可少
'''G 是零部件重量 单位 kg
G[0] 杆件 2 重量 G_Gan_2
G[1] 杆件 3 重量 G_Gan_3
G[2] 杆件 4 重量 G_Gan_4
G[3] 杆件 5 重量 G_Gan_5
G[4] 关节 3 重量 G_3
G[5] 关节 4 重量 G_4
G[6] 关节 5 重量 G_5
G[7] 关节 6 重量 G_6
'''
#正运动学函数
def draloha_fk(theta1, theta2, theta3, theta4, theta5, theta6):
    a2 = 152
    d4 = 152
    d6 = 70
    a3 = 62
    theta2_offset = math.pi / 2
    theta3_offset = math.pi / 2
    theta6_offset = math.pi
    
    theta2_in = theta2 + theta2_offset
    theta3_in = theta3 + theta3_offset
    theta6_in = theta6 + theta6_offset
    
    # T01 matrix
    T01 = np.array([
        [math.cos(theta1), 0, math.sin(theta1), 0],
        [math.sin(theta1), 0, -math.cos(theta1), 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1]
    ])
    
    # T12 matrix
    T12 = np.array([
        [math.cos(theta2_in), -math.sin(theta2_in), 0, a2 * math.cos(theta2_in)],
        [math.sin(theta2_in), math.cos(theta2_in), 0, a2 * math.sin(theta2_in)],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    # T23 matrix
    T23 = np.array([
        [math.cos(theta3_in), 0, math.sin(theta3_in), a3 * math.cos(theta3_in)],
        [math.sin(theta3_in), 0, -math.cos(theta3_in), a3 * math.sin(theta3_in)],
        [0, 1, 0, 0],
        [0, 0, 0, 1]
    ])
    
    # T34 matrix
    T34 = np.array([
        [math.cos(theta4), 0, math.sin(theta4), 0],
        [math.sin(theta4), 0, -math.cos(theta4), 0],
        [0, 1, 0, d4],
        [0, 0, 0, 1]
    ])
    
    # T45 matrix
    T45 = np.array([
        [math.cos(theta5), 0, -math.sin(theta5), 0],
        [math.sin(theta5), 0, math.cos(theta5), 0],
        [0, -1, 0, 0],
        [0, 0, 0, 1]
    ])
    
    # T56 matrix
    T56 = np.array([
        [math.cos(theta6_in), -math.sin(theta6_in), 0, 0],
        [math.sin(theta6_in), math.cos(theta6_in), 0, 0],
        [0, 0, 1, d6],
        [0, 0, 0, 1]
    ])
    
    # Calculate final transformation matrix
    T06 = T01 @ T12 @ T23 @ T34 @ T45 @ T56
    
    return T06
                   
def main():
    # '''''''不用的运动模式，所匹配的 PID 略有不同，正式运动控制前建议优化一下各关节PID，以下为进行点到点运动控制时的一组较优 PID，用户可进一步优化'''''''
    dr.set_pid(1, P=20, I=5, D=2.55)
    dr.set_pid(2, P=20, I=4.95, D=1.5)
    dr.set_pid(3, P=20, I=4.95, D=1.5)
    dr.set_pid(4, P=20, I=9, D=0.5)
    dr.set_pid(5,  P=20, I=5, D=2.1)
    dr.set_pid(6,  P=20, I=5, D=0.096)
    dr.set_pid(7,  P=20, I=5, D=0.096)
    # dr.torque_factors = [1, 1, 1, 1, 1, 1] # 于调节模型扭矩与电机扭矩的比例关系，当重力补偿或零力拖动效果不佳时可用该参数调节
    targetpose=[0.0, 80.0, -90.0, -0.0, -50, 0.0]
    pose = dr.detect_joints()
    if (pose ==targetpose) :
        pass
    else :
        dr.set_pose(x_y_z=[268, 0, 40], theta_4_5_6=[0, -60, 0], speed=1, param=1, mode=1) # 初始操作姿态
        dr.positions_done([1,2,3,4 ,5,6,7])
        time.sleep(1)
    dr.torque_factors = [1, 1, 1, 1, 1, 1] # 于调节模型扭矩与电机扭矩的比例关系，当重力补偿或零力拖动效果不佳时可用该参数调节

    Gp_angle=dr.get_angle(7)#获取夹爪角度
    id_list = dr.ID_list + [7] # 假设总线中只有六个机械臂关节电机，如有其它关节电机则需要加上对应的ID号
    length = len(id_list)
    dr.set_torques(id_list=[dr.ID_list[0],dr.ID_list[2],dr.ID_list[5],7], torque_list=[0, 0, 0, 0], param=0, mode=0) # 放松 1、6 号关节
    angle_list = []
    angle_speed_torque = dr.get_angle_speed_torque_all(id_list)
    for i in range(length-1):
        angle_list.append(angle_speed_torque[i][0])
    dr.gravity_compensation(angle_list=angle_list)#重力补偿开启一次就行，循环中开启会导致异常现象
    

    #启动零力拖动
    t = 10 # 零力拖动执行的时间
    dt=0.02 #更新频率50Hz
    start = time.time()
    start_time=start

    while(time.time() - start < t):
        if ((time.time()-start_time)>=dt):
            end_post=dr.detect_pose()#获取末端位姿，格式为[x,y,z],[qz1,qy,qz2]
            theta=dr.detect_joints()
            theta[1]=theta[1]-90
            Matrix=draloha_fk(theta[0],theta[1],theta[2],theta[3],theta[4],theta[5])
            r=Matrix[:3,:3]
            rotation = R.from_matrix(r)
            end_quaternion = rotation.as_quat()
            Gp_angle=dr.get_angle(7)#获取夹爪电机位置
            print(end_post[:3])
            print(end_quaternion)
            print("Gp_angle:",Gp_angle)
                    
        
    for j in range(len(dr.ID_list)):#停止1-6号关节电机工作
        dr.estop(dr.ID_list[j])

if __name__ == '__main__':
    main()
