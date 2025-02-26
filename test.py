import time
import math
import aloha_robot as robot # 忽略这里的报错
l_p = 150 # 工具参考点到电机输出轴表面的距离，单位mm（所有尺寸参数皆为mm）
l_p_mass_center = 55 # 工具（负载）质心到 6 号关节输出面的距离
G_p = 0.396 # 负载重量，单位kg，所有重量单位皆为kg
uart_baudrate = 115200 # 串口波特率，与CAN模块的串口波特率一致，（出厂默认为 115200，最高460800）
#com = 'COM9' # 在这里输入 COM 端口号
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

dr.torque_factors = [1, 1, 1, 1, 1, 1] # 于调节模型扭矩与电机扭矩的比例关系，当重力补偿或零力拖动效果不佳时可用该参数调节


'''''''不用的运动模式，所匹配的 PID 略有不同，正式运动控制前建议优化一下各关节PID，以下为进行点到点运动控制时的一组较优 PID，用户可进一步优化'''''''
# dr.set_pid(1, P=20, I=5, D=2.55)
# dr.set_pid(2, P=20, I=4.95, D=1.5)
# dr.set_pid(3, P=20, I=4.95, D=1.5)
# dr.set_pid(4, P=20, I=9, D=0.5)
# dr.set_pid(5,  P=20, I=5, D=2.1)
# dr.set_pid(6,  P=20, I=5, D=0.096)
# speed = 10
# dr.set_pose(x_y_z=[240, 0, 120], theta_4_5_6=[0, 0, 0], speed=speed, param=speed, mode=1)
# while 1:
#     dr.set_joints([0, 180, -180, 0, 0, 0], speed)
#     dr.pose_done()
#     dr.set_pose(x_y_z=[240, 0, 120], theta_4_5_6=[10, 45, 30], speed=speed, param=speed, mode=1)
#     dr.pose_done()
#     dr.set_pose(x_y_z=[160, 20, 50], theta_4_5_6=[-10, -45, -30], speed=speed, param=speed, mode=1)
#     dr.pose_done()
# # *********************************************************

'''''''运动到指定位置和姿态函数'''''''
# dr.set_pose(x_y_z=[dr.L[0] + dr.L[1] + dr.L[2], 0, dr.L[3]], theta_4_5_6=[0, 0, 0], speed=1, param=1, mode=1)
# dr.set_pose(x_y_z=[200, 200, 200], theta_4_5_6=[0, 0, 0], speed=1, param=1, mode=1)
# # *********************************************************

'''''''手爪开合函数（当机械臂末端安装大然手爪后可用，手爪关节ID号需为7）'''''''
# dr.grasp(wideth=50, speed=10, force=120)
# dr.grasp_done() # # 检测手爪开合是否到位
# # *********************************************************

'''''''运动到指定位置函数'''''''
# dr.set_position(x_y_z=[dr.L[0] + dr.L[1] + dr.L[2], 0, 0], speed=10, param=10, mode=1)
# # *********************************************************

'''''''运动到指定姿态函数'''''''
# dr.set_P_Y_R(theta_4_5_6=[0, -10, 0], speed=10, param=10, mode=1)
# # *********************************************************

'''''''运动到相对位置和姿态函数'''''''
# dr.set_relative_pose(x_y_z=[10, 10, 10],  theta_4_5_6=[0, 0, 0], speed=10, param=10, mode=1)
# # *********************************************************

'''''''运动到相对位置函数'''''''
# dr.set_relative_position(x_y_z=[10, 0, 0], speed=10, param=10, mode=1)
# # *********************************************************

'''''''运动到相对姿态函数'''''''
# dr.set_relative_P_Y_R(theta_4_5_6=[0, 10, 0], speed=10, param=10, mode=1)
# *********************************************************

# '''''''''''''''''''''轨迹跟踪函数'''''''''''''''''''''
# # '''''''画正方形'''''''
# def draw_rectangle(pl=[283, 0, -126.5], l=30, h=30):
#     ''''在水平面上画正方形
#     pl: 长方形左上角坐标（起始点），其中pl[2]代表作图平面与全局坐标系z轴的焦点的z坐标
#     l: 宽度
#     h: 高度
#     '''
#     n= 100 # 每条边分割的点数（数量越多画得越慢）
#     l_delta = l/n
#     h_delta = h/n
#     pl_list = []
#     pl_list.append(pl)
#     l1 = pl[1]
#     for i in range(1, n+1):
#         pl_temp = [pl[0], pl[1]-i*l_delta, pl[2]]
#         pl_list.append(pl_temp)
#     print(pl_temp)
#     for i in range(1, n+1):
#         pl_temp1 = [pl_temp[0]-i*h_delta, pl_temp[1], pl_temp[2]]
#         pl_list.append(pl_temp1)
#     print(pl_temp1)
#     for i in range(1, n+1):
#         pl_temp2 = [pl_temp1[0], pl_temp1[1]+i*l_delta, pl_temp1[2]]
#         pl_list.append(pl_temp2)
#     print(pl_temp2)
#     for i in range(1, n+1):
#         pl_temp3 = [pl_temp2[0]+i*h_delta, pl_temp2[1], pl_temp2[2]]
#         pl_list.append(pl_temp3)
#     print(pl_temp3)
#     print(pl_list)
#     return pl_list
# #
# #
# pl_list = draw_rectangle(pl=[220, 100, 100], l=200, h=50) #

# # ########轨迹运动之前可先调整一下 pid，以获得更好的曲线平滑度
# dr.set_pid_joint(1, P=10, I=5, D=2.55)
# # dr.set_pid_joint(2, P=10.56, I=4.95, D=0.39)
# dr.set_pid_joint(3, P=10.56, I=4.95, D=1.39)
# # dr.set_pid_joint(4, P=10, I=9, D=0.5)
# # dr.set_pid_joint(5,  P=12, I=5, D=0.1)
# # dr.set_pid_joint(6,  P=12, I=5, D=0.096)

# # ########控制机械臂末端连续运动到多个指定位置和姿态函数(必须单独一次性使用)
# # dr.set_poses(pls_temp=pl_list, theta_4_5_6s_temp=[[0, -45, 0]], t=5) # 控制机械臂末端连续运动到多个指定位置和姿态函数(必须单独一次性使用)
# dr.set_poses_curve_pre(pls_temp=pl_list, theta_4_5_6s_temp=[[0, -45, 0]]) # 预设机械臂末端轨迹函数
# dr.set_poses_curve_start_point(10) # 运动到轨迹起始位置函数
# while True:
#     dr.set_poses_curve_do(10) # 末端轨迹执行函数，参数为大致运行时间
# # ************************************************************

'''''''画椭圆'''''''
# def draw_ellipse(pl=[200, 0, 0], a=10, b=20):
#     ''''椭圆方程: (x-pl[0])²/a²+(y-pl[1])²/b²=1
#         pl: 椭圆中心点坐标（起始点）,其中pl[2]代表作图水平面在z轴上的位置
#         a: x轴对应轴长
#         b: x轴对应轴长y
#         '''
#     n = 800 # 每条边分割的点数（数量越多画得越慢）, n过小会在末尾与起始之间有明显停顿
#     angle_delta = math.pi/n * 2
#     pl_list = []
#     for i in range(0, n):
#         x = pl[0] + a * math.cos(angle_delta*i)
#         y = pl[1] + b * math.sin(angle_delta*i)
#         pl_list.append([x, y, pl[2]])
#     print(pl_list)
#     return pl_list


# pl_list = draw_ellipse(pl=[300, 0, 10], a=50, b=90) # 求点

# # ########轨迹运动之前可先调整一下 pid，以获得更好的曲线平滑度
# # dr.set_pid_joint(1, P=10, I=5, D=0.55)
# # dr.set_pid_joint(2, P=10.56, I=4.95, D=0.39)
# # dr.set_pid_joint(3, P=10.56, I=4.95, D=0.39)
# # dr.set_pid_joint(4, P=10, I=9, D=0.5)
# # dr.set_pid_joint(5,  P=12, I=5, D=0.1)
# # dr.set_pid_joint(6,  P=12, I=5, D=0.096)

# dr.set_poses(pls_temp=pl_list, theta_4_5_6s_temp=[[0, -45, 0]], t=10)
# dr.set_poses_curve_pre(pls_temp=pl_list, theta_4_5_6s_temp=[[0, -45, 0]]) # 预设机械臂末端轨迹函数
# dr.set_poses_curve_start_point(10) # 运动到轨迹起始位置函数
# dr.set_poses_curve_do(5) # 末端轨迹执行函数，参数为大致运行时间
# while True:
#     dr.set_poses_curve_do(5) # 末端轨迹执行函数，参数为大致运行时间
# # ************************************************************

'''''''控制机械臂关节角度函数'''''''
# dr.set_joints(angle_list=[0, 90, 0, 0, 0, 0], speed=1.0) # 对应竖直状态
# # *********************************************************

'''''''查看当前位姿函数'''''''
# show_pose = dr.show_pose()
# print(show_pose)
# # *********************************************************

'''''''查看当前关节模型角度函数(通过回读一体化关节角度计算)'''''''
pose = dr.detect_joints()
print(pose)
# # *********************************************************

'''''''查看当前位姿函数(通过回读一体化关节角度计算)'''''''
# detect_pose = dr.detect_pose()
# print(detect_pose)
# # *********************************************************

'''''''零力拖动'''''''
# t = 20 # 零力拖动执行的时间
# start = time.time()
# id_list = dr.ID_list + [7] # 假设总线中只有六个机械臂关节电机，如有其它关节电机则需要加上对应的ID号
# length = len(id_list)
# dr.set_torques(id_list=[dr.ID_list[0], dr.ID_list[5],7], torque_list=[0, 0], param=0, mode=0) # 放松 1、6 号关节
# while(time.time() - start < t):
#     angle_list = []
#     angle_speed_torque = dr.get_angle_speed_torque_all(id_list)
#     if angle_speed_torque is None:
#         pass
#     else:
#         for i in range(length-1):
#             angle_list.append(angle_speed_torque[i][0])
#         print(angle_list)
#         dr.gravity_compensation(angle_list=angle_list)
# for j in range(len(dr.ID_list)):
#     dr.estop(dr.ID_list[j])

'''''''回读关节 PID 参数函数'''''''
# dr.get_pid_joint(joint_num=1)
#
'''''''设置关节 PID 参数函数'''''''
# dr.set_pid_joint(joint_num=1, P=10, I=5, D=0.55)
#
'''''''回读关节参数函数'''''''
# dr.get_property(joint_num=1, property='dr.config.gear_ratio') # 关节 减速比
#
'''''''设置关节参数函数'''''''
# dr.set_property(joint_num=1, property='dr.controller.config.speed_limit', value=300) # 关节最大限制转速，此处转速为电机转速，输出端需要乘以减速比
