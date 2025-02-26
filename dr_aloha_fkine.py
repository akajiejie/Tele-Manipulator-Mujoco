import numpy as np
import math

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