'''
Author: yuzhe-yang chn.yuzhe.yang@gmail.com
LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
LastEditTime: 2026-05-20 20
FilePath: /standard/hw2-5-1.py
Description: 

Copyright (c) 2026 by yuzhe-yang, All Rights Reserved. 
'''
import numpy as np

def dh_matrix(theta, d, a, alpha):
    """
    根据标准DH参数计算变换矩阵 T_{i-1, i}
    单位: 角度使用弧度, 长度使用米
    """
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d   ],
        [0,   0,      0,     1   ]
    ])

def forward_kinematics_UR5(q):
    """
    计算UR5机械臂的末端位置
    """
    theta1, theta2, theta3, theta4, theta5, theta6 = q
    d = [0.08916, 0, 0, 0.10915, 0.09465, 0.08230]
    a = [0, -0.425, -0.39225, 0, 0, 0]
    alpha = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]
    
    T_01 = dh_matrix(theta1, d[0], a[0], alpha[0])
    T_12 = dh_matrix(theta2, d[1], a[1], alpha[1])
    T_23 = dh_matrix(theta3, d[2], a[2], alpha[2])
    T_34 = dh_matrix(theta4, d[3], a[3], alpha[3])
    T_45 = dh_matrix(theta5, d[4], a[4], alpha[4])
    T_56 = dh_matrix(theta6, d[5], a[5], alpha[5])
    
    # 总变换矩阵
    T_06 = T_01 @ T_12 @ T_23 @ T_34 @ T_45 @ T_56
    
    return T_06

# --- 验证 ---

q = [0, 0, 0, 0, 0, 0]

T_final = forward_kinematics_UR5(q)
pos_final = T_final[:3, 3]

print(f"关节角: {np.degrees(q)}")
print(f"计算出的末端位置 (x, y, z): {np.round(pos_final, 4)}")