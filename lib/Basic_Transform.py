# -*- coding:utf-8 -*-
# Author: Lu Liwen
# Modified Time: 2019-11-23

"""
格式：
行向量表示（平移向量，四元数）
列向量表示（坐标）

常用的旋转计算

包含方法：
1. 四元数——>旋转矩阵 quaternion_to_RotationMatrix(quaternion)
2. 构造位姿矩阵T：def constructT(R, t)
3. 四元数归一化：def normalize_q(q)
4. 构建齐次坐标：def homoLocation(p)

"""

import numpy as num


# 构建其次坐标([p
#              1])
# 输入：三维坐标p
# 输出: 齐次坐标p_home
def homoLocation(p):
    p_homo = num.mat(num.ones((4, 1)))
    p_homo[0:3,0] = p.copy()
    return p_homo


# 四元数——>旋转矩阵
# 输入：四元数
# 输出：对应的旋转矩阵
def quaternion_to_RotationMatrix(quaternion):
    q = quaternion.copy()
    if num.shape(q) == (1, 4):  # shape返回元组
        q = q.A[0]  # 转换为一维数组
        RotationMatrix = num.mat(
            [[1 - 2 * q[2] ** 2 - 2 * q[3] ** 2, 2 * q[1] * q[2] + 2 * q[0] * q[3], 2 * q[1] * q[3] - 2 * q[0] * q[2]],
             [2 * q[1] * q[2] - 2 * q[
                 0] * q[3], 1 - 2 * q[1] ** 2 - 2 * q[3] ** 2, 2 * q[2] * q[3] + 2 * q[0] * q[1]],
             [2 * q[1] * q[3] + 2 * q[0] * q[2], 2 * q[2] * q[3] - 2 * q[0] * q[1],
              1 - 2 * q[1] ** 2 - 2 * q[2] ** 2]])
        return RotationMatrix
    else:
        raise SystemError('四元数格式不正确')


# 构造位姿矩阵
# 输入：旋转矩阵:R, 平移向量:t(1行三列）
# 输出：位姿矩阵:T
def constructT(R, t):
    if num.shape(R) == (3, 3) and num.shape(t) == (1, 3):
        T = num.mat(num.zeros((4, 4)))
        T[0:3, 0:3] = R.copy()  # stop不包括在内
        T[0:3, 3] = t.T.copy()
        T[3, 0:3] = 0
        T[3, 3] = 1
        return T
    else:
        raise SystemError('输入格式不正确')


# 四元数归一化（只有单位四元数（四元数模长为1）才表示旋转）——>每一项都除以总和的均值
# 输入：未归一化的四元数:q
# 输出：归一化的四元数:q_normal
def normalize_q(q):
    q_normal = q.copy()
    q_sum = sum(q)
    q_normal = q_normal / q_sum
    return q_normal


# if __name__ == '__main__':
#     q = num.mat([0, 1, 2, 3])
#     t = num.mat([1, 2, 3])
#     R = quaternion_to_RotationMatrix(q)
#     T = constructT(R, t)
#     print(T)
