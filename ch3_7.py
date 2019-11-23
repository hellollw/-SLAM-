# -*- coding:utf-8 -*-
# Author: Lu Liwen
# Modified Time: 2019-11-23

"""
P72 第七题
"""
from lib import Basic_Transform as Basic
from numpy import *

if __name__ == '__main__':
    q1 = mat([0.35, 0.2, 0.3, 0.1])
    t1 = mat([0.3, 0.1, 0.1])
    q2 = mat([-0.5, 0.4, -0.1, 0.2])
    t2 = mat([-0.1, 0.5, 0.3])
    # 构建旋转矩阵
    R1 = Basic.quaternion_to_RotationMatrix(Basic.normalize_q(q1))
    R2 = Basic.quaternion_to_RotationMatrix(Basic.normalize_q(q2))
    # 构建位姿矩阵(从世界坐标系到相机坐标系的转换）
    Tcw1 = Basic.constructT(R1, t1)
    Tcw2 = Basic.constructT(R2, t2)
    # 求解在小罗卜二号坐标系下的坐标
    p = mat([0.5,0,0.2]).T
    pc1 = Basic.homoLocation(p)
    pw = Tcw1.I*pc1
    pc2 = Tcw2*pw
    print(pc2)
