from cmath import pi
#from syslog import closelog
from turtle import distance
import numpy as np
import math
import copy
import pandas as pd
import csv
import time


class calculation(object):
    def __init__(self, RotationMatrix):
        self.RotationMatrix = RotationMatrix
        #码垛托盘起始点

    def camera_calibration(self, barcodePos, Identification_point, vis_poin, ident_euler):
        """相机标定"""
        real_point = barcodePos
        vectorJ = []

        for i in range(0, len(Identification_point) - 3):
            for j in range(i + 1, len(Identification_point) - 2):
                for k in range(j + 1, len(Identification_point) - 1):
                    for h in range(k + 1, len(Identification_point)):

                        vectorB1 = vis_poin[i] - vis_poin[j]
                        vectorB2 = vis_poin[i] - vis_poin[k]
                        vectorB3 = vis_poin[i] - vis_poin[h]

                        vectorC = np.array([vectorB1, vectorB2, vectorB3]).T

                        det = np.linalg.det(vectorC)
                        if abs(det) > 0.00001:
                            vectorD1 = Identification_point[j] - Identification_point[i]
                            vectorD2 = Identification_point[k] - Identification_point[i]
                            vectorD3 = Identification_point[h] - Identification_point[i]

                            vectorE = np.array([vectorD1, vectorD2, vectorD3]).T

                            vectorC_1 = (np.mat(vectorC)).I
                            vectorA = np.array(vectorE @ vectorC_1)
                            suma = 0
                            for ai in range(0, 3):
                                for bi in range(0, 3):
                                    suma += vectorA[ai, bi] ** 2

                            vectorF = real_point - Identification_point[i] - (vectorA @ (vis_poin[i].T)).T
                            vectorJ.append([np.array(vectorA), np.array(vectorF),
                                            str(i) + "," + str(j) + "," + str(k) + "," + str(h)])
                            G = vectorA @ vectorC
                        else:
                            print("奇异矩阵")

        minSum = 1000000
        minVectorJ = vectorJ[0]
        for i in range(len(vectorJ)):
            averangSum = 0
            for j in range(0, len(vis_poin)):
                vectorA = vectorJ[i][0]
                vectorF = vectorJ[i][1]
                vectorH = real_point - (Identification_point[j] + vectorF + (vectorA @ (vis_poin[j].T)).T)
                averangSum += abs(vectorH[0]) + abs(vectorH[1]) + abs(vectorH[2])
            if averangSum < minSum:
                minSum = averangSum
                minVectorJ = vectorJ[i]

        offset_value = []

        for j in range(0, len(vis_poin)):
            saveData = list()
            saveTime = time.strftime('_%Y-%m-%d-%H-%M-%S')
            saveData.append(saveTime)

            vectorA = minVectorJ[0]
            vectorF = minVectorJ[1]

            #euler = {'u':df._values[1][6],'v':df._values[1][7],'w':df._values[1][8]}

            euler = ident_euler

            rota = self.rotation_matrix(euler)
            vectorA = ((np.mat(rota)).I) @ vectorA
            vectorA = np.array(vectorA)
            vectorH = Identification_point[j] + vectorF + self.revolve(euler, (vectorA @ (vis_poin[j].T)).T)

            vectorF = ((np.mat(rota)).I) @ vectorF
            vectorF = np.array(vectorF)[0]
            offset_value.append(real_point - vectorH)

        return vectorF, vectorA, offset_value

    def readExcel(self):
        df = pd.read_excel('abc.xls', sheet_name='Sheet1')

        # 打印数据框内容
        print(df)
        df._values
        real_point = np.array([df._values[1][0], df._values[1][1], df._values[1][2]])
        Identification_point = []
        vis_poin = []
        vectorJ = []

        for i in range(0, 13):
            Identification_point.append(np.array([df._values[i][3], df._values[i][4], df._values[i][5]]))

        for i in range(0, 13):
            vis_poin.append(np.array([df._values[i][9], df._values[i][10], df._values[i][11]]))

        for i in range(0, len(Identification_point) - 3):
            for j in range(i + 1, len(Identification_point) - 2):
                for k in range(j + 1, len(Identification_point) - 1):
                    for h in range(k + 1, len(Identification_point)):

                        vectorB1 = vis_poin[i] - vis_poin[j]
                        vectorB2 = vis_poin[i] - vis_poin[k]
                        vectorB3 = vis_poin[i] - vis_poin[h]

                        vectorC = np.array([vectorB1, vectorB2, vectorB3]).T
                        #det3 = (vectorC[0][0]*(vectorC[1][1]*vectorC[2][2]-vectorC[1][2]*vectorC[2][1])
                        #       -vectorC[0][1]*(vectorC[1][0]*vectorC[2][2]-vectorC[1][2]*vectorC[2][0])
                        #       +vectorC[0][2]*(vectorC[1][0]*vectorC[2][1]-vectorC[1][1]*vectorC[2][0]))
                        det = np.linalg.det(vectorC)
                        if abs(det) > 0.00001:
                            vectorD1 = Identification_point[j] - Identification_point[i]
                            vectorD2 = Identification_point[k] - Identification_point[i]
                            vectorD3 = Identification_point[h] - Identification_point[i]

                            vectorE = np.array([vectorD1, vectorD2, vectorD3]).T

                            vectorC_1 = (np.mat(vectorC)).I
                            vectorA = np.array(vectorE @ vectorC_1)
                            suma = 0
                            for ai in range(0, 3):
                                for bi in range(0, 3):
                                    suma += vectorA[ai, bi] ** 2
                            #vectorA = vectorA/((suma/3)**0.5)
                            if vectorA[2][0] >= -1 and vectorA[2][0] <= 1:
                                u = math.atan(vectorA[2][1] / vectorA[2][2]) / math.pi * 180
                                v = math.asin(-vectorA[2][0]) / math.pi * 180
                                w = math.atan(vectorA[1][0] / vectorA[0][0]) / math.pi * 180
                                vectorF = real_point - Identification_point[i] - (vectorA @ (vis_poin[i].T)).T
                                #euler = {'u':u,'v':v,'w':w}
                                #vectorF =  real_point - Identification_point[i] - config.manageConn.revolve(euler,vis_poin[i])
                                vectorJ.append([np.array(vectorA), np.array(vectorF),
                                                str(i) + "," + str(j) + "," + str(k) + "," + str(h)])
                                G = vectorA @ vectorC
                        else:
                            print("奇异矩阵")

        minSum = 1000000
        minVectorJ = vectorJ[0]
        for i in range(len(vectorJ)):
            averangSum = 0
            for j in range(0, len(vis_poin)):
                vectorA = vectorJ[i][0]
                vectorF = vectorJ[i][1]
                vectorH = real_point - (Identification_point[j] + vectorF + (vectorA @ (vis_poin[j].T)).T)
                averangSum += abs(vectorH[0]) + abs(vectorH[1]) + abs(vectorH[2])
            if averangSum < minSum:
                minSum = averangSum
                minVectorJ = vectorJ[i]

        #vectorA = minVectorJ[0]
        #a = (vectorA[0][0]**2+vectorA[0][1]**2+vectorA[0][2]**2)**0.5
        #b = (vectorA[1][0]**2+vectorA[1][1]**2+vectorA[1][2]**2)**0.5
        #c = (vectorA[2][0]**2+vectorA[2][1]**2+vectorA[2][2]**2)**0.5

        #vectorA[0] = vectorA[0]/a
        #vectorA[1] = vectorA[1]/b
        #vectorA[2] = vectorA[2]/c

        #u = math.atan2(vectorA[2][1],vectorA[2][2])/math.pi*180
        #v = math.asin(-vectorA[2][0])/math.pi*180
        #w = math.atan2(vectorA[1][0],vectorA[0][0])/math.pi*180

        #averangSum2 = 0
        #minSum2 = 1000000
        #minVectorJ2 = vectorJ[0]
        #for i in range(0,20):
        #    for j in range(0,20):
        #        for k in range(0,20):
        #            u2 = u + i*0.1-1
        #            v2 = v + j*0.1-1
        #            w2 = w + k*0.1-1
        #            rota = self.rotation_matrix({'u':u2,'v':v2,'w':w2})
        #            rota[0] = rota[0]*a
        #            rota[1] = rota[1]*b
        #            rota[2] = rota[2]*c

        #            averangSum2 = 0
        #            vectorSum = np.array([0.0,0.0,0.0])
        #            for j in range(0,len(vis_poin)):
        #                vectorA = rota
        #                vectorF = real_point - (Identification_point[j]+ (vectorA@(vis_poin[j].T)).T)
        #                vectorSum += vectorF

        #            vectorF = vectorSum/len(vis_poin)

        #            for j in range(0,len(vis_poin)):
        #                vectorH = real_point-(Identification_point[j]+vectorF+ (vectorA@(vis_poin[j].T)).T)
        #                averangSum2 +=abs(vectorH[0])+abs(vectorH[1])+abs(vectorH[2])

        #            if averangSum2 < minSum2:
        #                minSum2 = averangSum2
        #                minVectorJ2 = vectorA

        saveData = list()
        saveData.append("时间")
        saveData.append("序号")
        saveData.append("视觉坐标x")
        saveData.append("视觉坐标y")
        saveData.append("视觉坐标z")
        saveData.append("实际x")
        saveData.append("实际y")
        saveData.append("实际z")
        saveData.append("拟合x")
        saveData.append("拟合y")
        saveData.append("拟合z")
        saveData.append("识别点x")
        saveData.append("识别点y")
        saveData.append("识别点z")
        #saveData.append(vectorG[0])
        #saveData.append(vectorG[1])
        #saveData.append(vectorG[2])
        saveData.append("相机偏移x")
        saveData.append("相机偏移y")
        saveData.append("相机偏移z")
        saveData.append("旋转矩阵")
        saveData.append("旋转矩阵")
        saveData.append("旋转矩阵")
        saveData.append("旋转矩阵")
        saveData.append("旋转矩阵")
        saveData.append("旋转矩阵")
        saveData.append("旋转矩阵")
        saveData.append("旋转矩阵")
        saveData.append("旋转矩阵")
        #saveData.append("u")
        #saveData.append("v")
        #saveData.append("w")

        saveData.append("偏差x")
        saveData.append("偏差y")
        saveData.append("偏差z")
        saveData.append("组合")
        WriteDataFile = 'Data.csv'
        with open(WriteDataFile, "a+", newline='') as f:
            # with open(birth_weight_file, "w") as f:
            writer = csv.writer(f)
            writer.writerow(saveData)
            f.close()
        offset_value = []
        for j in range(0, len(vis_poin)):
            saveData = list()
            saveTime = time.strftime('_%Y-%m-%d-%H-%M-%S')
            saveData.append(saveTime)

            vectorA = minVectorJ[0]
            vectorF = minVectorJ[1]

            euler = {'u': df._values[1][6], 'v': df._values[1][7], 'w': df._values[1][8]}
            rota = self.rotation_matrix(euler)
            vectorA = ((np.mat(rota)).I) @ vectorA
            vectorA = np.array(vectorA)
            vectorH = Identification_point[j] + vectorF + self.revolve(euler, (vectorA @ (vis_poin[j].T)).T)

            vectorF = ((np.mat(rota)).I) @ vectorF
            vectorF = np.array(vectorF)[0]

            offset_value.append(real_point - vectorH)

            #vectorG = vector0+vectorF+ (vectorA@(vis_poin2[j].T)).T
            saveData.append(j)
            saveData.append(vis_poin[j][0])
            saveData.append(vis_poin[j][1])
            saveData.append(vis_poin[j][2])
            saveData.append(real_point[0])
            saveData.append(real_point[1])
            saveData.append(real_point[2])
            saveData.append(vectorH[0])
            saveData.append(vectorH[1])
            saveData.append(vectorH[2])
            saveData.append(Identification_point[j][0])
            saveData.append(Identification_point[j][1])
            saveData.append(Identification_point[j][2])
            #saveData.append(vectorG[0])
            #saveData.append(vectorG[1])
            #saveData.append(vectorG[2])
            saveData.append(vectorF[0])
            saveData.append(vectorF[1])
            saveData.append(vectorF[2])
            saveData.append(vectorA[0][0])
            saveData.append(vectorA[0][1])
            saveData.append(vectorA[0][2])
            saveData.append(vectorA[1][0])
            saveData.append(vectorA[1][1])
            saveData.append(vectorA[1][2])
            saveData.append(vectorA[2][0])
            saveData.append(vectorA[2][1])
            saveData.append(vectorA[2][2])

            #u = math.atan2(vectorA[2][1],vectorA[2][2])/math.pi*180
            #v = math.asin(-vectorA[2][0])/math.pi*180
            #w = math.atan(vectorA[1][0],vectorA[0][0])/math.pi*180
            #saveData.append(u)
            #saveData.append(v)
            #saveData.append(w)
            saveData.append(round(real_point[0] - vectorH[0], 2))
            saveData.append(round(real_point[1] - vectorH[1], 2))
            saveData.append(round(real_point[2] - vectorH[2], 2))

            saveData.append(minVectorJ[2])
            WriteDataFile = 'Data.csv'
            with open(WriteDataFile, "a+", newline='') as f:
                # with open(birth_weight_file, "w") as f:
                writer = csv.writer(f)
                writer.writerow(saveData)
                f.close()
        return vectorF, vectorA, offset_value

    def getBarcodePos(self, point, clamp_T):
        euler = {'u': point['u'], 'v': point['v'], 'w': point['w']}
        vector = np.array([point['x'], point['y'], point['z']])
        barcodePos = vector + self.revolve(euler, clamp_T)
        return barcodePos

    def jzbd(self):

        ###上侧标定 (-226 -7 -269)
        euler_tar1 = {'u': 0, 'v': 90, 'w': 0}
        euler_tar2 = {'u': 134.074, 'v': 24.132, 'w': 58.318}
        euler_tar3 = {'u': -73.163, 'v': 42.169, 'w': -33.061}
        euler_tar4 = {'u': 174.281, 'v': 48.496, 'w': -139.937}

        V1 = np.array([1126.197, 429.119, 744.678])
        V2 = np.array([1141.526, 399.026, 757.883])
        V3 = np.array([1037.475, 335.675, 580.679])
        V4 = np.array([1460.316, 389.189, 879.122])

        T_tar1 = self.rotation_matrix(euler_tar1)
        T_tar2 = self.rotation_matrix(euler_tar2)
        T_tar3 = self.rotation_matrix(euler_tar3)
        T_tar4 = self.rotation_matrix(euler_tar4)

        VR = np.array([1159.349, 439.903, 692]) + self.revolve({'u': -179.99, 'v': -90, 'w': 0},
                                                               np.array([-256.546, 0.352, 289]))
        jzpy = (np.mat(T_tar1).I) @ (VR - np.array([1127.220, 426.608, 744.263]))
        print(jzpy)

        vectorC_1 = (np.mat(T_tar1 - T_tar2 + T_tar3 - T_tar4)).I

        C = vectorC_1 @ ((V4 - V3 + V2 - V1).T)  #夹爪偏移
        C = np.array(C)[0]
        realpoint = []
        realpoint.append(V1 + np.array(T_tar1 @ (C.T)).T)
        realpoint.append(V2 + np.array(T_tar2 @ (C.T)).T)
        realpoint.append(V3 + np.array(T_tar3 @ (C.T)).T)
        realpoint.append(V4 + np.array(T_tar4 @ (C.T)).T)
        point0 = np.array([0.0, 0.0, 0.0])
        for i in realpoint:
            point0 += i

        point0 = 0.25 * point0
        for i in realpoint:
            print(point0 - i)

        det = np.linalg.det(vectorC_1)
        det2 = np.linalg.det(T_tar1 - T_tar2)

        sumDer = 20000
        sumDerMin = 20000
        targetList = []
        targeT = C
        print(C)
        starTime = time.strftime('_%Y-%m-%d-%H-%M-%S')
        for i in range(0, 40):
            for j in range(0, 40):
                for k in range(0, 40):
                    d = np.array([C[0] - 20 + i, C[1] - 20 + j, C[2] - 20 + k])
                    realpoint1 = V1 + np.array(T_tar1 @ (d.T)).T
                    realpoint2 = V2 + np.array(T_tar2 @ (d.T)).T
                    realpoint3 = V3 + np.array(T_tar3 @ (d.T)).T
                    realpoint4 = V4 + np.array(T_tar4 @ (d.T)).T
                    pointList = []

                    pointList.append(realpoint1)
                    pointList.append(realpoint2)
                    pointList.append(realpoint3)
                    pointList.append(realpoint4)

                    avg = (realpoint1 + realpoint2 + realpoint3 + realpoint4) / 4
                    sumDer = 0
                    for itm in pointList:
                        der = itm - avg
                        sumDer += (abs(der[0]) + abs(der[1]) + abs(der[2]))
                    if sumDer < sumDerMin:
                        sumDerMin = sumDer
                        targetList = pointList
                        targeT = d

        point0 = np.array([0.0, 0.0, 0.0])
        for i in targetList:
            point0 += i

        point0 = 0.25 * point0
        for i in targetList:
            print(point0 - i)

        #endTime = time.strftime('_%Y-%m-%d-%H-%M-%S')
        #print(realpoint)
        #print(vectorC_1)
        print('夹爪偏移')
        print(targeT)
        return 0

    def gripper_calibration(self, pointList):
        """夹爪标定"""

        euler_tar1 = {'u': pointList[0]['u'], 'v': pointList[0]['v'], 'w': pointList[0]['w']}
        euler_tar2 = {'u': pointList[1]['u'], 'v': pointList[1]['v'], 'w': pointList[1]['w']}
        euler_tar3 = {'u': pointList[2]['u'], 'v': pointList[2]['v'], 'w': pointList[2]['w']}
        euler_tar4 = {'u': pointList[3]['u'], 'v': pointList[3]['v'], 'w': pointList[3]['w']}

        V1 = np.array([pointList[0]['x'], pointList[0]['y'], pointList[0]['z']])
        V2 = np.array([pointList[1]['x'], pointList[1]['y'], pointList[1]['z']])
        V3 = np.array([pointList[2]['x'], pointList[2]['y'], pointList[2]['z']])
        V4 = np.array([pointList[3]['x'], pointList[3]['y'], pointList[3]['z']])

        T_tar1 = self.rotation_matrix(euler_tar1)
        T_tar2 = self.rotation_matrix(euler_tar2)
        T_tar3 = self.rotation_matrix(euler_tar3)
        T_tar4 = self.rotation_matrix(euler_tar4)

        vectorC_1 = (np.mat(T_tar1 - T_tar2 + T_tar3 - T_tar4)).I

        C = vectorC_1 @ ((V4 - V3 + V2 - V1).T)  #夹爪偏移
        C = np.array(C)[0]

        det = np.linalg.det(vectorC_1)
        det2 = np.linalg.det(T_tar1 - T_tar2)

        sumDer = 20000
        sumDerMin = 20000
        targetList = []
        targeT = C
        starTime = time.strftime('_%Y-%m-%d-%H-%M-%S')
        for i in range(0, 40):
            for j in range(0, 40):
                for k in range(0, 40):
                    d = np.array([C[0] - 20 + i, C[1] - 20 + j, C[2] - 20 + k])
                    realpoint1 = V1 + np.array(T_tar1 @ (d.T)).T
                    realpoint2 = V2 + np.array(T_tar2 @ (d.T)).T
                    realpoint3 = V3 + np.array(T_tar3 @ (d.T)).T
                    realpoint4 = V4 + np.array(T_tar4 @ (d.T)).T
                    pointList = []

                    pointList.append(realpoint1)
                    pointList.append(realpoint2)
                    pointList.append(realpoint3)
                    pointList.append(realpoint4)

                    avg = (realpoint1 + realpoint2 + realpoint3 + realpoint4) / 4
                    sumDer = 0
                    for itm in pointList:
                        der = itm - avg
                        sumDer += (abs(der[0]) + abs(der[1]) + abs(der[2]))
                    if sumDer < sumDerMin:
                        sumDerMin = sumDer
                        targetList = pointList
                        targeT = d

        point0 = np.array([0.0, 0.0, 0.0])
        for i in targetList:
            point0 += i

        point0 = 0.25 * point0

        offset_value = []
        for i in targetList:
            offset_value.append(point0 - i)

        return targeT, offset_value

    """
    euler 欧拉角
    vector 向量
    函数功能：求出欧拉变换后的向量
    """

    def revolve(self, euler, vector):
        vector2 = vector.T
        u = euler['u'] / 180 * math.pi
        v = euler['v'] / 180 * math.pi
        w = euler['w'] / 180 * math.pi

        a = np.array([[1, 0, 0], [0, math.cos(u), -math.sin(u)], [0, math.sin(u), math.cos(u)]])
        b = np.array([[math.cos(v), 0, math.sin(v)], [0, 1, 0], [-math.sin(v), 0, math.cos(v)]])
        c = np.array([[math.cos(w), -math.sin(w), 0], [math.sin(w), math.cos(w), 0], [0, 0, 1]])

        d = np.dot(c, b)
        d = np.dot(d, a)
        e = d @ vector2

        return e.T

    """
    euler 欧拉角
    函数功能：求出旋转矩阵
    """

    def rotation_matrix(self, euler):
        u = euler['u'] / 180 * math.pi
        v = euler['v'] / 180 * math.pi
        w = euler['w'] / 180 * math.pi

        a = np.array([[1, 0, 0], [0, math.cos(u), -math.sin(u)], [0, math.sin(u), math.cos(u)]])
        b = np.array([[math.cos(v), 0, math.sin(v)], [0, 1, 0], [-math.sin(v), 0, math.cos(v)]])
        c = np.array([[math.cos(w), -math.sin(w), 0], [math.sin(w), math.cos(w), 0], [0, 0, 1]])

        d = np.dot(c, b)
        d = np.dot(d, a)

        return d

    """
    ident_point 识别点
    visual_coordinate 视觉坐标
    ident_euler 识别姿态
    camera_T 相机偏移矩阵
    函数名：获取物体的实际位置
    """

    def GetRealPoint(self, ident_point, visual_coordinate, ident_euler, camera_T):  #获取物体的实际位置
        camera_T_R = self.revolve(ident_euler, camera_T)
        visual_coordinate_R = self.revolve(ident_euler, self.RotationMatrix @ visual_coordinate)
        real_point = ident_point + camera_T_R + visual_coordinate_R

        return copy.deepcopy(real_point)

    """
    real_point 物体的实际位置
    clamp_euler 夹爪夹取姿态
    clamp_T 夹爪偏移矩阵
    函数名：获取夹取点
    """

    def GetThePinchPoint(self, real_point, clamp_euler, clamp_T):  #获取夹取点
        clamp_T_R = self.revolve(clamp_euler, clamp_T)
        clamp_point = real_point - clamp_T_R

        return copy.deepcopy(clamp_point)

    """
    real_point 物体的实际点位
    camera_T 相机偏移
    ident_euler 二次识别时的姿态
    hight 再次识别物体，相机与物体的高度
    函数名：获取再次识别点
    """

    def GetTwiceIdentPoint(self, real_point, camera_T, ident_euler, hight):  #获取二次识别点位
        vector_target = self.revolve(ident_euler, self.RotationMatrix @ np.array([0, 0, hight]))
        camera_T_R = self.revolve(ident_euler, camera_T)
        twice_ident_point = real_point - vector_target - camera_T_R

        return copy.deepcopy(twice_ident_point)

    """
    input_list 输入列表
    limit 误差范围
    函数功能：实现将输入列表中误差范围小于指定范围的点归一化
    """

    def GetOneList(self, input_list, limit):
        output_list = []
        while len(input_list) != 0:
            first = input_list[0]
            flag = 0
            for i in output_list:
                distanceItoFirst = ((first[0] - i[0]) ** 2 + (first[1] - i[1]) ** 2 + (first[2] - i[2]) ** 2) ** 0.5
                if distanceItoFirst <= limit:
                    flag = 1
                    break
            if flag == 0:
                output_list.append(copy.deepcopy(first))
            input_list.remove(first)

        return output_list

    """
    input_list 输入列表
    lenth 长度
    函数功能：计算出与上挂钩端点距离为lenth长度的一个点
    """

    def GetAboveVector(self, input_list, lenth):
        P0 = np.array([input_list[0][0], input_list[0][1], input_list[0][2]])  #二维码坐标1
        P1 = np.array([input_list[1][0], input_list[1][1], input_list[1][2]])  #二维码坐标2
        P2 = np.array([input_list[2][0], input_list[2][1], input_list[2][2]])  #挂钩1起点
        P3 = np.array([input_list[3][0], input_list[3][1], input_list[3][2]])  #挂钩1终点

        a = P3 - P2  #挂钩方向向量
        b = P1 - P0  #二维码方向向量
        c = np.cross(a, b)  #公垂向量
        d = np.array([[a[0], a[1], a[2]], [b[0], b[1], b[2]], [c[0], c[1], c[2]]])
        inverse_d = np.linalg.inv(d)

        e = inverse_d @ (P0 - P2)  #求三个方向系数

        P4 = P2 + a * e[0]  #公垂线与挂钩交点
        P6 = P0 - b * e[1]  #公垂线与二维码连线交点

        P5 = P4 + a / np.linalg.norm(a) * lenth  #与上挂钩端点距离lenth长度的一个点

        return P5

    """
    input_list 输入列表
    lenth 长度
    函数功能：计算出与上挂钩端点距离为lenth长度的一个点
    """

    def GetBelowVector(self, input_list, hight, lenth):
        P0 = np.array([input_list[0][0], input_list[0][1], input_list[0][2]])  #二维码坐标1
        P1 = np.array([input_list[1][0], input_list[1][1], input_list[1][2]])  #二维码坐标2
        P2 = np.array([input_list[2][0], input_list[2][1], input_list[2][2]])  #挂钩起点
        P3 = np.array([input_list[3][0], input_list[3][1], input_list[3][2]])  #挂钩终点

        a = P3 - P2  #挂钩方向向量
        b = P1 - P0  #二维码方向向量
        c = np.cross(a, b)  #公垂向量
        d = np.array([[a[0], a[1], a[2]], [b[0], b[1], b[2]], [c[0], c[1], c[2]]])
        inverse_d = np.linalg.inv(d)

        e = inverse_d @ (P0 - P2)  #求三个方向系数

        P4 = P2 + a * e[0]  #公垂线与挂钩交点
        P6 = P0 - b * e[1]  #公垂线与二维码连线交点

        h = b[2]  #二维码高度差
        l = (b[0] ** 2 + b[1] ** 2) ** 0.5  #二维码距离差
        angle = math.atan2(h, l)  #两二维码连线倾斜角（即横杠在机械臂坐标系的倾斜角）
        Hight = hight * math.cos(angle)  #挂钩与二维码的距离差（机械臂坐标系中）

        r = P3 - P2  #挂钩1方向向量
        P8 = P2 + r * ((P6[2] - Hight - P2[2]) / (r[2]))  #挂钩1真实起点
        P11 = P8 + r / np.linalg.norm(r) * lenth  #与挂钩1下挂钩端点距离为lenth的点

        return P11

    #"""
    #input_list 输入列表
    #lenth 长度
    #函数功能：计算出与上挂钩端点距离为lenth长度的一个点
    #"""
    #def GetBelowVector(self,input_list,hight,lenth):
    #    P0 = np.array([input_list[0][0],input_list[0][1],input_list[0][2]])#二维码坐标1
    #    P1 = np.array([input_list[1][0],input_list[1][1],input_list[1][2]])#二维码坐标2
    #    P2 = np.array([input_list[2][0],input_list[2][1],input_list[2][2]])#挂钩起点
    #    P3 = np.array([input_list[3][0],input_list[3][1],input_list[3][2]])#挂钩终点

    #    e = P1 - P0                           #二维码方向向量         
    #    a = P0-0.5*e                          #挂钩1在二维码直线上的投影点

    #    h = e[2]                              #二维码高度差                                                      
    #    l = (e[0]**2+e[1]**2)**0.5            #二维码距离差
    #    angle = math.atan2(h,l)               #两二维码连线倾斜角（即横杠在机械臂坐标系的倾斜角）
    #    Hight = hight*math.cos(angle)         #挂钩与二维码的距离差（机械臂坐标系中）

    #    r = P3 - P2                           #挂钩1方向向量
    #    P8  = P2+r*((a[2]-Hight-P2[2])/(r[2]))  #挂钩1真实起点
    #    P11 = P8 +r/np.linalg.norm(r)*lenth   #与挂钩1下挂钩端点距离为lenth的点

    #    return P11

    """
    input_list 输入列表
    lenth 长度
    函数功能：计算出所有上挂钩点位
    """

    def GetAllAboveVector(self, input_list, lenth1, lenth2):
        output_list = []

        for i in range(0, int((len(input_list) - 2) / 2)):
            input_list2 = []
            input_list2.append(input_list[0])
            input_list2.append(input_list[1])
            input_list2.append(input_list[2 * i + 2])
            input_list2.append(input_list[2 * i + 3])
            pos1 = self.GetAboveVector(input_list2, lenth1)
            pos2 = self.GetAboveVector(input_list2, lenth2)
            output_list.append(pos1)
            output_list.append(pos2)

        return output_list

    """
    input_list 输入列表
    lenth 长度
    函数功能：计算出所有下挂钩点位
    """

    def GetAllBelowVector(self, input_list, hight, lenth1, lenth2):
        output_list = []
        for i in range(0, int((len(input_list) - 2) / 2)):
            input_list2 = []
            input_list2.append(input_list[0])
            input_list2.append(input_list[1])
            input_list2.append(input_list[2 * i + 2])
            input_list2.append(input_list[2 * i + 3])
            pos1 = self.GetBelowVector(input_list2, hight, lenth1)
            pos2 = self.GetBelowVector(input_list2, hight, lenth2)
            output_list.append(pos1)
            output_list.append(pos2)

        return output_list

    #"""
    #input_list 输入列表
    #lenth 长度
    #函数功能：计算出所有下挂钩点位
    #"""
    #def GetAllBelowVector(self,input_list,hight,lenth1,lenth2):
    #    output_list = []
    #    for i in range(0,(len(input_list)-2)/2):
    #        input_list2 = []
    #        input_list2.append(input_list[0])
    #        input_list2.append(input_list[1])
    #        input_list2.append(input_list[2*i+2])
    #        input_list2.append(input_list[2*i+3])
    #        pos1 = self.GetBelowVector(input_list2,hight,lenth1)
    #        pos2 = self.GetBelowVector(input_list2,hight,lenth2)
    #        output_list.append(pos1,pos2)  

    #    return output_list

    """
    input1 坐标1
    input2 坐标2
    函数功能：计算旋转角度
    """

    def GetRevolveAngle(self, input1, input2):
        P0 = np.array([input1[0], input1[1], input1[2]])  #坐标1
        P1 = np.array([input2[0], input2[1], input2[2]])  #坐标2
        e = P1 - P0
        angle = math.atan2(e[1], e[0]) / math.pi * 180

        return angle
