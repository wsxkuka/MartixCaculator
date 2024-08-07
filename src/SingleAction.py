import copy

import numpy as np
import math
from src.ScaraDriver import *
from Logger import Logger

log = loginit()


class SingleAction(threading.Thread):
    def __init__(self, scara):
        threading.Thread.__init__(self, name="SingleAction")
        self.lock = threading.Lock()
        self.scara = scara
        self.links = []
        self.stepEnble = True
        self.log = loginit()
        self.log.logger.info("单动作程序模块初始化")
        self.currentPos = {'x': 460, 'y': 0, 'z': 500, 'u': 0, 'v': 0, 'w': 180, 'ID': 0}

    def run(self):
        while True:
            if self.stepEnble:
                link = self.getStep()
                if link:
                    self.executionStep(link)
            time.sleep(0.01)

    """
        获取机械臂动作步骤
        函数名：getStep()  
    """

    def getStep(self):
        try:
            if len(self.links) == 0:
                return False
            for link in self.links:
                if link['State'] == True:
                    return link
            return False
        except Exception as e:
            self.log.logger.info('SCARA Error:' + str(e.args))

    """
        函数名：executionStep()  
        将列表中的单条信息转换成报文发送出去
    """

    def executionStep(self, link):
        if link == None:
            return

        #0计数器确认位置；1中间变量M0；2下发坐标；3下发速度；4获取当前坐标让机械臂到n托盘或n缓存区识别点；5开始；6结束；7PLC机械臂夹爪
        #0 确认位置
        if link['typeID'] == "getPostion":
            result = self.scara.GetPostion()
            if result[0] == True:
                self.currentPos = result[1]
                if self.currentPos['ID'] >= link['minID'] and self.currentPos["ID"] <= link['maxID']:
                    link['State'] = False

        #1中间变量M010、M011等
        if link['typeID'] == "modifyOutput":
            if self.scara.modifyOutput(link):
                link['State'] = False
        #2下发单个坐标
        if link['typeID'] == "rewriteDataList":
            if self.scara.rewriteDataList(link):
                link['State'] = False
        #3下发速度
        if link['typeID'] == "setSpeed":
            if self.scara.setSpeed(link):
                link['State'] = False
        #4获取当前关节角度
        if link['typeID'] == "getAngle":
            if self.scara.Postionaxle():
                link['State'] = False
        #5开始
        if link['typeID'] == "startButton":
            if self.scara.startButton():
                link['State'] = False
        #6结束
        if link['typeID'] == "stopButton":
            if self.scara.stopButton():
                link['State'] = False
        #7设置计数器
        if link['typeID'] == "modifyCounter":
            if self.scara.modifyCounter(link):
                link['State'] = False
        #8下发多个坐标 
        if link['typeID'] == "setOfCoordinates":
            #self.log.logger.info("typeID = 8")
            if self.scara.rewriteDataListStr(link):
                link['State'] = False

        #9下发多个坐标
        if link['typeID'] == "delay":
            time.sleep(link['time'])
            link['State'] = False

    """
        函数名：timeoutDetection
        超时检测
    """

    def timeoutDetection(self, timeout):
        timeCount = 0
        singleTime = 0.01
        errorCode = ""
        while self.getStep() != False:
            time.sleep(singleTime)
            timeCount += singleTime
            if timeCount > timeout:
                errorCode = "robotInit timeout"
                self.links = []
                return False, errorCode
        return True, errorCode

    """
        函数名：robotInit()  
        初始化，停止当前动作，发送启动
    """

    def robotInit(self, speed, timeout) -> None:

        links = []
        links.append({'State': True, 'typeID': "stopButton"})
        links.append({'State': True, 'typeID': "stopButton"})
        links.append({'State': True, 'typeID': "delay", 'time': 1})
        links.append({'State': True, 'typeID': "modifyCounter", 'number': 0, 'value': 0})
        links.append({'State': True, 'typeID': "getPostion", 'minID': 0, 'maxID': 0})
        links.append({'State': True, 'typeID': "setSpeed", 'speed': min(int(speed * 10), 1000)})
        links.append({'State': True, 'typeID': "modifyOutput", 'borad': 4, 'point': 0, 'funtion': 0})  #M010 OFF
        links.append({'State': True, 'typeID': "modifyOutput", 'borad': 4, 'point': 1, 'funtion': 0})  #M011 OFF
        links.append({'State': True, 'typeID': "modifyOutput", 'borad': 4, 'point': 2, 'funtion': 0})  #M012 OFF
        links.append({'State': True, 'typeID': "modifyOutput", 'borad': 4, 'point': 3, 'funtion': 0})  #M013 OFF
        links.append({'State': True, 'typeID': "modifyOutput", 'borad': 4, 'point': 4, 'funtion': 0})  #M014 OFF
        links.append({'State': True, 'typeID': "modifyOutput", 'borad': 4, 'point': 5, 'funtion': 0})  #M015 OFF
        links.append({'State': True, 'typeID': "modifyOutput", 'borad': 4, 'point': 6, 'funtion': 0})  #M016 OFF
        links.append({'State': True, 'typeID': "modifyOutput", 'borad': 4, 'point': 7, 'funtion': 0})  #M017 OFF
        links.append({'State': True, 'typeID': "startButton"})
        self.links = links

        canProceed, errorCode = self.timeoutDetection(timeout)
        if not canProceed:
            log.logger.info(errorCode)
            return 0

    """
        函数名：Straight_line_Motion()  
        从当前位置直线移动到目标位置
    """

    def Straight_line_Motion(self, targetPoint, timeout):
        poss1 = ""
        p = targetPoint
        poss1 += "\"" + "," + "\"" + str(int(p['x']) * 1000)
        poss1 += "\"" + "," + "\"" + str(int(p['y']) * 1000)
        poss1 += "\"" + "," + "\"" + str(int(p['z']) * 1000)
        poss1 += "\"" + "," + "\"" + str(int(p['u']) * 1000)
        poss1 += "\"" + "," + "\"" + str(int(p['v']) * 1000)
        poss1 += "\"" + "," + "\"" + str(int(p['w']) * 1000)
        links = []
        if self.currentPos['ID'] != 1 and self.currentPos['ID'] != 2:
            links.append({'State': True, 'typeID': "setOfCoordinates", 'addr': 800, 'poslen': 6, 'pos': poss1})
            links.append({'State': True, 'typeID': "modifyOutput", 'borad': 4, 'point': 0, 'funtion': 1})
            links.append({'State': True, 'typeID': "getPostion", 'minID': 2, 'maxID': 2})
        else:
            links.append({'State': True, 'typeID': "setOfCoordinates", 'addr': 806, 'poslen': 6, 'pos': poss1})
            links.append({'State': True, 'typeID': "modifyOutput", 'borad': 4, 'point': 1, 'funtion': 1})
            links.append({'State': True, 'typeID': "getPostion", 'minID': 4, 'maxID': 4})
        self.links = links
        canProceed, errorCode = self.timeoutDetection(timeout)
        if not canProceed:
            log.logger.info(errorCode)
            return False, errorCode
        return True

    """
    函数名：senCoordinates()  
    发送6个坐标
    """

    def senCoordinates(self, targetPoint, HEIGHT, timeout):
        poss = self.route_planning(targetPoint, HEIGHT)
        print(poss)
        line = len(poss)
        errorCode = ""
        if line != 6:
            errorCode = "坐标长度不等于6"
            return False, errorCode
        poss1 = ""
        try:
            for p in poss:
                poss1 += "\"" + "," + "\"" + str(int(p['x']) * 1000)
                poss1 += "\"" + "," + "\"" + str(int(p['y']) * 1000)
                poss1 += "\"" + "," + "\"" + str(int(p['z']) * 1000)
                poss1 += "\"" + "," + "\"" + str(int(p['u']) * 1000)
                poss1 += "\"" + "," + "\"" + str(int(p['v']) * 1000)
                poss1 += "\"" + "," + "\"" + str(int(p['w']) * 1000)
            links = []
            if self.currentPos['ID'] != 5 and self.currentPos['ID'] != 6:
                links.append(
                    {'State': True, 'typeID': "setOfCoordinates", 'addr': 812, 'poslen': line * 6, 'pos': poss1})
                links.append({'State': True, 'typeID': "modifyOutput", 'borad': 4, 'point': 2, 'funtion': 1})
                links.append({'State': True, 'typeID': "getPostion", 'minID': 6, 'maxID': 6})
            else:
                links.append(
                    {'State': True, 'typeID': "setOfCoordinates", 'addr': 848, 'poslen': line * 6, 'pos': poss1})
                links.append({'State': True, 'typeID': "modifyOutput", 'borad': 4, 'point': 3, 'funtion': 1})
                links.append({'State': True, 'typeID': "getPostion", 'minID': 8, 'maxID': 8})
            self.links = links
        except Exception as e:
            errorCode = "坐标格式错误"
            log.logger.error(errorCode)
            return 0
        canProceed, errorCode = self.timeoutDetection(timeout)
        if not canProceed:
            log.logger.info(errorCode)
            return 0

    """
    回起始点
    函数名：initialPosition
    """

    def initialPosition(self, timeout) -> [bool, str]:
        links = []
        if self.currentPos['ID'] != 9 and self.currentPos['ID'] != 10:
            links.append({'State': True, 'typeID': "modifyOutput", 'borad': 4, 'point': 4, 'funtion': 1})
            links.append({'State': True, 'typeID': "getPostion", 'minID': 10, 'maxID': 10})
        else:
            links.append({'State': True, 'typeID': "modifyOutput", 'borad': 4, 'point': 5, 'funtion': 1})
            links.append({'State': True, 'typeID': "getPostion", 'minID': 12, 'maxID': 12})
        self.links = links
        canProceed, errorCode = self.timeoutDetection(timeout)
        if not canProceed:
            log.logger.info(errorCode)
            return 0

        """
    euler 欧拉角
    vector 坐标
    功能：根据欧拉角计算出带角度的坐标
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

    def angle_J1(self, euler_tar, vector_tar):
        JS = []
        a = np.array([169.588, 0.498, 494.6])  #(X1ecc,-Y1ecc,Z)
        b = np.array([0, 0, 730.870])  #(0,0,L23)
        c = np.array([826.042, 0, 99.301])  #(L34b,0,L34a)
        d = np.array([164.00, 0, 0])  #(L56,0,0)
        d2 = np.array([0, 0, 164.00])  #(0,0,L56)
        P2 = vector_tar - self.revolve(euler_tar, d2)

        J1 = math.atan2(vector_tar[1], vector_tar[0]) / math.pi * 180
        return J1

    def gripperControl(self, timeout, state):
        links = []
        links.append({'State': True, 'typeID': "modifyOutput", 'borad': 0, 'point': 8, 'funtion': state})
        self.links = links
        return self.timeoutDetection(timeout)
    """
    startPos 起始点
    EndPos 目标点
    hight 过渡点高度
    函数功能：从当前位置到结束点加5个中间过渡点位实现门字形动作
    """

    def route_planning(self, EndPos, hight) -> list:

        poss = []
        startPos = self.currentPos

        vector_start = np.array([startPos['x'], startPos['y'], startPos['z']])
        euler_start = {'u': startPos['u'], 'v': startPos['v'], 'w': startPos['w']}
        JStart = self.angle_J1(euler_start, vector_start)

        vector_end = np.array([EndPos['x'], EndPos['y'], EndPos['z']])
        euler_end = {'u': EndPos['u'], 'v': EndPos['v'], 'w': EndPos['w']}
        JEnd = self.angle_J1(euler_end, vector_end)

        # if euler_end['u'] != 0 or euler_end['v'] != 180:
        #     errorCode = "输入坐标格式错误"
        #     log.logger.error(errorCode)
        #     return 0

        t_start = self.revolve(euler_start, np.array([-100, 0, 0]))
        t_end = self.revolve(euler_end, np.array([-100, 0, 0]))

        w_start = math.atan2(t_start[1], t_start[0]) / math.pi * 180
        w_end = math.atan2(t_end[1], t_end[0]) / math.pi * 180

        w_point_start = math.atan2(startPos['y'], startPos['x']) / math.pi * 180
        w_point_end = math.atan2(EndPos['y'], EndPos['x']) / math.pi * 180

        r_start = (startPos['x'] ** 2 + startPos['y'] ** 2) ** 0.5
        r_end = (EndPos['x'] ** 2 + EndPos['y'] ** 2) ** 0.5

        point = copy.deepcopy(startPos)
        point['z'] = hight
        poss.append(point)

        for i in range(0, 3):
            point = {'x': 0, 'y': 0, 'z': 0, 'u': 0, 'v': 180, 'w': 0}
            w_point = w_point_start + (w_point_end - w_point_start) / 4 * (i + 1)
            r = r_start + (r_end - r_start) / 4 * (i + 1)
            point['x'] = r * math.cos(w_point / 180 * math.pi)
            point['y'] = r * math.sin(w_point / 180 * math.pi)
            point['z'] = hight
            point['w'] = w_start + (w_end - w_start) / 4 * (i + 1)
            poss.append(point)

        point = copy.deepcopy(EndPos)
        point['z'] = hight
        poss.append(point)
        poss.append(EndPos)
        return poss


def loginit():
    basedir = os.path.abspath(os.path.dirname(__file__))
    log_path = os.path.join(r'.\Logs', 'SingleActionLogs')  # 日志根目录 ../logs/
    if not os.path.exists(log_path):
        os.mkdir(log_path)
    log_filename = time.strftime("%F") + '.log'
    log_name = os.path.join(log_path, log_filename)
    return Logger(log_name, level='debug')
