import sys
import socket
import threading
import time
import sys
import os
import traceback
import queue
import json
import re
import logging
from logging import handlers


#机械臂通信收发类
class SCARAConnect(threading.Thread):
    sck = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    def __init__(self, st_host, n_port):
        threading.Thread.__init__(self, name="SCARAConnect")
        self.lock = threading.Lock()
        self.st_host = st_host
        self.n_port = n_port
        # 发送接收数据队列
        self.rqueueGetPostion = queue.Queue()
        self.rqueueModifyOutput = queue.Queue()
        self.rqueueRewriteDataList = queue.Queue()
        self.rqueueSetSpeed = queue.Queue()
        self.rqueueStartButton = queue.Queue()
        self.rqueueStopButton = queue.Queue()
        self.rqueueActionStop = queue.Queue()
        self.rqueueModifyCounter = queue.Queue()

        self.squeue = queue.Queue()
        # 发送线程
        self.sendthread = threading.Thread(target=self.Sender, name='SCARASender', daemon=True)
        self.state = threading.Condition()
        self.sendPaused = True
        self.rTimeErr = 0
        self.recvTag = 0
        self.rTime = 5
        self.currentPos = {'x': 0, 'y': 0, 'z': 0, 'u': 0, 'v': 0, 'w': 0, 'ID': 0}
        self.currentaxle = {'J1': 0, 'J2': 0, 'J3': 0, 'J4': 0, 'J5': 0, 'J6': 0, 'ID': 0}
        self.log = loginit()
        self.log.logger.info("机械臂程序模块初始化")

    # 主线程，接收解析
    def run(self):
        # 启动发送线程
        self.sendthread.start()
        self.doConnect()
        while True:
            try:
                # wait recv
                data = SCARAConnect.sck.recv(1024)  #接收数据
                if len(data):
                    self.rTimeErr = 0
                    self.recvTag = 1
                    self.ParsePackage(data)

            except OSError:
                traceback.print_exc()
                self.threadPause()
                time.sleep(2)
                self.log.logger.info('SCARA connect error, doing connect in 2s ....')
                SCARAConnect.sck = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.doConnect()
            except Exception as e:
                self.log.logger.info('other error occur:{}'.format(str(e)))
                traceback.print_exc()
                self.threadPause()
                time.sleep(4)
                if SCARAConnect.sck._closed:
                    SCARAConnect.sck = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.doConnect()
                #if SCARAConnect.sck._closed:

    # 重连
    def doConnect(self):
        while True:
            try:
                #self.sck.settimeout(1)
                SCARAConnect.sck.connect((self.st_host, self.n_port))
                self.threadResume()
                time.sleep(1)
                self.log.logger.info('connect to SCARA {}:{}'.format(self.st_host, self.n_port))
                break
            except ConnectionRefusedError:
                self.log.logger.info('SCARA refused or not started, reconnect to SCARA in 3s ...')
                time.sleep(3)

            except Exception as e:
                traceback.print_exc()
                self.log.logger.info('do connect error:{}'.format(str(e)))
                time.sleep(5)

    # 恢复线程运行
    def threadResume(self):
        with self.state:
            self.sendPaused = False
            self.state.notify()

    # 超时判断
    def TimeErr(self):
        if self.rTimeErr >= 3:
            self.threadPause()
            time.sleep(4)
            self.rTimeErr = 0
            SCARAConnect.sck = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.doConnect()

    # 接收数据解析
    def ParsePackage(self, data):
        # 数据转成json格式
        pattern = r'({.*?})'
        r1 = re.findall(pattern, str(data))
        for i in r1:
            packages = json.loads(i)
            if "cmdReply" in packages:
                #设置中间变量M0
                if packages["cmdReply"] == ['modifyOutput', 'ok']:
                    self.rqueueModifyOutput.put(packages)
                #下发坐标
                if packages["cmdReply"] == ['rewriteDataList', 'ok']:
                    self.rqueueRewriteDataList.put(packages)
                #设置速度
                if packages["cmdReply"] == ['modifyGSPD', 'ok']:
                    self.rqueueSetSpeed.put(packages)
                #启动机械臂
                if packages["cmdReply"] == ['startButton', 'ok']:
                    self.rqueueStartButton.put(packages)
                #停止机械臂
                if packages["cmdReply"] == ['stopButton', 'ok']:
                    self.rqueueStopButton.put(packages)
                #停止机械臂2
                if packages["cmdReply"] == ['actionStop', 'ok']:
                    self.rqueueActionStop.put(packages)
                if packages["cmdReply"] == ['modifyCounter', 'ok']:
                    self.rqueueModifyCounter.put(packages)

            elif "cmdType" in packages:
                # 获取当前点位
                if packages["cmdType"] == 'query':
                    self.rqueueGetPostion.put(packages)

    # 发送线程
    def Sender(self):
        while True:
            self.TimeErr()

            with self.state:
                if self.sendPaused:
                    self.state.wait()
            try:
                msg = self.squeue.get()
                if 'queryAddr' not in str(msg):
                    self.log.logger.info("send:{}".format(msg))
                SCARAConnect.sck.send(msg)  #发送数据
                # 限流
                for i in range(0, 300):
                    if self.recvTag == 1:
                        break;
                    else:
                        time.sleep(0.01)
                if self.recvTag == 1:
                    time.sleep(0.1)
                self.recvTag = 0
            except OSError:
                traceback.print_exc()
                self.threadPause()
            except Exception as e:
                self.log.logger.info('other error occur:{}'.format(str(e)))
                traceback.print_exc()
                self.threadPause()

    # 挂起线程
    def threadPause(self):
        with self.state:
            self.sendPaused = True

    #获取当前坐标和计数器信息
    def GetPostion(self):
        msg = b"{\"dsID\":\"HCRemoteMonitor\",\"cmdType\":\"query\",\"queryAddr\":[\"world-0\",\"world-1\",\"world-2\",\"world-3\",\"world-4\",\"world-5\",\"counter-0\"]}"
        self.rqueueGetPostion.queue.clear()
        self.squeue.put(msg)
        while True:
            try:
                data = self.rqueueGetPostion.get(timeout=self.rTime)
            except Exception as e:
                self.log.logger.info('GetPostion error:{}'.format(str(e)))
                self.rTimeErr += 1
                break
            currentPos = {'x': 0, 'y': 0, 'z': 0, 'u': 0, 'v': 0, 'w': 0, 'ID': 0}
            currentPos['x'] = float(data['queryData'][0])
            currentPos['y'] = float(data['queryData'][1])
            currentPos['z'] = float(data['queryData'][2])
            currentPos['u'] = float(data['queryData'][3])
            currentPos['v'] = float(data['queryData'][4])
            currentPos['w'] = float(data['queryData'][5])
            currentPos['ID'] = float(data['queryData'][6][2])
            return currentPos
        return False, []

    def Postionaxle(self):
        msg = b"{\"dsID\":\"HCRemoteMonitor\",\"cmdType\":\"query\",\"queryAddr\":[\"axis-0\",\"axis-1\",\"axis-2\",\"axis-3\",\"axis-4\",\"axis-5\",\"counter-0\"]}"
        self.rqueueGetPostion.queue.clear()
        self.squeue.put(msg)
        while True:
            try:
                data = self.rqueueGetPostion.get(timeout=self.rTime)
            except Exception as e:
                self.log.logger.info('GetPostion error:{}'.format(str(e)))
                self.rTimeErr += 1
                break
            self.currentaxle['J1'] = float(data['queryData'][0])
            self.currentaxle['J2'] = float(data['queryData'][1])
            self.currentaxle['J3'] = float(data['queryData'][2])
            self.currentaxle['J4'] = float(data['queryData'][3])
            self.currentaxle['J5'] = float(data['queryData'][4])
            self.currentaxle['J6'] = float(data['queryData'][5])
            self.currentPos['ID'] = float(data['queryData'][6][2])
            return True
        return False

    #设置中间变量M0
    def modifyOutput(self, register):
        msg = "{\"dsID\":\"HCRemoteMonitor\",\"cmdType\":\"command\",\"cmdData\":[\"modifyOutput\",\"" + str(
            register["borad"]) + "\",\"" + str(register["point"]) + "\",\"" + str(register["funtion"]) + "\"]}"
        self.rqueueModifyOutput.queue.clear()
        self.squeue.put(bytes(msg, encoding='utf8'))

        while True:
            try:
                data = self.rqueueModifyOutput.get(timeout=self.rTime)
            except Exception as e:
                self.log.logger.info('modifyOutput error:{}'.format(str(e)))
                self.rTimeErr += 1
                break
            self.log.logger.info('recv data:{}'.format(data))
            return True
        return False

    #下发单个坐标
    def rewriteDataList(self, pos):
        msg = "{\"dsID\":\"HCRemoteMonitor\",\"cmdType\":\"command\",\"cmdData\":[\"rewriteDataList\",\"" + str(
            pos["addr"]) + "\",\"6\",\"0\""
        msg += ",\"" + str(pos['x'] * 1000) + "\""
        msg += ",\"" + str(pos['y'] * 1000) + "\""
        msg += ",\"" + str(pos['z'] * 1000) + "\""
        msg += ",\"" + str(pos['u'] * 1000) + "\""
        msg += ",\"" + str(pos['v'] * 1000) + "\""
        msg += ",\"" + str(pos['w'] * 1000) + "\""
        msg += "]}"
        self.rqueueRewriteDataList.queue.clear()
        self.squeue.put(bytes(msg, encoding='utf8'))
        while True:
            try:
                data = self.rqueueRewriteDataList.get(timeout=self.rTime)
            except Exception as e:
                self.log.logger.info('rewriteDataList error:{}'.format(str(e)))
                self.rTimeErr += 1
                break
            self.log.logger.info('recv data:{}'.format(data))
            return True
        return False

    #下发多个坐标
    def rewriteDataListStr(self, link):
        msg = "{\"dsID\":\"HCRemoteMonitor\",\"cmdType\":\"command\",\"cmdData\":[\"rewriteDataList\",\"" + str(
            link["addr"]) + "\",\"" + str(link["poslen"]) + "\",\"0"
        msg += str(link['pos'])
        msg += "\"]}"
        self.rqueueRewriteDataList.queue.clear()
        self.squeue.put(bytes(msg, encoding='utf8'))
        while True:
            try:
                data = self.rqueueRewriteDataList.get(timeout=self.rTime)
            except Exception as e:
                self.log.logger.info('rewriteDataListStr error:{}'.format(str(e)))
                self.rTimeErr += 1
                break
            self.log.logger.info('recv data:{}'.format(data))
            return True
        return False

    #设置速度
    def setSpeed(self, link):
        msg = "{\"dsID\":\"HCRemoteMonitor\",\"cmdType\":\"command\",\"cmdData\":[\"modifyGSPD\",\"" + str(
            link["speed"]) + "\"]}"
        self.rqueueSetSpeed.queue.clear()
        self.squeue.put(bytes(msg, encoding='utf8'))
        while True:
            try:
                data = self.rqueueSetSpeed.get(timeout=self.rTime)
            except Exception as e:
                self.log.logger.info('setSpeed error:{}'.format(str(e)))
                self.rTimeErr += 1
                break
            self.log.logger.info('recv data:{}'.format(data))
            return True
        return False

    #启动机械臂
    def startButton(self):
        msg = b"{\"dsID\":\"HCRemoteMonitor\",\"cmdType\":\"command\",\"cmdData\":[\"startButton\"]}"
        self.rqueueStartButton.queue.clear()
        self.squeue.put(msg)
        while True:
            try:
                data = self.rqueueStartButton.get(timeout=self.rTime)
            except Exception as e:
                self.log.logger.info('startButton error:{}'.format(str(e)))
                self.rTimeErr += 1
                break
            self.log.logger.info('recv data:{}'.format(data))
            return True
        return False

    #停止机械臂
    def stopButton(self):
        msg = b"{\"dsID\":\"HCRemoteMonitor\",\"cmdType\":\"command\",\"cmdData\":[\"stopButton\"]}"
        self.rqueueStopButton.queue.clear()
        self.squeue.put(msg)
        while True:
            try:
                data = self.rqueueStopButton.get(timeout=self.rTime)
            except Exception as e:
                self.log.logger.info('stopButton error:{}，请检查机械臂网络连接是否正常'.format(str(e)))
                self.rTimeErr += 1
                break
            self.log.logger.info('recv data:{}'.format(data))
            return True
        return False

    #设置计数器
    def modifyCounter(self, counter):
        msg = "{\"dsID\":\"HCRemoteMonitor\",\"cmdType\":\"command\",\"cmdData\":[\"modifyCounter\",\"counter-" + str(
            counter['number']) + "\",\"" + str(counter['value']) + "\",\"-1\"]}"
        self.rqueueModifyCounter.queue.clear()
        self.squeue.put(bytes(msg, encoding='utf8'))
        while True:
            try:
                data = self.rqueueModifyCounter.get(timeout=self.rTime)
            except Exception as e:
                self.log.logger.info('modifyOutput error:{}'.format(str(e)))
                self.rTimeErr += 1
                break
            #self.log.logger.info('recv data:{}'.format(data))
            return True
        return False


class Logger(object):
    level_relations = {
        'debug': logging.DEBUG,
        'info': logging.INFO,
        'warning': logging.WARNING,
        'error': logging.ERROR,
        'crit': logging.CRITICAL
    }  #日志级别关系映射

    def __init__(self, filename, level='info', when='D', backCount=3,
                 fmt='【%(asctime)s】  %(message)s'):  #- %(pathname)s[line:%(lineno)d] - %(levelname)s:
        self.logger = logging.getLogger(filename)
        format_str = logging.Formatter(fmt)  #设置日志格式
        self.logger.setLevel(self.level_relations.get(level))  #设置日志级别
        sh = logging.StreamHandler()  #往屏幕上输出
        sh.setFormatter(format_str)  #设置屏幕上显示的格式
        th = handlers.TimedRotatingFileHandler(filename=filename, when=when, backupCount=backCount,
                                               encoding='utf-8')  #往文件里写入#指定间隔时间自动生成文件的处理器
        #实例化TimedRotatingFileHandler
        #interval是时间间隔，backupCount是备份文件的个数，如果超过这个个数，就会自动删除，when是间隔的时间单位，单位有以下几种：
        # S 秒
        # M 分
        # H 小时、
        # D 天、
        # W 每星期（interval==0时代表星期一）
        # midnight 每天凌晨
        th.setFormatter(format_str)  #设置文件里写入的格式
        self.logger.addHandler(sh)  #把对象加到logger里
        self.logger.addHandler(th)


def loginit():
    basedir = os.path.abspath(os.path.dirname(__file__))
    log_path = os.path.join(r'.\Logs', 'scaraLogs')  # 日志根目录 ../logs/
    if not os.path.exists(log_path):
        os.mkdir(log_path)
    log_filename = time.strftime("%F") + '.log'
    log_name = os.path.join(log_path, log_filename)
    return Logger(log_name, level='debug')

#def main():
#    scara = SCARAConnect("127.0.0.1",9760)
#    scara.start()

#if __name__ == "__main__":
#    try:
#        main()
#    except Exception as e:
#        print("ScaraDriver Error:" + str(e.args))
