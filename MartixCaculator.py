import sys
import threading
import socket
from configparser import ConfigParser
from datetime import datetime
import os

import pandas as pd
from PyQt5 import QtGui, QtWidgets, QtCore
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QApplication, QWidget, QTableWidgetItem, QMessageBox
import numpy as np

from MartixCaculator_UI import Ui_Form
from src.ScaraDriver import SCARAConnect
from src.caculation import calculation
from src.SingleAction import SingleAction


class EmittingStr(QtCore.QObject):
    textWritten = QtCore.pyqtSignal(str)  # 定义信号

    def write(self, text):
        self.textWritten.emit(str(text))  # emit()发射信号


class MartixCaculator(QWidget, Ui_Form):
    sig = pyqtSignal()
    qr_signal = pyqtSignal(str, str)

    def __init__(self):
        super().__init__()
        self.qrcodes = None
        self.RotationMatrix = None
        self.folder_path = None
        self.camera_pos = None
        self.ident_point = None
        self.curpath = os.path.dirname(os.path.realpath(__file__))  # 当前文件路径
        self.inipath = os.path.join(self.curpath, "param_store.ini")
        self.conf = ConfigParser()
        self.conf.read(self.inipath, encoding="utf-8")
        self.ident_euler = None
        self.real_point = None
        self.camera_T = np.array(eval(self.conf['param']['camera_T']))
        self.form2 = None
        self.NewWindow = None
        self.setupUi(self)
        self.scara = None
        self.singleAct = None
        self.clamp_T = np.array(eval(self.conf['param']['clamp_T']))
        self.client_socket = None
        self.now_cmr_pos = None
        self._is_arm_connected = False
        self._is_camera_connected = False
        column_names = ["时间", '识别姿态', '相机旋转矩阵', '相机偏移矩阵', '夹爪偏移矩阵',
                        '二维码坐标计算值(探针)', '二维码坐标计算值（相机）', '识别点坐标',
                        '视觉坐标', '二维码坐标实际值（示教器)', '夹爪坐标']
        self.export_data = pd.DataFrame(columns=column_names)
        self.software_init()  # 软件初始化，创建数据目录
        self.initial_param()
        self.cal = calculation(self.RotationMatrix)

        sys.stdout = EmittingStr(textWritten=self._outputWritten)  # 输出流，把文本输出到指定位置
        sys.stderr = EmittingStr(textWritten=self._outputWritten)
        self.signal_init()

    def signal_init(self):
        self.btn_pinch_get.clicked.connect(self.pinch_getData)  # 夹爪：采集数据
        self.btn_cmr_get.clicked.connect(self.camera_getData)  # 相机：采集数据
        self.btn_cmr_connect.clicked.connect(self.camera_connect)  # 连接相机的按钮
        self.btn_arm_connect.clicked.connect(self.arm_connect)  # 连接机械臂的按钮
        self.btn_cmr_output.clicked.connect(self.camera_outputData)  # 相机：开始标定
        self.btn_pinch_output.clicked.connect(self.pinch_outputData)  # 夹爪：开始标定
        self.btn_pinch_clear.clicked.connect(self.pinch_clearData)  # 夹爪：清空数据
        self.btn_cmr_clear.clicked.connect(self.camera_clearData)  # 相机：清空数据
        self.btn_pinch_del.clicked.connect(self.pinch_del)  # 夹爪：删除数据
        self.btn_cmr_del.clicked.connect(self.camera_del)  # 夹爪：删除数据
        self.btn_pinch_getQR.clicked.connect(self.pinch_getQR)  # 夹爪：计算二维码坐标
        # self.btn_pinch_export.clicked.connect(self.pinch_data_export)  # 夹爪：数据导出
        self.btn_cal1.clicked.connect(self.arm_pos_cal)  # 绑定计算机械臂坐标按钮
        self.btn_cal2.clicked.connect(self.arm_twice_cal)  # 绑定二次计算识别点按钮
        self.btn_cal3.clicked.connect(self.pinch_pos_cal)  # 绑定计算夹爪坐标按钮
        self.btn_get1.clicked.connect(self.get_data1)  # 视觉转机械臂坐标的按钮
        self.btn_get2.clicked.connect(self.get_twiceIdent_data)  # 二次识别按钮
        self.btn_get3.clicked.connect(self.get_data3)  # 计算夹爪坐标按钮
        self.btn_export_test_data.clicked.connect(self.export_test_data)  # 导出测试数据按钮

    def initial_param(self):
        self.arm_ip.setText(self.conf['param']['arm_ip'])
        self.arm_port.setText(self.conf['param']['arm_port'])
        self.cmr_ip.setText(self.conf['param']['camera_ip'])
        self.cmr_port.setText(self.conf['param']['camera_port'])
        self.RotationMatrix = np.array(eval(self.conf['param']['RotationMartix']))
        self.qrcodes = np.array(eval(self.conf['param']['qrcode']))
        self.edit_qrcode.setText(np.array2string(self.qrcodes, separator=', '))
        self.edit_rotationMartix.setText(np.array2string(self.RotationMatrix, separator=', '))
        self.edit_pinchOffsetMartix.setText(np.array2string(self.clamp_T, separator=', '))
        self.edit_cmrOffsetMartix.setText(np.array2string(self.camera_T, separator=', '))

    def software_init(self):
        current_directory = os.getcwd()
        folder_name = '夹爪标定数据'
        self.folder_path = os.path.join(current_directory, folder_name)
        if not os.path.exists(self.folder_path):
            os.mkdir(self.folder_path)

    # ======================================测试区域:获取数据按钮=============================================
    def get_data1(self):
        """视觉转机械臂的数据获取"""
        if not self._is_arm_connected or not self._is_camera_connected:
            QtWidgets.QMessageBox.warning(self, "警告", "请检查相机或机械臂是否连接", )
            return
        try:
            arm_position = self.scara.GetPostion()
            self.ident_point: np.ndarray = np.array([arm_position[key] for key in ['x', 'y', 'z']])
            self.camera_pos: np.ndarray = np.array(self.bytes_to_int(self.now_cmr_pos))
            self.ident_euler: dict = {key: arm_position[key] for key in ['u', 'v', 'w']}
        except AttributeError:
            QtWidgets.QMessageBox.warning(self, "警告", "数据异常，请检查相机状态或重试", )
            return
        self.edit_visualpos1.setText(np.array2string(self.camera_pos, separator=', '))
        self.edit_armpos1.setText(np.array2string(self.ident_point, separator=', '))
        self.edit_euler1.setText(str(self.ident_euler))
        self.edit_cmr_martix1.setText(self.edit_cmrOffsetMartix.text())

    def get_twiceIdent_data(self):
        """二次识别的数据获取"""
        if not self._is_arm_connected or not self._is_camera_connected:
            QtWidgets.QMessageBox.warning(self, "警告", "请检查相机或机械臂是否连接", )
            return
        try:
            arm_position = self.scara.GetPostion()
            self.ident_euler: dict = {key: arm_position[key] for key in ['u', 'v', 'w']}
        except AttributeError:
            QtWidgets.QMessageBox.warning(self, "警告", "数据异常，请检查相机状态或重试", )
            return
        try:
            self.edit_qrcode2.setText(self.edit_qrcode.text())
            self.edit_euler2.setText(str(self.ident_euler))
            self.edit_cmr_matrix2.setText(self.edit_cmrOffsetMartix.text())
        except Exception as e:
            print(e)

    def get_data3(self):
        """夹爪坐标数据获取按钮"""
        if not self._is_arm_connected or not self._is_camera_connected:
            QtWidgets.QMessageBox.warning(self, "警告", "请检查相机或机械臂是否连接", )
            return
        try:
            arm_position = self.scara.GetPostion()
            self.ident_euler: dict = {key: arm_position[key] for key in ['u', 'v', 'w']}
        except AttributeError:
            QtWidgets.QMessageBox.warning(self, "警告", "数据异常，请检查相机状态或重试", )
            return
        self.edit_euler3.setText(str(self.ident_euler))
        self.edit_qrcode3.setText(self.edit_qrcode.text())
        self.edit_pinch_martix.setText(self.edit_pinchOffsetMartix.text())

    # ======================================测试区域:获取数据按钮=============================================
    # ======================================测试区域:计算按钮================================================
    def arm_pos_cal(self):
        """视觉转机械臂坐标计算"""
        fields_to_check = [self.edit_armpos1, self.edit_visualpos1, self.edit_euler1, self.edit_cmr_martix1]
        if any(field.text() == "" for field in fields_to_check):
            QtWidgets.QMessageBox.warning(self, "警告", "请检查是否有数据为空", )
            return
        try:
            self.cal = calculation(np.array(eval(self.edit_rotationMartix.text())))
            camera_T = np.array(eval(self.edit_cmr_martix1.text()))
            self.real_point: np.ndarray = np.around(
                self.cal.GetRealPoint(
                    self.ident_point, self.camera_pos, self.ident_euler, camera_T),
                decimals=3)
            [func.setText(np.array2string(self.real_point, separator=', '))
             for func in [
                 self.edit_qrcode1, self.edit_qrcode2, self.edit_qrcode3]
             ]
        except:
            QtWidgets.QMessageBox.warning(self, "警告", "检查数据是否异常！", )

    def arm_twice_cal(self):
        """二次识别计算"""
        fields_to_check = [self.edit_qrcode2, self.edit_cmr_matrix2, self.edit_euler2, self.edit_distance]
        if any(field.text() == "" for field in fields_to_check):
            QtWidgets.QMessageBox.warning(self, "警告", "请检查是否有数据为空", )
            return
        try:
            qrcode: np.ndarray = np.array(eval(self.edit_qrcode2.text()))
            camera_T: np.ndarray = np.array(eval(self.edit_cmr_matrix2.text()))
            ident_euler: dict = eval(self.edit_euler2.text())
            target_distance: int = int(self.edit_distance.text())
            twice_ident_point: np.ndarray = self.cal.GetTwiceIdentPoint(
                qrcode, camera_T, ident_euler, target_distance
            )
            self.result2.setText(np.array2string(twice_ident_point, separator=', '))
        except TypeError:
            print("数据类型错误！请检查输入的参数是否正常")

    def pinch_pos_cal(self):
        """夹爪坐标计算"""
        fields_to_check = [self.edit_qrcode3, self.edit_pinch_martix, self.edit_euler3]
        if any(field.text() == "" for field in fields_to_check):
            QtWidgets.QMessageBox.warning(self, "警告", "请检查是否有数据为空", )
            return
        arm_position = self.scara.GetPostion()
        clamp_T = np.array(eval(self.edit_pinch_martix.text()))
        self.ident_euler: dict = {key: arm_position[key] for key in ['u', 'v', 'w']}
        try:
            clamp_point = np.around(
                self.cal.GetThePinchPoint(
                    self.real_point, self.ident_euler, clamp_T),
                decimals=3)
        except:
            QtWidgets.QMessageBox.warning(self, "警告", "检查数据是否异常", )
            return
        self.result3.setText(np.array2string(clamp_point, separator=', '))

    # ======================================测试区域:计算按钮================================================
    # ======================================初始化/中间处理函数==============================================
    def _outputWritten(self, text):
        cursor = self.textBrowser.textCursor()  # 定义光标
        cursor.movePosition(QtGui.QTextCursor.End)  # 把光标移动到末尾
        cursor.insertText(text)  # 在光标处插入文本
        self.textBrowser.setTextCursor(cursor)  # 放置光标
        self.textBrowser.ensureCursorVisible()  # 自动换行

    def arm_initial(self):
        if not self._is_arm_connected:
            QtWidgets.QMessageBox.warning(self, "警告", "您未连接机械臂", )
            return
        self.singleAct = SingleAction(self.scara)
        self.singleAct.start()

    def _isconnect(self, sock: socket.socket):
        try:
            sock.getpeername()
            return True
        except socket.error:
            return False

    def _camera_connect_thread(self):
        # 创建一个socket对象
        socket_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        host = self.cmr_ip.text()
        port = int(self.cmr_port.text())
        # 绑定地址
        try:
            socket_server.bind((host, port))
            # 设置监听
            socket_server.listen(5)
            # socket_server.accept()返回一个元组, 元素1为客户端的socket对象, 元素2为客户端的地址(ip地址，端口号)
            self.client_socket, address = socket_server.accept()
            self.label_cmr_state.setText("已连接")
            self._is_camera_connected = True
        except socket.error:
            print("连接失败")
            return
        while True:
            # 接收客户端的请求
            recvmsg = self.client_socket.recv(1024)
            if len(recvmsg) == 22:
                self.now_cmr_pos = recvmsg

    def camera_connect(self):
        thread = threading.Thread(target=self._camera_connect_thread)
        thread.start()

    def _str_split(self, obj, sec):
        return [obj[i:i + sec] for i in range(0, len(obj), sec)]

    def bytes_to_int(self, data: bytes):
        hex_data = data.hex()
        xyz_data = hex_data[len(hex_data) - 14:-2]
        data_list = self._str_split(xyz_data, 4)
        result_list = [(int(i, 16) - 65535) / 10 if int(i, 16) > 40000 else int(i, 16) / 10 for i in data_list]
        return result_list

    def _arm_connect_thread(self):

        arm_ip = self.arm_ip.text()
        arm_port = int(self.arm_port.text())
        self.scara = SCARAConnect(arm_ip, arm_port)
        self.scara.start()
        while True:
            if self._isconnect(self.scara.sck):
                self.label_arm_state.setText("已连接")
                self._is_arm_connected = True
                break
            else:
                self.label_arm_state.setText("未连接")
                continue

    def arm_connect(self):
        thread = threading.Thread(target=self._arm_connect_thread)
        thread.start()

    # ======================================初始化/中间处理函数==============================================
    # ======================================清空、删除按钮================================================

    def pinch_clearData(self):
        reply = QMessageBox.information(
            self, "提示", "是否清空数据？", QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
        if reply == QMessageBox.Yes:
            row_count = self.tw_pinch.rowCount()
            [self.tw_pinch.removeRow(0) for i in range(row_count)]

    def camera_clearData(self):
        reply = QMessageBox.information(
            self, "提示", "是否清空数据？", QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
        if reply == QMessageBox.Yes:
            row_count = self.tw_cmr.rowCount()
            [self.tw_cmr.removeRow(0) for i in range(row_count)]

    def pinch_del(self):
        selected_items = self.tw_pinch.selectedItems()
        if len(selected_items) == 0:  # 说明没有选中任何行
            return
        selected_items = [selected_items[i] for i in range(len(selected_items) - 1, -1, -7)]
        # 将选定行的行号降序排序，只有从索引大的行开始删除，才不会出现错误
        for items in selected_items:
            self.tw_pinch.removeRow(self.tw_pinch.indexFromItem(items).row())

    def camera_del(self):
        selected_items = self.tw_cmr.selectedItems()
        if len(selected_items) == 0:  # 说明没有选中任何行
            return
        selected_items = [selected_items[i] for i in range(len(selected_items) - 1, -1, -7)]
        for items in selected_items:
            self.tw_cmr.removeRow(self.tw_cmr.indexFromItem(items).row())

    # ======================================清空、删除按钮================================================
    # ======================================夹爪获取/计算按钮=============================================

    def pinch_getData(self):
        """夹爪采集数据按钮"""
        if not self._is_arm_connected:
            QtWidgets.QMessageBox.warning(self, "警告", "您未连接机械臂")
            return
        row_count = self.tw_pinch.rowCount()
        if row_count == 4:
            QtWidgets.QMessageBox.information(self, "提示", "数据已够四组", )
            return
        self.tw_pinch.setRowCount(row_count + 1)
        arm_position = self.scara.GetPostion()
        self.tw_pinch.setItem(row_count, 0, QTableWidgetItem(str(arm_position)))

    def pinch_outputData(self):
        """夹爪矩阵计算按钮"""
        offset_value = None
        if self.tw_pinch.rowCount() < 4:
            QtWidgets.QMessageBox.warning(self, "警告", "您未采集满4组数据！请继续采集", )
            return

        pinch_pos_list = [eval(self.tw_pinch.item(i, 0).text()) for i in range(4)]
        try:
            self.clamp_T, offset_value = self.cal.gripper_calibration(pinch_pos_list)
            # self.clamp_T = np.around(self.clamp_T, decimals=3)
        except np.linalg.linalg.LinAlgError:
            print("奇异矩阵！请检查坐标")
            return
        except AttributeError:
            print("请检查是否有空值！")
            return
        except Exception as e:
            print(e)

        try:
            self.pinch_output_martix.setText(f"{np.array2string(self.clamp_T, separator=', ')}")
            self.pinch_output_offset.setText(f"{str(offset_value)}")
            self.edit_pinchOffsetMartix.setText(f"{np.array2string(self.clamp_T, separator=', ')}")
        except Exception as e:
            print(e)
            return

    def pinch_getQR(self):
        if not self._is_arm_connected:
            QtWidgets.QMessageBox.warning(self, "警告", "您未连接机械臂", )
            return
        if not self.edit_pinchOffsetMartix.text():
            QtWidgets.QMessageBox.warning(self, "警告", "您还未标定夹爪偏移矩阵，请标定或手动输入", )
            return
        self.clamp_T: np.ndarray = np.array(eval(self.edit_pinchOffsetMartix.text()))
        self.qrcodes = self.cal.getBarcodePos(point=self.scara.GetPostion(), clamp_T=self.clamp_T)
        self.pinch_output_qrcode.setText(f"{np.array2string(self.qrcodes, separator=', ')}")
        self.edit_qrcode.setText(f"{np.array2string(self.qrcodes, separator=', ')}")

    # ======================================夹爪获取/计算按钮================================================
    # ======================================相机获取/计算按钮================================================

    def camera_getData(self):
        """相机数据采集按钮"""
        if not self._is_arm_connected or not self._is_camera_connected:
            QtWidgets.QMessageBox.warning(self, "警告", "请检查相机或机械臂是否连接", )
            return
        row_count = self.tw_cmr.rowCount()
        try:
            camera_pos = self.bytes_to_int(self.now_cmr_pos)
        except AttributeError:
            QtWidgets.QMessageBox.warning(self, "警告", "获取视觉坐标失败！请检查相机工作状态！", )
            return
        arm_position = self.scara.GetPostion()
        array_arm_position = np.around(np.array([arm_position[key] for key in ['x', 'y', 'z']]), decimals=3)
        self.tw_cmr.setRowCount(row_count + 1)
        self.tw_cmr.setItem(row_count, 1, QTableWidgetItem(np.array2string(array_arm_position, separator=', ')))
        self.tw_cmr.setItem(row_count, 0, QTableWidgetItem(str(camera_pos)))

    # region 相机：计算按钮
    def camera_outputData(self):
        """相机矩阵计算按钮"""
        row_count = self.tw_cmr.rowCount()
        self.qrcodes = np.array(eval(self.edit_qrcode.text()))

        if not self.qrcodes.size:
            QtWidgets.QMessageBox.warning(self, "警告", "您未输入或标定二维码坐标", )
            return
        if row_count < 4:
            QtWidgets.QMessageBox.warning(self, "警告", "您至少要采集四组数据！", )
            return
        reply = QMessageBox.information(
            self, "提示", "请检查当前的二维码坐标是否为最新", QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
        if reply == QMessageBox.No:
            return
        try:
            visual_pos_list = [np.array(eval(self.tw_cmr.item(i, 0).text())) for i in range(row_count)]  # 视觉列表
            arm_pos_list = [np.array(eval(self.tw_cmr.item(i, 1).text())) for i in range(row_count)]  # 机械臂坐标列表
            ident_euler: dict = {key: self.scara.GetPostion()[key] for key in ['u', 'v', 'w']}  # 姿态
            print(f"当前姿态:{ident_euler}")
        except Exception as e:
            print("传参坐标有误请反馈给开发者")
            print(e)
            return
        try:
            self.camera_T, self.RotationMatrix, offset_value = self.cal.camera_calibration(
                self.qrcodes, arm_pos_list, visual_pos_list, ident_euler)
            self.camera_T = np.around(self.camera_T, decimals=3)
            self.RotationMatrix = np.around(self.RotationMatrix, decimals=3)
            print(f"偏差矩阵:{offset_value}")
        except Exception as e:
            print(e)
            print("奇异矩阵")
            return

        try:
            self.edit_cmr_martix1.setText(f"{np.array2string(self.camera_T, separator=', ')}")
            self.edit_cmrOffsetMartix.setText(f"{np.array2string(self.camera_T, separator=', ')}")
            self.cmr_output1.setText(
                f"偏移矩阵:{np.array2string(self.camera_T, separator=', ')}")
            self.cmr_output_offsetValue.setText(f"偏差矩阵:{str(offset_value)}")
            self.cmr_output2.setText(np.array2string(self.RotationMatrix, separator=', '))
            self.edit_rotationMartix.setText(np.array2string(self.RotationMatrix, separator=', '))

        except Exception as e:
            print("this")
            print(e)

    # endregion

    # ======================================相机获取//计算按钮=========================================
    # ======================================导出按钮=================================================

    # region 导出按钮

    def pinch_data_export(self):
        column_names = ['1', '2', '3', '4']
        pinch_data = pd.DataFrame(columns=column_names)
        pinch_pos_list = [self.tw_pinch.item(i, 0).text() for i in range(4)]
        components = [self.pinch_output_martix, self.pinch_output_qrcode, self.pinch_output_offset]
        text_set = {
            "pinch_offset_matrix": None,
            "pinch_offset_value": None,
            "pinch_qrcodes": None,
        }
        for i in range(len(components)):
            text_set[i] = components[i].text() if components[i].text() != "" else None
        pinch_offset_matrix = self.pinch_output1.text()
        pinch_offset_value = eval(self.pinch_output_offset.text())
        pinch_qrcodes = self.pinch_output2.text()
        rows_to_update = {
            "机械臂坐标": {col: val for col, val in zip(pinch_data.columns, pinch_pos_list)},
            "偏移矩阵": [pinch_offset_matrix] + [None] * (len(pinch_data.columns) - 1),
            "偏差矩阵": {col: val for col, val in zip(pinch_data.columns, pinch_offset_value)},
            "二维码坐标": [pinch_qrcodes] + [None] * (len(pinch_data.columns) - 1)
        }

        for row_name, values in rows_to_update.items():
            if isinstance(values, dict):
                pinch_data.loc[row_name] = values
            else:
                pinch_data.loc[row_name] = values

    # endregion

    def export_test_data(self):
        """导出测试数据"""
        fields_to_check = [self.edit_euler1, self.edit_rotationMartix, self.edit_cmrOffsetMartix,
                           self.edit_pinchOffsetMartix, self.edit_qrcode, self.edit_qrcode1, self.edit_armpos1,
                           self.edit_visualpos1, self.result3]
        if any(field.text() == "" for field in fields_to_check):
            QtWidgets.QMessageBox.warning(self, "警告", "请检查是否有数据为空", )
            return
        now_time_ = datetime.now().strftime('%Y-%m-%d')
        if not os.path.exists(self.folder_path):
            df = pd.DataFrame()
            df.to_csv('夹爪标定数据/{now_time_}测试数据.csv')
        export_data = pd.read_csv(f'夹爪标定数据/{now_time_}测试数据.csv')
        try:
            euler = self.edit_euler1.text()
            rotation_T = self.edit_rotationMartix.text()
            camera_T = self.edit_cmrOffsetMartix.text()
            clamp_T = self.edit_pinchOffsetMartix.text()
            qrcode_probe = self.edit_qrcode.text()
            qrcode_camera = self.edit_qrcode1.text()
            arm_pos = self.edit_armpos1.text()
            visual_pos = self.edit_visualpos1.text()
            arm_position = self.scara.GetPostion()
            array_arm_position = str(np.around(np.array([arm_position[key] for key in ['x', 'y', 'z']]), decimals=3))
            final_clamp_pos = self.result3.text()
            now_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

            new_data = pd.DataFrame({
                '时间': now_time,
                '识别姿态': euler,
                '相机旋转矩阵': rotation_T,
                '相机偏移矩阵': camera_T,
                '夹爪偏移矩阵': clamp_T,
                '二维码坐标计算值（探针）': qrcode_probe,
                '二维码坐标计算值（相机）': qrcode_camera,
                '识别点坐标': arm_pos,
                '视觉坐标': visual_pos,
                '二维码坐标实际值（示教器)': array_arm_position,
                '夹爪坐标': final_clamp_pos,
            }, index=[0])

            result = pd.concat([export_data, new_data], ignore_index=True, axis=0)
            result = result.loc[:, ~result.columns.str.contains('Unnamed')]
            result = result.set_index('时间')
            result.to_csv(f'夹爪标定数据/{now_time_}测试数据.csv')
            print(result)
        except Exception as e:
            print(e)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    Caculator = MartixCaculator()
    Caculator.show()
    sys.exit(app.exec_())
