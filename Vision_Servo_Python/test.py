from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox
from ui.Ui_Vision_Mian_UI import Ui_MainWindow
import cv2
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap

from ultralytics import YOLO

import serial
import serial.tools.list_ports
import struct

class MyWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.cap = None
        self.setWindowTitle("My PyQt5 Window")
        self.setGeometry(100, 100, 600, 400)
        self.timer = QTimer()
        self.timer.timeout.connect(self.show_frame)

        self.scan_cameras()  # 初始化时扫描摄像头
        self.scan_serial_ports()  # 初始化时扫描串口

        # 绑定信号
        self.Refresh.clicked.connect(self.scan_cameras)
        self.Show_connect_btn.clicked.connect(self.toggle_connect)
        # 绑定串口信号
        self.Port_refresh_btn.clicked.connect(self.scan_serial_ports)
        self.Connect_port_bnt.clicked.connect(self.toggle_serial_connection)

        # 标记
        # 标记连接
        self.is_connect = False
        # 标记串口连接
        self.is_serial_connect = False

        # 视频旋转
        self.rotate_angle = 0
        self.Show_rotate_btn.clicked.connect(self.rotate_image)

        # 初始化yolov8n-face模型
        self.model = YOLO("model/yolov8n-face.pt")

        #人脸中心
        self.x1 = 0
        self.y1 = 0
        self.x2 = 0
        self.y2 = 0
        self.w = 0
        self.h = 0



    def show_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        frame = self.img_rotate(frame)
        frame = self.face_position(frame)

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # frame = cv2.resize(frame, (800, 600))
        self.w = frame.shape[1]
        self.h = frame.shape[0]
        qimg = QImage(frame, frame.shape[1], frame.shape[0], QImage.Format_RGB888)
        pix = QPixmap.fromImage(qimg.scaled(self.show_img_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        self.show_img_label.setPixmap(QPixmap.fromImage(qimg))

        dx, dy = self.put_face_center(self.x1, self.y1, self.x2, self.y2, self.w, self.h)
        # 把坐标写入到文本框中,保留2位小数
        self.dx_dy_text.setText(f"DX: {dx:.2f}, DY: {dy:.2f}")
        self.send_data()



    # 扫描可用摄像头
    def scan_cameras(self):
        self.Combo_cam.clear()
        cam_list = []
        for i in range(3):
            cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
            if cap.isOpened():
                cam_list.append(i)
                cap.release()
        for cam in cam_list:
            self.Combo_cam.addItem(f"Camera {cam}", cam)

    # 旋转视频
    def img_rotate(self, frame):
        # 旋转视频
        if self.rotate_angle == 90:
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        elif self.rotate_angle == 180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif self.rotate_angle == 270:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        return frame

    # 人脸位置
    def face_position(self, frame):
        results = self.model.track(frame, tracker="bytetrack.yaml", persist=True, conf=0.4, iou=0.5)
        boxes = results[0].boxes

        if boxes.id is not None:
                ids = boxes.id.cpu().numpy()
                xyxy = boxes.xyxy.cpu().numpy().astype(int)
                for tid, box in zip(ids, xyxy):
                    self.x1, self.y1, self.x2, self.y2 = box
                    # 最后一个参数 = 线宽，1就是很细；2是默认粗度
                    cv2.rectangle(frame, (self.x1,self.y1), (self.x2,self.y2), (0,255,0), thickness=1)
                    cv2.putText(frame, f"ID:{int(tid)}", (self.x1, self.y1-5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0), 1)       
        return frame

    # 切换摄像头时,如果连接则先断开
    def change_camera(self):
        if self.is_connect:
            self.disconnect_camera()
    def toggle_connect(self):
        if self.is_connect:
            self.disconnect_camera()
        else:
            self.connect_camera()

    def connect_camera(self):
        cid = self.Combo_cam.currentData()
        if cid is None:
            QMessageBox.warning(self, "Warning", "Please select a camera first.")
            return

        self.cap = cv2.VideoCapture(cid, cv2.CAP_DSHOW)

        self.is_connect = True
        self.timer.start(30)
        self.Show_connect_btn.setText("断开连接")

        
    def disconnect_camera(self):
        self.timer.stop()
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
        self.is_connect = False
        self.Show_connect_btn.setText("连接")
        self.show_img_label.clear()

    def rotate_image(self):
        self.rotate_angle += 90
        self.rotate_angle %= 360

    def put_face_center(self,x1, y1, x2, y2, w, h):
        norm_x = ((x1 + x2) / 2 - w/2) / (w/2)
        norm_y = ((y1 + y2) / 2 - h/2) / (h/2)
        return norm_x, norm_y


    #- norm_x ∈ [-1,1]：-1 最左，+1 最右
    #- norm_y ∈ [-1,1]：-1 最上，+1 最下


    # 扫描串口
    def scan_serial_ports(self):
        self.Combo_port.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.Combo_port.addItem(port.device)

    # 连接/断开串口
    def toggle_serial_connection(self):
        if self.is_serial_connect:
            self.disconnect_serial()
        else:
            self.connect_serial()

    def connect_serial(self):
        port = self.Combo_port.currentText()

        self.serial_port = serial.Serial(port, baudrate=115200, timeout=1)
        self.is_serial_connect = True
        self.Connect_port_bnt.setText("断开串口")

    def disconnect_serial(self):
        if self.serial_port.is_open:
            self.serial_port.close()
        self.is_serial_connect = False
        self.Connect_port_bnt.setText("连接串口")
    # 刷新串口列表
    def refresh_serial_ports(self):
        self.scan_serial_ports()

    def send_data(self):
        x,y = self.put_face_center(self.x1, self.y1, self.x2, self.y2, self.w, self.h)
        buf = struct.pack("<BffB", 0xAA,x,y,0xBB)
        if self.is_serial_connect:
            self.serial_port.write(buf)
        print(buf)

    




if __name__ == "__main__":
    app = QApplication([])
    window = MyWindow()
    window.show()
    app.exec_()
