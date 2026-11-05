# STM32 Servo Gimbal / 视觉舵机云台

基于 STM32 + FreeRTOS 的二维舵机云台，配合上位机（PyQt5 + YOLOv8 人脸检测）实现视觉追踪。

## 项目结构

```
Servo_Gimbel/
├── main.c                          # STM32 主程序（FreeRTOS + SSD1306 OLED + 串口指令解析）
└── Vision_Servo_Python/            # 上位机视觉程序
    ├── test.py                     # 主程序：摄像头采集 → YOLOv8 人脸检测 → 串口发送角度
    ├── model/
    │   └── yolov8n-face.pt         # 人脸检测模型权重
    └── ui/
        ├── Vision_Mian_UI.ui       # Qt Designer 界面文件
        └── Ui_Vision_Mian_UI.py    # pyuic5 生成的界面代码
```

## 硬件

- STM32 开发板（Cortex-M，运行 FreeRTOS）
- 两路舵机（云台 X / Y 轴）
- SSD1306 OLED（I2C，用于显示状态）
- USB 摄像头 / 串口连接上位机

## 上位机运行

```bash
cd Vision_Servo_Python
pip install PyQt5 opencv-python ultralytics pyserial
python test.py
```

界面中可以扫描并选择摄像头与串口；开启检测后，程序通过 YOLOv8 定位画面中人脸的中心点，
计算与画面中心的偏差后转换为舵机角度，经串口下发给 STM32。

## 下位机

`main.c` 为 STM32CubeMX 生成的工程主文件，主要逻辑在 `USER CODE` 区域内：

- FreeRTOS 任务划分：串口接收 / 舵机控制 / OLED 刷新
- 串口接收队列解析上位机下发的角度指令
- 通过 PWM 输出驱动两路舵机

将 `main.c` 放回你的 CubeMX 工程对应路径，编译烧录即可。

## 备注

- `model/yolov8n-face.pt` 为人脸检测权重文件（约 6 MB）。
- 串口波特率需与上位机保持一致。
