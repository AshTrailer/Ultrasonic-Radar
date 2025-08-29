# Ultrasonic-Radar(超声雷达)

这是一个基于 Arduino 开发的近距离超声波雷达传感器项目。
项目的核心思路是将 HC-SR04 超声波测距模块安装在步进电机 上，通过控制电机进行平面旋转扫描，从而实现对周围环境的 360° 近距离测量。
Arduino 控制超声波传感器周期性发射/接收脉冲，同时记录当前位置的角度和对应的测量距离。数据会通过串口实时输出，配合 MATLAB 脚本可以生成动态的全周雷达扫描图，实现类似简易雷达的效果。

This is an Arduino-based short-range ultrasonic radar sensor project.
The system uses an HC-SR04 ultrasonic distance sensor mounted on a stepper motor, which rotates in the horizontal plane to perform 360° environmental scanning.

The Arduino controls the ultrasonic sensor to periodically emit/receive pulses while tracking the stepper motor’s angular position. At each angle, the measured distance is recorded. The data is continuously transmitted over the serial port, and a MATLAB script is provided to generate a real-time radar-style polar plot for full-circle visualization.
