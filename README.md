# MLA-Tuner
## 简介
小环天线具有体积小，架设方便，抗干扰能力强等优点。但也存在非常明显的缺点，就是调谐特别困难。当天线与设备有一定的距离时，比如说天线安装在室外时，这个问题尤其突出。
鄙人经过一段时间的摸索和测试，成功的实现了使用手机App，通过低功耗蓝牙与安装在小环天线上的ESP32开发板连接驱动步进电机连接实现了小环天线的遥控操作。还通过附加的驻波检测电路实现了小环天线的全自动调谐。现分享遥控调谐的设计方案和程序代码， 该设计还很初级，一定有不足和可以改进的地方，恳请大家批评指正多多指教。

## 通信模式
为了方便实现手机与控制盒的双向通信，同时又尽最大可能减少对设备的干扰，我们选用蓝牙低功耗通信模式(Bluetooth Low Energy),简称为BLE。在众多的支持BLE的MCU中,国产的ESP32开发板具有价廉物美、文档资料丰富、函数库完备，是我们的首选。

## 调谐是关键
大家知道小环天线(Small Loop Antenna)又称为磁环天线(Magnetic Loop Antenna)，其中一个关键的部件是一只高耐压的可变电容器，改变电容容量用于调谐天线的谐振频率，从而实现接收时具有最强的信号，发射时有最小的驻波及最高的效率。所以如何方便精密地调谐可变电容是使用好小环天线的关键。经过对多种步进电机的测试和比较，发现最常见的28BYJ4 64:1步进电机和ULN2003步进电机驱动板就能非常好的满足慢速旋转空气可变电容甚至真空可变电容器的工作。

## 开发环境和整体架构
ESP32具有非常丰富的生开发环境ESP-IDF(https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html)，但为了能更好的移植和方便大家在现有的MCU上体验，我还是选用了Arduino。系统分为BLE UART服务、步进电机驱动、参数永久保存、命令解析执行等部分。BLE有非常丰富的功能，其中BLE UART非常契合我们的使用环境。调试信息通过ESP32的串口输出，ESP32板载的LED作为步进电机工作指示。预留了BOOT按键的输入功能以备将来使用。

## 手机App采用BLE UART与ESP32通信
BLE功能已经是现代手机标准配置的功能，只要不是太古老的手机都可以通过BLE与ESP32通信。AdaFruit为了方便用户测试BLE功能特意推出了Bluefruit Connect BLE免费工具，有iOS和Android版本。我们可以直接使用Bluefruit Connect来作为我们的遥控器App。该App上的控制面板功能提供了，4个方向键和4个数字键。我们把Up/Down键作为快速调谐按钮，长按可以连续快进退。Left/Right按键为微调按键，长按为连续进或退微调。 数字1-4我设计为预存位置调出，长按为设置预存位置。Bulefruit Connect还具有UART的直接通信模式，我设计了2个命令：reset命令用于将当前的位置设置为起始位置0，speed:为临时修改步进电机速度。比如临时将默认的步进电机速度(10RPM)调整为15RPM可以发出speed:15。当然也可以将速度减低以便更精确地微调。 Bluefruit Connect可以在应用商店找到，或可以直接下载相关的apk在Android设备上直接安装。

## ESP32与ULN2003的连接
ESP32与ULN2003通过4+2条杜邦线连接，为了方便连接采用了4个连续的GPIO接脚。另外为地线和电源线，当使用非5V供电的步进电机时，需要另外提供ESP32的供电。连接方式见附图
![](https://raw.githubusercontent.com/justsoft/MLA-Tuner/main/ESP-32-ULN2003-Wiring-Overview.png)
![](https://raw.githubusercontent.com/justsoft/MLA-Tuner/main/ESP-32-ULN2003-Wiring.png)
## 保存最后的位置
ESP32具有512个字节的EEPROM可以用来永久保存数据。我们用其中20个字节用来保存当前位置和4个记忆位置，即使断电这些数据也不会丢失。可以方便的调节到需要的位置(由于步进电机和齿轮间隙回调的位置通常还需要微调)

## 代码和库
可采用最新版本的Arduino，我采用的是目前最新的1.8.16。按ESP32的Arduino开发文档添加正确的ESP32板信息。并添加相应步进电机库和EEPROM库就可以开始体验了。
完整的代码请见: https://github.com/justsoft/MLA-Tuner

## 参考资料:

1) https://github.com/Godefridus/ESP32/blob/main/ESP32_UART.ino
2) https://randomnerdtutorials.com/esp32-stepper-motor-28byj-48-uln2003/
3) ![ESP32-Devkit接脚](https://raw.githubusercontent.com/justsoft/MLA-Tuner/main/ESP32-38%20PIN-DEVBOARD.png) 

## TODO:
1) 减少耗电。目前代码没有考虑任何休眠状态，只是把步进电机的停止状态时电流全部置低电平(可能会轻微影响定位精度)，如何能进入休眠状态并用ESP32上的BOOT按钮唤醒，可以更好的节省能耗
2) 编写专用的App

## 欢迎交流分享改进
欢迎加入QQ群: 870801104

## 附图
1) 5米周长19毫米直径的铝管，上乘质量的小空气可变电容可以支持最大20W功率发射的小环天线![](https://raw.githubusercontent.com/justsoft/MLA-Tuner/main/5m-avc.jpg)
2) 4米周长19毫米直径的铜管，真空可变电容和3D打印的安装支架及减速齿轮![](https://raw.githubusercontent.com/justsoft/MLA-Tuner/main/4m-vvc.jpg)
3) 窗外工作中的4米周长小环，实现与5KL远程小功率通联![](https://raw.githubusercontent.com/justsoft/MLA-Tuner/main/4m-vvc-on-air.jpg)
