# Hardware

## Configuration

1. Intel NUC

    整个机器人的核心部分，在不必要时请勿卸下上面的USB和Ethernet连线。

2. Kinetic

    Microsoft Kinetic 2深度摄像头。其Source和Data是分开两路传输的。

3. Joystick

    Logitech手柄，可以用于控制机器人运动，目前接收器未接到Intel NUC上。

## Connection

Positive - Red
Negative - Black

注意导线上面对应的各条线功能提示。

1. 12V

    对应与12V接线链接，这根线比较短。(用于Gripper供电？)

2. 48V

    对应与48V接线连接，这根线比较长。(用于左侧UR5供电？)

3. 右电源

    右端UR5电源。

4. 右急停

    控制右端UR5的紧急停止，使用后方红色急停按钮实现。

5. Kinetic

    Source直接连接Robot电源，Data线为Blue USB接口，连接到Intel NUC上。该Data线与另外一个小的单孔线组合形成Box转接器, Kinetic摄像头可以在外部连接该Box。

6. Intel NUC

    主要部分为黑色单头电源线，Ethernet接口, 各USB接口，及HDMI接口。
    
    主机Ethernet连线连接至后方面面板，在外部使用黄色接口和黄色Ethernet连线连接至路由器。

    主机上HDMI接口比正常显示屏HDMI接口小一点点，一端使用较小的HDMI接口接主机，另一端使用正常HDMI接口接显示器可以显示图形界面。如果需要购买转接线，注意带一根正常HDMI线比较大小。HDMI接口应该有三种尺寸，选中间那一种比较合适。

    主机上白色USB分线器上两根USB线分别接到后面板的两个USB扩展接口，目前我们可以在后面板扩展口再接一个分线器用来插鼠标和键盘。

7. UR5s

    电源线与数据线在UR5内部，末端通过白色粗导线与灰色L形接口连接，左侧UR5连接口需要接到后面板对应接口。右侧UR5连接口已经连接至云台后方，无需勿动。

8. Robotiq 3 Finger Grippers

    两端夹爪电源都已经和云台内部电路连接，外面可能有通过焊接而暴露的导线，注意拆卸过程不要扯断。如果焊接处已经完全暴露在外面，注意用胶带保护，防止正(Red)负(Black)电线接触导致短路。

    电源线和数据线通过捆在一起的黑色和绿色粗导线从外部连接到Grippers上。

9. Laser LADAR

    雷达通过橙色电源与数据线与白色Ethenet连线工作。如需卸下时，其接口在Robot前方雷达底部，Ethenet接最左边接口，橙色电源与数据线接最右边接口，主要用接口处螺母固定。如果需要取出橙色与白色线，请先将雷达固定板四周螺钉卸下。

    雷达的白色网线螺母无法旋紧，因而连接不是很紧密，需要考虑更换。


## Network

1. Intel NUC
    
    IP = 192.168.1.30

    若重装系统后需要对网络重新进行配置，首先在连上路由器后第一次启动系统时使用以下命令

        ifconfig
    
    查看Ethenet端口名称, Ubuntu 14.04为eth[X], Ubuntu 16.04及18.04为enp[X]s[XX], 使用如下命令进行网络配置

        sudo gedit /etc/network/interfaces

    添加以下内容

        auto eth[X](enp[X]s[XX])
        iface eth[X](enp[X]s[XX]) inet static
        address 192.168.1.30
        netmask 255.255.255.0
        gateway 192.168.1.1
        dns-nameservers 166.111.8.28

    其中166.111.8.28为Tsinghua内网DNS端口，也可以用8.8.8.8。之后重启系统

        sudo reboot
    
    或重启网络

        sudo /etc/init.d/networking restart

    检查连接是否可用

        ping 166.111.8.28
        ping www.baidu.com

    若166.111.8.28可以ping通而www.baidu.com不能ping通说明需要登录校园网之后才能使用。

2. Left UR5

    IP = 192.168.1.50 
    Control Port = 29999

3. Right UR5

    IP = 192.168.1.60 
    Control Port = 29999

4. Left Robotiq 3 Finger Gripper

    IP = 192.168.2.11

5. Right Robotiq 3 Finger Gripper

    IP = 192.168.3.11

6. Laser LADAR

    IP = 192.168.1.40

## Power

已知的电压有两种12V和48V, 具体作用待定。

## Hardware to Software Mapping

### Descriptions

1. ttyACM0

    该接口是轮子电机接口，通过最底层电路连接（连接方式未知）形成一个USB接口，直接插在Intel NUC上。其接口为黑色且比较短，默认插在白色USB分线器下方。硬件调试结束后注意检查该接口是否正确连接：

    Open a terminal in Ubuntu system. And type in the following command

        ls /dev/ttyACM0

    如果Terminal中显示该设备则该接口正常，反之需要重新检查连线。

2. ttyUSB0

    IMU接口，用于计算底盘Odometry和Speed。
    
    该接口用于bsm，具体功能未知，需要和公司沟通。

3. ttyUSB1

    该接口是Robot车厢后方显示屏的接口，通过USB接口插在Intel NUC上。其接口为黑色且比较长，默认插在Kinetic Blue Data线的旁边。

### Rules

以上接口为硬件在系统device目录下的表示，Linux系统在硬件与对应设备的驱动程序之间还有对应的hardware rules。
由于硬件默认使用权限为root而不是普通用户，该Rule可以通过硬件出厂信息match到具体的硬件设备，实现开放权限给普通
用户使用，定义带有意义的symlink等功能。若重装系统后在使用driver前请先配置hardware rules。

注意使用google搜索udev rules的使用方法。

以下为参考blog: https://clearpathrobotics.com/blog/2015/01/udev/


1. rules存放目录

        cd /lib/udev/rules.d

    该目录下存放有系统默认的hardware rule，可以控制键盘、鼠标等基本设备的使用，注意文件的命名方式和内容格式，请勿直接修改这个目录中的rules或添加新的rules。

        cd /etc/udev/rules.d

    该目录存放custom rules，新增的rules会覆盖掉相同设备的默认rules，请将新的rules存放在该目录中。

2. 查看hardware information

    使用如下命令能够查看各设备出厂信息

        udevadm info -a -p $(udevadm info -q path -n <devpath>)

    其中`<devpath>`为/dev目录下对应的各个设备名称。一下为ttyACM0, ttyUSB0, ttyUSB1对应的部分信息

    - ttyACM0 - motor controller

        ATTRS{product}=="Motor Controller"
        ATTRS{idProduct}=="5740"
        ATTRS{idVendor}=="20d2"

    - ttyUSB0 - imu controller
    
        ATTRS{product}=="USB-Serial Controller"
        ATTRS{idProduct}=="2303"
        ATTRS{idVendor}=="067b"
        ATTRS{devnum}=="2"

    - ttyUSB1 - monitor controller

        ATTRS{product}=="USB-Serial Controller"
        ATTRS{idProduct}=="2303"
        ATTRS{idVendor}=="067b"
        ATTRS{devnum}=="5"

3. 拷贝bulldog_rules中的内容至系统目录

        cd bulldog_rules
        sudo cp * /etc/udev/rules.d

4. 重启udev

        sudo udevadm control --reload-rules
        sudo service udev restart
        sudo udevadm trigger

    之后可以正常启动底盘driver。

