# ros2_d2xx
> �N FTDI USB�PAltera �s����USB-blaster �ϥ� ROS2 �sĶ�ð�������USB�O�_���Q���q

## Getting Started 
* �ݭn�w���w�˾��ROS2 Humble ���� [ros2-humble install Windows(source)](https://docs.ros.org/en/humble/Installation/Alternatives/Windows-Development-Setup.html)
* �w���N�JAltera DE0 Broad  : DE0_top.sof ���b /DE0/output_files ��
### Environment
* Windows10
* Altera DE0 Board
* [FTDI D2XX Driver](https://ftdichip.com/drivers/d2xx-drivers/)
### Prerequisites 
* Quastus II 13.0 sp 1(64-bit)
* Vistual Studio 2022
*  ROS2 humble
* FTDI D2XX Library

### Create Project with VS2022/cmd.exe
```bash
call D:\humble\install\local_setup.bat                 # set ros2 env.
mkdir \path\ros2_ws\src                                # create  ros2 workspace Folder 
cd ros2_ws\src                                         # into folder
ros2 pkg create --build-type ament_cmake ros2_d2xx     # create new package
```
### Build Project with VS2022/cmd.exe

```bash
cd ros2_ws/src
git clone https://github.com/ericchuanggit/ros2_d2xx    # replace created package
call D:\humble\install\local_setup.bat                  # set whole ros2 env.
call install\local_setup.bat                            # set created package env.
colcon build --packages-select ros2_d2xx --merge-install # build ros2 package with colcon build
```
* Colcon Build<br>
![Success](result/colcon_build.jpg)

## Burn the DE0_top.sof file into the DE0 board
```bash
D:\dev\ros2_ws\src\ros2_d2xx\DE0\output_files>quartus_pgm -c usb-blaster -m jtag -o "p;DE0_top.sof@1"
```
* Burn in FPGA
![Burnin](result/burnin.png)
## Usage ros2_d2xx package 
```bash
ros2 run ros2_d2xx pub_sub
```

## Result
* Success Connection<br>
![Success](result/success.png)

* Failure Connection<br>
![Failure](result/Failure.png)


## Authors 

* **Eric Chuang** 



