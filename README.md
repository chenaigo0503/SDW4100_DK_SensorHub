# Apollo3B_Hub

#### 介绍
Apollo3 blue作为目标芯片
使用Ambiq SDK 2.5.1

#### 软件架构
软件架构说明

#### 生成升级bin文件

1. 电脑安装git bash
2. 使用git bash进入工程目录，执行项目根目录下creat_ota_binary.bat
3. 在工程路径下会生成apollo3b_hub\out\apollo.bin
4. 将apollo.bin文件push到设备/vendor/firmware/并重启设备
    - adb push apollo.bin /vendor/firmware
    - adb reboot