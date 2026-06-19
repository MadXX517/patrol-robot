启动后命令行运行：

sudo rpi-eeprom-config --edit

添加：

PSU_MAX_CURRENT=5000

保存重启

USB的电流限制从600mA放宽到1.6A，USB-C口3A电流限制取消。