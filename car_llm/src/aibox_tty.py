# v1.0.0 By Danche
# v1.1.0
#    25/12/17 增加USB串口断线重连功能
import serial
import serial.tools.list_ports
import time
import sys
import os

class CrossPlatformSerial:
    def __init__(self, port=None, baudrate=115200):
        """初始化串口参数，自动选择默认端口（根据操作系统）"""
        self.baudrate = baudrate
        self.ser = None
        # 配置串口参数：8数据位，1停止位，无校验
        self.bytesize = serial.EIGHTBITS
        self.stopbits = serial.STOPBITS_ONE
        self.parity = serial.PARITY_NONE
        self.attempt_times=0
        
        # 根据操作系统设置默认端口
        if port is None:
            if os.name == 'nt':  # Windows系统
                self.port = 'COM8'  # Windows默认端口
            else:  # Linux系统（包括Jetson Nano）
                self.port = '/dev/ttyUSB0'  # Linux默认端口
        else:
            self.port = port
        
    def list_available_ports(self):
        """列出所有可用的串口端口"""
        ports = serial.tools.list_ports.comports()
        if not ports:
            print("未找到可用的串口设备")
            return []
        
        print("可用的串口设备:")
        port_list = []
        for port, desc, hwid in sorted(ports):
            print(f"  {port}: {desc} ({hwid})")
            port_list.append(port)
        return port_list
    
    def open(self):
        """打开串口"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=self.bytesize,
                stopbits=self.stopbits,
                parity=self.parity,
                timeout=1  # 读取超时时间
            )
            
            # 检查串口是否已打开
            if self.ser.is_open:
                print(f"串口 {self.port} 已打开，波特率: {self.baudrate}")
                return True
            return False
            
        except serial.SerialException as e:
            print(f"打开串口失败: {e}")
            return False
        except Exception as e:
            print(f"发生错误: {e}")
            return False
    
    def close(self):
        """关闭串口"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"串口 {self.port} 已关闭")
    
    def send_data(self, data):
        """发送数据"""
        if self.ser and self.ser.is_open:
            try:
                # 如果是字符串，转换为字节
                if isinstance(data, str):
                    data = data.encode('utf-8')
                self.ser.write(data)
                print(f"发送数据: {data}")
                return True
            except Exception as e:
                print(f"发送数据失败: {e}")
                return False
        else:
            print("串口未打开，无法发送数据")
            return False
    
    def read_data(self, size=32):
        """读取数据"""
        if self.ser and self.ser.is_open:
            try:
                data = self.ser.read(size)

                if data:
                    print(f"接收到数据: {data}")
                    # 尝试解码为字符串
                    try:
                        return data.decode('utf-8')
                    except UnicodeDecodeError:
                        return data  # 无法解码时返回原始字节
                return None
            except Exception as e:
                print(f"读取数据失败: {e},尝试重新连接设备：")
                return False
        else:
            print("串口未打开，无法读取数据")
            return False
    
    def reconnect(self):
        self.attempt_times+=1
        self.open()
        

if __name__ == "__main__":
    # 可以指定端口，如Windows: 'COM3', Linux: '/dev/ttyUSB0'
    # 不指定则使用默认端口
    serial_comm = CrossPlatformSerial(baudrate=115200)
    
    # 列出所有可用串口
    print("当前系统可用的串口端口:")
    serial_comm.list_available_ports()
    
    # 打开串口
    if serial_comm.open():
        try:
            # 示例：发送一条测试消息
            serial_comm.send_data("Hello from Serial!\r\n")
            
            # 循环读取数据
            print("开始监听串口数据，按Ctrl+C退出...")
            while True:
                data = serial_comm.read_data()
                if data:
                    # 收到数据后可以在这里添加处理逻辑
                    pass
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n用户中断程序")
        finally:
            # 关闭串口
            serial_comm.close()
    else:
        print("无法打开串口，程序退出")
        sys.exit(1)
    