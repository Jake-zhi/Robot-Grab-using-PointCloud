import socket

def socket_init():
    s = socket.socket()
    # host_ip = "192.168.31.2"
    host_ip = 'localhost'
    port = 12345
    print("连接主机: %s:%s" % (host_ip, port))
    Arr = (host_ip, port)
    try:
        s.connect(Arr)
        print('Connection OK!')
    except ConnectionRefusedError:
        print('Connection failed, please check.')
        raise ConnectionRefusedError
    s.send('hello'.encode())
    return s

# while 1:
#     send_mes = input("输入发送的数据：")
#     s.send(send_mes.encode())

# 位置字符串（七个数字，空格隔开）
def send_pose(s, pose):
    s.send(pose.encode())
    
if __name__ == "__main__":
    # s = socket_init()
    f = open("poselog.txt", "w")
    while 1:
        mess = input("输入位置：")
        if mess == 'q' or mess == '': break
        f.write(mess)
        f.write('\n')
        # send_pose(s, mess)
    f.close()
