import urx
from time import sleep
import math3d as m3d
import math


base_transform = m3d.Transform((0, 0, -0.665, 0, 0, 3*math.pi/4))
rob = urx.Robot("192.168.56.10")

rob.set_csys(base_transform)
rob.set_tcp((0, 0, 0.17, 0, 0, 0))
rob.set_payload(2, (0, 0, 0.1))
sleep(0.2)  #leave some time to robot to process the setup commands
# print("Current tool pose is: ",  rob.getl())
# rob.movel_tool((0, 0, 0, 0, 0, math.pi), acc=0.05, vel=0.05)
# rob.movep(rob.getl())
# rob.movej((1, 2, 3, 4, 5, 6), a, v)
# rob.movel((x, y, z, rx, ry, rz), a, v)
# print("到达位置1")
# rob.movel((0, 0, 0, 0, 0, math.pi/8), relative=True)  # move relative to current pose
# print("Current tool pose is: ",  rob.getl())

# rob.translate((-0.1, 0, 0))  #move tool and keep orientation
# rob.stopj(a)


# robot.movel(x, y, z, rx, ry, rz), wait=False)
# while True :
#     sleep(0.1)  #sleep first since the robot may not have processed the command yet
#     if rob.is_program_running():
#         break

# robot.movel(x, y, z, rx, ry, rz), wait=False)
# while rob.getForce() < 50:
#     sleep(0.01)
#     if not rob.is_program_running():
#         break
# rob.stopl()

# try:
#     rob.movel((0,0,0.1,0,0,0), relative=True)
# except RobotError, ex:
#     print("Robot could not execute move (emergency stop for example), do something", ex)
if __name__ == '__main__':
    f = open("poselog.txt", "w")
    while 1:
        mess = input("输入s保存当前位置，q退出：")
        if mess == "q":
            break
        elif mess == "s":
            pose = rob.getl()
            f.write(str(pose))
            f.write('\n')
    f.close()