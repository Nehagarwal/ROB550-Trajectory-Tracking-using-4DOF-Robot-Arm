import os
import sys, tty, termios
import dynamixel_functions as dynamixel

class DXL_BUS:
    def __init__(self,device,baud):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        self.port_num = dynamixel.portHandler(device)
        dynamixel.packetHandler()

        # Open port
        if dynamixel.openPort(self.port_num):
            print("Succeeded to open the port!")
        else:
            print("Failed to open the port!")
            print("Press any key to terminate...")
            self.getch()
            quit()

        # Set port baudrate
        if dynamixel.setBaudRate(self.port_num, baud):
            print("Succeeded to change the baudrate!")
        else:
            print("Failed to change the baudrate!")
            print("Press any key to terminate...")
            self.getch()
            quit()

    def port(self):
        return self.port_num

    def close(self):
        dynamixel.closePort(self.port_num)

    def getch(self):
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        return ch
