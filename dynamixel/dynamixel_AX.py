import dynamixel_functions as dxl

# Control table address for Dynamixel AX
#EEPROM_ADDRESSES
ADDR_MODEL_NUMBER        = 0
ADDR_FIRMWARE_VERSION    = 2
ADDR_ID                  = 3
ADDR_BAUD_RATE           = 4
ADDR_RETURN_DELAY        = 5
ADDR_CW_LIMIT            = 6
ADDR_CCW_LIMIT           = 8
ADDR_TEMP_LIMIT          = 11
ADDR_LOW_VOLTAGE_LIMIT   = 12
ADDR_HIGH_VOLTAGE_LIMIT  = 13
ADDR_MAX_TORQUE          = 14
ADDR_STATUS_RTN          = 16
ADDR_ALARM_LED           = 17
ADDR_ALARM_SHUTDOWN      = 18

#RAM_ADDRESSES
ADDR_TORQUE_ENABLE       = 24
ADDR_LED                 = 25
ADDR_CW_COMPLIANCE_M     = 26
ADDR_CCW_COMPLIANCE_M    = 27
ADDR_CW_COMPLIANCE_M     = 28
ADDR_CCW_COMPLIANCE_S    = 29
ADDR_GOAL_POSITION       = 30
ADDR_MOVING_SPEED        = 32
ADDR_TORQUE_LIMIT        = 34
ADDR_PRESENT_POSITION    = 36
ADDR_PRESENT_SPEED       = 38
ADDR_PRESENT_LOAD        = 40
ADDR_PRESENT_VOLTAGE     = 42
ADDR_PRESENT_TEMP        = 43
ADDR_INSTR_REG           = 44
ADDR_MOVING              = 46
ADDR_LOCK                = 47
ADDR_PUNCH               = 48

PROTOCOL                 = 1
VERBOSE                  = 0
RETRIES                  = 10

def print_v(arg):
    if(VERBOSE):
        print(arg)


class DXL_AX:
    def __init__(self, port, id):
        self.type = "AX"
        self.port = port
        self.id = id
        self.mode = self.get_mode()
        self.max_speed = 11.89 #rad/s

    def set_mode(self,mode, num_tries = 5):
        # 1 for wheel, 2 for joint
        if((mode != 1) & (mode != 2)):
            print_v('Error: Mode must be 1 or 2')
            return -1
        retry = 0
        while(retry < num_tries): 
            if(mode == 1):
                dxl.write1ByteTxRx(self.port, PROTOCOL, self.id, ADDR_CW_LIMIT, 0)
                dxl.write1ByteTxRx(self.port, PROTOCOL, self.id, ADDR_CCW_LIMIT, 0)
            if(mode == 2):
                dxl.write1ByteTxRx(self.port, PROTOCOL, self.id, ADDR_CW_LIMIT, 0)
                dxl.write1ByteTxRx(self.port, PROTOCOL, self.id, ADDR_CCW_LIMIT, 4095)
            dcomm_result = dxl.getLastTxRxResult(self.port, PROTOCOL)
            derror = dxl.getLastRxPacketError(self.port, PROTOCOL)
            if dcomm_result != 0:
                print_v(dxl.getTxRxResult(PROTOCOL, dcomm_result))
                retry +=1
            elif derror != 0:
                print_v(dxl.getRxPacketError(PROTOCOL, derror))
                retry +=1
            else:
                self.mode = mode
                return 0
        raise Exception("set_mode failed after {} tries".format(num_tries))
        

    def get_mode(self, num_tries = RETRIES):
        mode=0
        retry = 0
        while(retry < num_tries):        
            cw_lim = dxl.read1ByteTxRx(self.port, PROTOCOL, self.id, ADDR_CW_LIMIT)
            dcomm_result = dxl.getLastTxRxResult(self.port, PROTOCOL)
            derror = dxl.getLastRxPacketError(self.port, PROTOCOL)
            ccw_lim = dxl.read1ByteTxRx(self.port, PROTOCOL, self.id, ADDR_CCW_LIMIT)
            dcomm_result += dxl.getLastTxRxResult(self.port, PROTOCOL)
            derror = dxl.getLastRxPacketError(self.port, PROTOCOL)
            if dcomm_result != 0:
                print_v(dxl.getTxRxResult(PROTOCOL, dcomm_result))
                retry +=1
            elif derror != 0:
                print_v(dxl.getRxPacketError(PROTOCOL, derror))
                retry +=1
            else:
                if((cw_lim == 0)&(ccw_lim == 0)):
                    mode = 1
                else:
                    mode = 2
                return mode
        raise Exception("get_mode failed after {} tries".format(num_tries))

    def set_compliance(self,compliance_margin,compliance_slope, num_tries = RETRIES):
        retry = 0
        while(retry < num_tries):
            dxl.write1ByteTxRx(self.port, PROTOCOL, self.id, ADDR_CW_COMPLIANCE_M,compliance_margin)
            dcomm_result = dxl.getLastTxRxResult(self.port, PROTOCOL)
            derror = dxl.getLastRxPacketError(self.port, PROTOCOL)
            dxl.write1ByteTxRx(self.port, PROTOCOL, self.id, ADDR_CCW_COMPLIANCE_M,compliance_margin)
            dcomm_result += dxl.getLastTxRxResult(self.port, PROTOCOL)
            derror += dxl.getLastRxPacketError(self.port, PROTOCOL)
            dxl.write1ByteTxRx(self.port, PROTOCOL, self.id, ADDR_CW_COMPLIANCE_S,compliance_slope)
            dcomm_result += dxl.getLastTxRxResult(self.port, PROTOCOL)
            derror += dxl.getLastRxPacketError(self.port, PROTOCOL)
            dxl.write1ByteTxRx(self.port, PROTOCOL, self.id, ADDR_CCW_COMPLIANCE_S,compliance_slope)
            dcomm_result += dxl.getLastTxRxResult(self.port, PROTOCOL)
            derror += dxl.getLastRxPacketError(self.port, PROTOCOL)
            if dcomm_result != 0:
                print_v(dxl.getTxRxResult(PROTOCOL, dcomm_result))
                retry +=1
            elif derror != 0:
                print_v(dxl.getRxPacketError(PROTOCOL, derror))
                retry +=1
            else:
                return 0
        raise Exception("set_gains failed after {} tries".format(num_tries))

    def enable_torque(self, num_tries = RETRIES):
        retry = 0
        while(retry < num_tries):
            dxl.write1ByteTxRx(self.port, PROTOCOL, self.id, ADDR_TORQUE_ENABLE, 1)
            dcomm_result = dxl.getLastTxRxResult(self.port, PROTOCOL)
            derror = dxl.getLastRxPacketError(self.port, PROTOCOL)
            if dcomm_result != 0:
                print_v(dxl.getTxRxResult(PROTOCOL, dcomm_result))
                retry +=1
            elif derror != 0:
                print_v(dxl.getRxPacketError(PROTOCOL, derror))
                retry +=1
            else:
                return 0
        raise Exception("enable_torque failed after {} tries".format(num_tries))

    def disable_torque(self, num_tries = RETRIES):
        retry = 0
        while(retry < num_tries):
            dxl.write1ByteTxRx(self.port, PROTOCOL, self.id, ADDR_TORQUE_ENABLE, 0)
            dcomm_result = dxl.getLastTxRxResult(self.port, PROTOCOL)
            derror = dxl.getLastRxPacketError(self.port, PROTOCOL)
            if dcomm_result != 0:
                print_v(dxl.getTxRxResult(PROTOCOL, dcomm_result))
                retry +=1
            elif derror != 0:
                print_v(dxl.getRxPacketError(PROTOCOL, derror))
                retry +=1
            else:
                return 0
        raise Exception("disable_torque failed after {} tries".format(num_tries))

    # set_position() takes position in radians from -pi to pi
    def set_position(self, pos, num_tries = RETRIES):
        value = self.rad2value(pos)
        retry = 0
        while(retry < num_tries):
            dxl.write2ByteTxRx(self.port, PROTOCOL, self.id, ADDR_GOAL_POSITION, value)
            dcomm_result = dxl.getLastTxRxResult(self.port, PROTOCOL)
            derror = dxl.getLastRxPacketError(self.port, PROTOCOL)
            if dcomm_result != 0:
                print_v(dxl.getTxRxResult(PROTOCOL, dcomm_result))
                retry +=1
            elif derror != 0:
                print_v(dxl.getRxPacketError(PROTOCOL, derror))
                retry +=1
            else:
                return 0
        raise Exception("set_position failed after {} tries".format(num_tries))

    # set_speed() takes -1.0 to 1.0
    def set_speed(self, speed, num_tries = RETRIES):
        if(self.mode == 2):
            value = int(1023*abs(speed))
        elif(self.mode == 1):
            value = int(1023*abs(speed))
            if(speed>0.0):
                value = value + 1024
        retry = 0
        while(retry < num_tries):
            dxl.write2ByteTxRx(self.port, PROTOCOL, self.id, ADDR_MOVING_SPEED, value)
            dcomm_result = dxl.getLastTxRxResult(self.port, PROTOCOL)
            derror = dxl.getLastRxPacketError(self.port, PROTOCOL)
            if dcomm_result != 0:
                print_v(dxl.getTxRxResult(PROTOCOL, dcomm_result))
                retry +=1
            elif derror != 0:
                print_v(dxl.getRxPacketError(PROTOCOL, derror))
                retry +=1
            else:
                return 0
        raise Exception("set_speed failed after {} tries".format(num_tries))

    def set_torque_limit(self, torque, num_tries = RETRIES):
        retry = 0
        while(retry < num_tries):
            dxl.write2ByteTxRx(self.port, PROTOCOL, self.id, ADDR_TORQUE_LIMIT, int(torque * 1023))
            dcomm_result = dxl.getLastTxRxResult(self.port, PROTOCOL)
            derror = dxl.getLastRxPacketError(self.port, PROTOCOL)
            if dcomm_result != 0:
                print_v(dxl.getTxRxResult(PROTOCOL, dcomm_result))
                retry +=1
            elif derror != 0:
                print_v(dxl.getRxPacketError(PROTOCOL, derror))
                retry +=1
            else:
                return 0
        raise Exception("set_torque_limit failed after {} tries".format(num_tries))

    def get_position(self, num_tries = RETRIES):
        retry = 0
        while(retry < num_tries):
            pos = dxl.read2ByteTxRx(self.port, PROTOCOL, self.id, ADDR_PRESENT_POSITION)
            dcomm_result = dxl.getLastTxRxResult(self.port, PROTOCOL)
            derror = dxl.getLastRxPacketError(self.port, PROTOCOL)
            if dcomm_result != 0:
                print_v(dxl.getTxRxResult(PROTOCOL, dcomm_result))
                retry +=1
            elif derror != 0:
                print_v(dxl.getRxPacketError(PROTOCOL, derror))
                retry +=1
            else:
                return self.value2rad(pos)
        raise Exception("get_position failed after {} tries".format(num_tries))

    def get_speed(self, num_tries = RETRIES):
        retry = 0
        while(retry < num_tries):
            speed = dxl.read2ByteTxRx(self.port, PROTOCOL, self.id, ADDR_PRESENT_SPEED)
            dcomm_result = dxl.getLastTxRxResult(self.port, PROTOCOL)
            derror = dxl.getLastRxPacketError(self.port, PROTOCOL)
            if dcomm_result != 0:
                print_v(dxl.getTxRxResult(PROTOCOL, dcomm_result))
                retry +=1
            elif derror != 0:
                print_v(dxl.getRxPacketError(PROTOCOL, derror))
                retry +=1
            else:
                return (speed/1023.0)-1.0
        raise Exception("get_speed failed after {} tries".format(num_tries))

    def get_load(self, num_tries = RETRIES):
        retry = 0
        while(retry < num_tries):
            load = dxl.read2ByteTxRx(self.port, PROTOCOL, self.id, ADDR_PRESENT_LOAD)
            dcomm_result = dxl.getLastTxRxResult(self.port, PROTOCOL)
            derror = dxl.getLastRxPacketError(self.port, PROTOCOL)
            if dcomm_result != 0:
                print_v(dxl.getTxRxResult(PROTOCOL, dcomm_result))
                retry +=1
            elif derror != 0:
                print_v(dxl.getRxPacketError(PROTOCOL, derror))
                retry +=1
            else:
                return (load/1023.0)-1.0
        raise Exception("get_load failed after {} tries".format(num_tries))

    def get_temp(self, num_tries = RETRIES):
        retry = 0
        while(retry < num_tries):
            temp = dxl.read1ByteTxRx(self.port, PROTOCOL, self.id, ADDR_PRESENT_TEMP)
            dcomm_result = dxl.getLastTxRxResult(self.port, PROTOCOL)
            derror = dxl.getLastRxPacketError(self.port, PROTOCOL)
            if dcomm_result != 0:
                print_v(dxl.getTxRxResult(PROTOCOL, dcomm_result))
                retry +=1
            elif derror != 0:
                print_v(dxl.getRxPacketError(PROTOCOL, derror))
                retry +=1
            else:
                return temp
        raise Exception("get_temp failed after {} tries".format(num_tries))

    def get_voltage(self, num_tries = RETRIES):
        retry = 0
        while(retry < num_tries):
            voltage = dxl.read1ByteTxRx(self.port, PROTOCOL, self.id, ADDR_PRESENT_VOLTAGE)
            dcomm_result = dxl.getLastTxRxResult(self.port, PROTOCOL)
            derror = dxl.getLastRxPacketError(self.port, PROTOCOL)
            if dcomm_result != 0:
                print_v(dxl.getTxRxResult(PROTOCOL, dcomm_result))
                retry +=1
            elif derror != 0:
                print_v(dxl.getRxPacketError(PROTOCOL, derror))
                retry +=1
            else:
                return voltage/10.0
        raise Exception("get_voltage failed after {} tries".format(num_tries))

    def is_moving(self, num_tries = RETRIES):
        retry = 0
        while(retry < num_tries):
            moving = dxl.read1ByteTxRx(self.port, PROTOCOL, self.id, ADDR_MOVING)
            dcomm_result = dxl.getLastTxRxResult(self.port, PROTOCOL)
            derror = dxl.getLastRxPacketError(self.port, PROTOCOL)    
            if dcomm_result != 0:
                print_v(dxl.getTxRxResult(PROTOCOL, dcomm_result))
                retry += 1
            elif derror != 0:
                print_v(dxl.getRxPacketError(PROTOCOL, derror))
                retry +=1
            else:
                return moving
        raise Exception("is_moving failed after {} tries".format(num_tries))

    def deg2value(self, angle):
        return int(1023*(angle + 150.0)/300)

    def value2deg(self, value):
        return (300.0*value/1023.0)-150.0

    def rad2value(self, angle):
        return int(1023*(angle + 2.617993878)/5.235987756)

    def value2rad(self, value):
        return ((5.235987756*value/1023.0)-2.617993878)