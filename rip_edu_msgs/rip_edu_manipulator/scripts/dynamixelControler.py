from dynamixel_sdk import *

# Dynamixel constants
BAUDRATE = 57600                    # Dynamixel baudrate
DXL_MAXIMUM_POSITION_VALUE = 4095   # Dynamixel will rotate between this value
MX_MOVE_STEP = 0.088                # Dynamixel Move degree step from [0-4095]
MX_SPEED_STEP =  0.114              # Dynamixel Speed step from [0-1023]

# Control table addresses for MX-64 and MX-28
ADDR_MX_ID = 3
ADDR_MX_TORQUE_ENABLE      = 24
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_SET_VELOCITY       = 32
ADDR_MX_SET_TORQUE         = 34
ADDR_MX_PRESENT_POSITION   = 36
ADDR_MX_ACC                = 73

# Convert degrees to Dynamixel units
def deg_to_dxl_units(deg) :
    return int((deg / 360.0) * DXL_MAXIMUM_POSITION_VALUE)

# Convert Dynamixel units to degrees
def dxl_units_to_deg(dxl) :
    return (dxl / DXL_MAXIMUM_POSITION_VALUE) * 360.0

class dyControl() :
    def __init__(self, DEVICENAME = '/dev/ttyUSB0', PROTOCOL_VERSION = 1.0, max_motor = 4) -> None :
        self.max_motor = max_motor
        
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        
        if self.portHandler.openPort() :
            print("Succeeded to open the port.")
        else :
            e = "Failed to open the port."
            raise e
        if self.portHandler.setBaudRate(BAUDRATE) :
            print("Succeeded to change the baudrate.")
        else :
            e = ("Failed to change the baudrate.")
            raise e
        
    """Comunication fuction.
    """
    
    def write_1(self, id : int, reg_num : int, reg_value : int) -> None :
        """Write 1 Byte Data and send.

        Args:
            id (int): ID of the motor that need to send command.
            reg_num (int): Registor Number. Depent on each type of Dynamixel motor and version.
            reg_value (int): Registor Value. the data that you want to sent.
        """
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, reg_num, reg_value)
        self.check_err(dxl_comm_result, dxl_error)

    def read_1(self, id : int, reg_num : int) :
        """Read 1 Byte Data.

        Args:
            id (int): ID of the motor that need to send command.
            reg_num (int): Registor Number. Depent on each type of Dynamixel motor and version.

        Returns:
            AnyType: Return Data from motor.
        """
        reg_data, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, id, reg_num)
        self.check_err(dxl_comm_result, dxl_error)
        return reg_data

    def write_2(self, id : int, reg_num : int, reg_value : int) -> None :
        """Write 2 Byte Data and send.

        Args:
            id (int): ID of the motor that need to send command.
            reg_num (int): Registor Number. Depent on each type of Dynamixel motor and version.
            reg_value (int): Registor Value. the data that you want to sent.
        """
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, reg_num, reg_value)
        self.check_err(dxl_comm_result, dxl_error)

    def read_2(self, id : int, reg_num_low : int) :
        """Read 2 Byte Data.

        Args:
            id (int): ID of the motor that need to send command.
            reg_num (int): Registor Number. Depent on each type of Dynamixel motor and version.

        Returns:
            AnyType: Return Data from motor.
        """
        reg_data, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, id, reg_num_low)
        self.check_err(dxl_comm_result, dxl_error)
        return reg_data
        
    def check_err(self, comm_result, dxl_err) :
        """Check error from communicate.
        """
        if comm_result != COMM_SUCCESS :
            e = (f"{self.packetHandler.getTxRxResult(comm_result)}")
            print(e)
            # raise Exception(e)
        elif dxl_err != 0 :
            e = (f"{self.packetHandler.getRxPacketError(dxl_err)}")
            print(e)
            # raise Exception(e)
            
    """Action fuction.
    """

    def enable_motor_torque(self, id : int, enable : bool = True) -> None :
        """Enable motor.

        Args:
            id (int): ID of the motor.
            enable (bool): Enable stage.
        """
        if enable :
            self.write_1(id, ADDR_MX_TORQUE_ENABLE, 1)
        else:
            self.write_1(id, ADDR_MX_TORQUE_ENABLE, 0)

    def move_synchrony_motors(self, angles : list, action_time : float) -> None :
        """Move multiple motors by index of list with synchrony speed on action time.

        Args:
            angles (list): list of each angle(deg) for each motors by index of list.
            action_time (float): Time(s) for all motor to action.
        """
        for i, angle in enumerate(angles) :
            pos = self.get_motor_deg(i+1)
            rpm = (abs(angle - pos) / action_time) / 6
            rpm = 0.1 if rpm == 0 else rpm
            for _ in range(100) :
                try :
                    self.set_speed(i+1, rpm)
                    break
                except :
                    self.set_speed(i+1, rpm)
        
        time.sleep(0.1)
        for i, angle in enumerate(angles) :
            self.move_motor(i+1, angle)
    
    def move_mutiple_motors(self, angles : list, rpm : float = 0) -> None :
        """Move multiple motors by index of list.

        Args:
            angles (list): list of each angle for each motors by index of list.
        """
        for i, angle in enumerate(angles) :
            for _ in range(40) :
                try :
                    self.set_speed(i+1, rpm)
                    break
                except :
                    self.set_speed(i+1, rpm)
            self.move_motor(i+1, angle)

    def move_motor(self, id : int, angle : float) -> None :
        """Move one motor by ID.

        Args:
            id (int): Id of the motor that need to move.
            angle (float): angle of the motor.
        """
        dxl_goal_position = deg_to_dxl_units(angle)
        self.write_2(id, ADDR_MX_GOAL_POSITION, dxl_goal_position)

    def set_id(self, old_id : int, new_id : int) -> None :
        """Change motor ID.

        Args:
            old_id (int): old ID of the motor that you want to change
            new_id (int): new motor ID.
        """
        self.write_1(old_id, ADDR_MX_ID, new_id)

    def reset_id(self) -> None :
        """Reset all motors ID that connected to 1. this fuction will take some time to action.
        """
        print("Resetting...")
        for dxl_id in range(1, 254) :
            self.set_id(dxl_id, 1)
        print("Reset Done.")

    def set_speed(self, id : int, rpm : float) -> None : 
        """Set speed of motor by ID.

        Args:
            id (int): ID of the motor that need to set.
            rpm (float): speed of motor in rpm.
        """
        speed_dxl = int(abs(rpm) / MX_SPEED_STEP)
        speed_dxl = 1023 if speed_dxl > 1023 else speed_dxl
        self.write_2(id, ADDR_MX_SET_VELOCITY, speed_dxl)

    def set_torque(self, id : int, torque : int) -> None : 
        """Set torque of the motor by ID.

        Args:
            id (int): Id of the motor that want to set torque.
            torque (int): torque of the motor. this value have range at 0 to 1023. please reminding that each motor type have different maximum speed.
        """
        self.write_2(id, ADDR_MX_SET_TORQUE, torque)
        
    """Request fuction.
    """
        
    def get_motor_deg(self, id : int) -> float :
        """Get position of the moter as Degree unit.

        Args:
            id (int): Id of the motor.

        Returns:
            float: Position of the motor as Degree.
        """
        dxl = self.read_2(id, ADDR_MX_PRESENT_POSITION)
        return dxl_units_to_deg(dxl)
            
# if __name__ == "__main__" :
#     dy = dyControl("COM5")
#     dy.move_mutiple_motors([90, 90, 90, 90], 10) # init deg.
#     while True :
#         try :
#             id = int(input("Please Registor ID : "))
#             break
#         except :
#             print("Invalid Datatype!")
#     # dy.set_speed(id, 100)
#     while True :
#         dat = int(input("deg_input : "))
#         dat2 = int(input("speed_input : "))
#         dy.set_speed(id, dat2)
#         dy.move_motor(id, dat)
