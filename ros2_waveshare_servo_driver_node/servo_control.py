import math
from ros2_waveshare_servo_driver_node.STservo_sdk import *  # Uses STServo SDK library


class ServoControl:
    def __init__(self, config):
        # Initialize PortHandler instances
        self.config = config
        board_front = config['board_front']
        board_back = config['board_back']
        self.port_handler_front = PortHandler(board_front['device_name'])
        self.port_handler_back = PortHandler(board_back['device_name'])

        # Initialize PacketHandler instances
        self.packetHandlerFront = sts(self.port_handler_front)
        self.packetHandlerBack = sts(self.port_handler_back)

        # Open ports
        if self.port_handler_front.openPort():
            print("Succeeded to open port front")
        else:
            print("Failed to open port front")

        if self.port_handler_back.openPort():
            print("Succeeded to open port back")
        else:
            print("Failed to open port back")

        # Set baud rates
        if self.port_handler_front.setBaudRate(board_front['baudrate']):
            print("Succeeded to set baud rate front")
        else:
            print("Failed to set baud rate front")

        if self.port_handler_back.setBaudRate(board_back['baudrate']):
            print("Succeeded to set baud rate back")
        else:
            print("Failed to set baud rate back")

    def __del__(self):
        self.port_handler_front.closePort()
        self.port_handler_back.closePort()

    def get_angle(self, id):
        packetHandler = self.packetHandlerFront if id <= 6 else self.packetHandlerBack 
        position, result, error = packetHandler.ReadPos(id)
        if result == COMM_SUCCESS:
            print(f"Servo ID {id} Position: {position}")
            angle = position / 2047 * math.pi
            return angle
        else:
            print(f"Error reading position: {packetHandler.getTxRxResult(result)}")
            return None
        
    def get_joint_by_name(self, name):
        return next((joint for joint in self.config['joints'] if joint['name'] == name), None)

            
    def process_msg(self,name,pos,vel,eff):
        joint = self.get_joint_by_name(name)
        if joint is None:
            print(f"Joint {name} not found in config.")
            return

        id = joint['servo_id']
        board = joint['board']
        max_steps = joint['max_steps']
        min_steps = joint['min_steps']
        max_vel = joint['max_vel']
        acc = joint['acc']
        offset = joint['offset']
        inverted = joint['inverted']
        
        steps = pos/math.pi*2047
        if inverted:
            steps = 4095 - steps -offset
        else:
            steps = steps + offset
        velocity = vel/math.pi*2047
        if velocity > max_vel:
            velocity = max_vel
            

        # Set position
        self.set_pos(board,id, int(steps), int(velocity),int(acc),int(min_steps),int(max_steps))
        
        
        if eff == 0:
            self.enable_torque(id, False)

    def set_pos(self, board, id: int, servo_position: float, velocity: float, acceleration: float, min_steps: int, max_steps:int ):
        packetHandler = self.packetHandlerFront if board == "front" else self.packetHandlerBack
        # Prevents mechanical damage
        if servo_position < min_steps:
            servo_position = min_steps
            print("limited move from servo:%s" % id)
        elif servo_position > max_steps:
            servo_position = max_steps
            print("limited move from servo:%s" % id)
        # Set goal position
        sts_comm_result = packetHandler.SyncWritePosEx(
            id, servo_position, velocity, acceleration
        )
        if sts_comm_result != True:
            print("[ID:%03d] groupSyncWrite add param failed" % servo_position)

    def move_positions(self):
        result_front = self.packetHandlerFront.groupSyncWrite.txPacket()
        result_back = self.packetHandlerBack.groupSyncWrite.txPacket()

        # if result_front != COMM_SUCCESS:
        #     print("%s"% self.packetHandlerFront.getTxRxResult(result_front))
        # if result_back != COMM_SUCCESS:
        #     print("%s"% self.packetHandlerBack.getTxRxResult(result_back))

        self.packetHandlerFront.groupSyncWrite.clearParam()
        self.packetHandlerBack.groupSyncWrite.clearParam()

    def enable_torque(self, id: int, torque_state: bool):
        packetHandler = self.packetHandlerFront if id <= 6 else self.packetHandlerBack

        # Torque states
        TORQUE_ENABLE = 1  # Enable torque
        TORQUE_DISABLE = 0  # Disable torque

        torque_value = TORQUE_ENABLE if torque_state else TORQUE_DISABLE
        sts_comm_result, sts_error = packetHandler.write1ByteTxRx(
            id, STS_TORQUE_ENABLE, torque_value
        )
        if sts_comm_result != COMM_SUCCESS:
            print(
                "Failed to change torque state: %s"
                % packetHandler.getTxRxResult(sts_comm_result)
            )
        elif sts_error != 0:
            print("Servo error: %s" % packetHandler.getRxPacketError(sts_error))
        else:
            state_text = "enabled" if torque_value == TORQUE_ENABLE else "disabled"
            print(f"Torque {state_text} for Servo ID {id}")

    def get_load(self, id: int):
            packetHandler = self.packetHandlerFront if id <= 6 else self.packetHandlerBack
            load_low, result_low, error_low = packetHandler.read1ByteTxRx(id, STS_PRESENT_LOAD_L)
            load_high, result_high, error_high = packetHandler.read1ByteTxRx(id, STS_PRESENT_LOAD_H)
            
            if result_low == COMM_SUCCESS and result_high == COMM_SUCCESS:
                load = (load_high << 8) + load_low
            # print(f"Servo ID {id} Load: {load}")
                return load
            else:
                if result_low != COMM_SUCCESS:
                    print(f"Error reading load low byte: {packetHandler.getTxRxResult(result_low)}")
                if result_high != COMM_SUCCESS:
                    print(f"Error reading load high byte: {packetHandler.getTxRxResult(result_high)}")
                return None

    def get_present_current(self, id: int):
            packetHandler = self.packetHandlerFront if id <= 6 else self.packetHandlerBack
            current_low, result_low, error_low = packetHandler.read1ByteTxRx(id, STS_PRESENT_CURRENT_L)
            current_high, result_high, error_high = packetHandler.read1ByteTxRx(id, STS_PRESENT_CURRENT_H)
            
            if result_low == COMM_SUCCESS and result_high == COMM_SUCCESS:
                current = (current_high << 8) + current_low
            # print(f"Servo ID {id} current: {current}")
                return current
            else:
                if result_low != COMM_SUCCESS:
                    print(f"Error reading current low byte: {packetHandler.getTxRxResult(result_low)}")
                if result_high != COMM_SUCCESS:
                    print(f"Error reading current high byte: {packetHandler.getTxRxResult(result_high)}")
                return None
            
    def read_movement(self, id):
        packetHandler = self.packetHandlerFront if id <= 6 else self.packetHandlerBack
        return packetHandler.ReadMoving(id)
    
def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min