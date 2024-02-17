from adafruit_servokit import ServoKit
import board
import busio
import time
import logging
# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# On the Jetson Nano
# Bus 0 (pins 28,27) is board SCL_1, SDA_1 in the jetson board definition file
# Bus 1 (pins 5, 3) is board SCL, SDA in the jetson definition file
# Default is to Bus 1; We are using Bus 0, so we need to construct the busio first ...
logging.info("Initializing Servos")
i2c_bus0=(busio.I2C(board.SCL_1, board.SDA_1))
logging.info("Initializing ServoKit")

kit = ServoKit(channels=16, i2c=i2c_bus0)

logging.info("Done initializing")

# [0]~[2] : FL // [3]~[5] : FR // [6]~[8] : RL // [9]~[11] : RR
cur_angle = None
#default positions to go back to
def_dict = {0: 110,
            1: 100,
            2: 60,
            4: 70,
            5: 110,
            6: 150,
            8: 110,
            9: 100,
            10: 130,
            12: 70,
            13: 110,
            14: 60}


if __name__ == '__main__':
    var_dict = def_dict.copy()
    try:
        for key, value in var_dict.items():
            kit.servo[key].angle = value
        while True:
            # motor_num is index of motor to rotate
            motor_num=int(input("Enter Servo to rotate (0-14): "))
            
            # new angle to be written on selected motor
            cur_angle=int(input("Enter new angles (0-180): "))
            
            prev_angle = var_dict[motor_num]
            sweep = range(prev_angle, cur_angle, 1) if (prev_angle < cur_angle) else range(prev_angle, cur_angle, -1)

            for degree in sweep:
                kit.servo[motor_num].angle = cur_angle
                time.sleep(0.01)
            var_dict[motor_num] = cur_angle
            
    except KeyboardInterrupt:
        logging.info("KeyboardInterrupt has been caught.")
        logging.info("Cleaning up...")
        for key, value in def_dict.items():
            kit.servo[key].angle = value
            logging.info("Done cleaning up")