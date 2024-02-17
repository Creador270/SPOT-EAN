from adafruit_servokit import ServoKit
import board
import busio

i2c_bus0 = busio.I2C(board.SCL_1, board.SDA_1)

kit = ServoKit(channels=16, i2c=i2c_bus0)

#Parado

#DOWN
kit.servo[0].angle= 110 #front_left

kit.servo[4].angle= 70 #front_right

kit.servo[8].angle= 110 #back_left

kit.servo[12].angle= 70 #back_right

#MID

kit.servo[1].angle= 100 #front_left

kit.servo[5].angle= 110 #front_right

kit.servo[9].angle= 100 #back_left

kit.servo[13].angle= 110 #back_right

#UP

kit.servo[2].angle= 60 #front_left

kit.servo[6].angle= 150 #front_right

kit.servo[10].angle= 130 #back_left

kit.servo[14].angle= 60 #back_right