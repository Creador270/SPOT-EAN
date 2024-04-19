from servo_controller import Controllers
import time

class StandUpController(Controllers):
    def standUp(self):
        # Define the servo indices for the shoulders, lower legs, and upper legs
        shoulders = [2, 6, 10, 14]
        lower_legs = [0, 4, 8, 12]
        upper_legs = [1, 5, 9, 13]

        # Activate the servos in the specified order
        for servo_group in [shoulders, lower_legs, upper_legs]:
            for servo in servo_group:
                standing_angle = self._servo_offsets[servo]
                self.kit.servo[servo].angle = standing_angle
            time.sleep(1.5)  # wait for 1.5 seconds

if __name__=="__main__":
    controller = StandUpController()
    controller.standUp()