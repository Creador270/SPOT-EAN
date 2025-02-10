==============================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
# This message defines the velocity of the robot in the x and y components
float32 lin_vel_x
float32 lin_vel_y

================================================================================
# This message defines the angular velocity of the robot in the z orientation.
float32 ang_vel_z

================================================================================
# This message defines a pose with reference coordinate frame and timestamp
float32 height_z

================================================================================
# This message defines the type of action between: stop, move, reset
string action_type=move|stop|reset

================================================================================
