# Trajectory2D.msg
# Represents a 2D trajectory as a sequence of points.

std_msgs/Header header
geometry_msgs/Point[] points # Array of 2D points (z component will be ignored)