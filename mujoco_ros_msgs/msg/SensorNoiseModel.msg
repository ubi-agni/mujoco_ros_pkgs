# Set the noise model of a sensor defining mean and standard deviation for each dimension
# For quaternion sensors noise is calculated in euler angles (rad), converted to a quaternion and then applied. Thus only three mean/std pairs are required!
string sensor_name
float64[] mean
float64[] std
uint8 set_flag
