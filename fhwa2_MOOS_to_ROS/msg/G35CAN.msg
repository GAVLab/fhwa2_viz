Header header
# Assume that all CAN messages will come from MOOS with the same time step
time MOOSstamp
# Steering information is in radians
fhwa2_MOOS_to_ROS/FourWheels WheelSpeed
    float32 FrontLeft
    float32 FrontRight
    float32 RearLeft
    float32 RearRight
float32 SteerAngle
# speed displayed to the driver in m/s (delivered from MOOS in km/hr)
float32 VehicleSpeed