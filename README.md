# Drona_Avaition_Git

Task_1 Submission
Refer task_1.py
1. Script communicates with drone using socket library
2. class Pluto has functions  
    a) command_message_in for sending values to drone  
    b) msp_set_cmd sets takeoff,land commands  
    c) msp_set_raw sets throttle,pitch,roll,yaw values  
    d) request_message gets drone status  
    e) obs_attitude_val gets attitude message from drone  
    f) obs_altitude_val gets altitude message  
    g) obs_rc_channel_vals gets values of rc_channels  
    h) obs_imu_vals gets acc/gyro values  
3. In the main function
    Drone object is initialised

    Drone takes off first,
    then it goes up (throttle value: 1800) for some time
    then pitches forward (pitch value: 1600) for some time
    then rolls right (roll value: 1550) for some time
    then finally lands

Task_2 Submission
Refer task_2.py  
Associated file: camera_calibration\calib_data\MultiMatrix.npz  
For calibrating camera: camera_calibration\calib_data\calib_cam

1. In class constructor
    a) Parameters are initialised for OpenCV part
    b) Initial PID parameters are initialised
2. Class functions  
    a) As in task_1.py, command_message_in, msp_set_cmd, msp_set_raw, request_message functions are used    
    b) The heart of the script is the pid function. Here the aruco-marker is detected first, then if it is present in the frame, the pid controller gives the appropriate throttle, roll, pitch, yaw values to the drone so that it reaches the setpoint and stays there. If in case aruco is not detected, then same rc_values are sent or if the drone moves out of camera range, then the drone is made to land.  
    c) hover function is used to make the drone hover at the setpoint for some time  
    d) pitching and rolling functions adjust kp values for pitch and roll. The pid function is called to make the drone move to the setpoint.   
    e) rectangle_path makes the drone to moves along a rectangular trajectory it goes to 1st point and hovers there, then it goes to 2nd point by pitch it goes to 3rd point by roll it goes to 4th point by pitch then hovers there.  

3. In the main function, drone object is initialised, drone is armed it takes off goes in a rectangular trajectory then lands after reaching the final point
