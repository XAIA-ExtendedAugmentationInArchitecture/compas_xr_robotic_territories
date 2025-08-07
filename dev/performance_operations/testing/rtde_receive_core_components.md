# rtde_receive documented features:

# 1. getActualQ()                 
#    → Current joint angles (rad)
# 2. getActualTCPPose()           
#    → Current TCP pose [x, y, z, rx, ry, rz]
# 3. getActualTCPSpeed()          
#    → Current Cartesian speed [vx, vy, vz, vrx, vry, vrz]
# 4. getActualTCPForce()          
#    → Measured TCP forces/torques [Fx, Fy, Fz, Tx, Ty, Tz]
# 5. getActualCurrent()           
#    → Joint electrical currents (A)
# 6. getJointControlOutput()      
#    → Controller output torques (Nm)
# 7. getActualDigitalInputBits()  
#    → Digital input bitmask (0–63)
# 8. getActualDigitalOutputBits() 
#    → Digital output bitmask (0–63)
# 9. getStandardAnalogInput0()    
#    → Analog input channel 0 (V or mA)
# 10. getStandardAnalogOutput1()  
#    → Analog output channel 1 (V or mA)
# 11. getRobotMode()              
#    → Current robot mode (e.g., idle, running)
# 12. getSafetyMode()             
#    → Safety state (e.g., normal, protective stop)
# 13. getActualToolAccelerometer()
#    → Tool‐frame accelerometer data [ax, ay, az]
# 14. getActualMomentum()         
#    → Cartesian momentum magnitude
# 15. getTargetQ()                
#    → Target joint angles of the motion plan
# 16. getTargetTCPPose()          
#    → Target TCP pose of the motion plan