import numpy as np
from robot_kinematics import DiffDriveKinematics, MecanumKinematics, ThreeWheelOmniKinematics, FourWheelOmniKinematics

robot = MecanumKinematics(L=0.6, R=0.1, W=0.8)

V = np.array([ 0.8, -0.4,  0.5 ])

W = robot.inverse(*V)

print("W =", W)

V_recovered = robot.forward(W)

print("Original =", V)
print("Recovered =", V_recovered)
