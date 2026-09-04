import numpy as np

class Kinematics:
    def __init__(self,L,R,W):
        self.L = L # track width (distance between the left and right wheels)
        self.R = R #wheel radius
        self.W = W # wheelbase (distance between the front and rear wheels)

        self.M_forward = np.array([]) # forward mapping matrix
        self.M_inverse = np.array([]) # inverse mapping matrix
        
    def forward(self,wheel_speeds): # initialization of the parent forward kinematics function
        raise NotImplementedError 
    def inverse(self,Vx,Vy,wz): # initialization of the parent inverse kinematics function
        raise NotImplementedError


class DiffDriveKinematics(Kinematics):
    def __init__(self, L,R,W):
        super().__init__(L,R,W)
        self.M_forward = np.array([[R/2, R/2],
                                    [R/L, -R/L]])
        
        self.M_inverse = np.linalg.inv(self.M_forward)

    def forward(self,wheel_speeds):
        w1, w2, w3, w4 = wheel_speeds
        Vx, wz = self.M_forward @ np.array([w1, w2])
        Vy = 0 
        return np.array([Vx,Vy, wz])
    
    def inverse(self,Vx,Vy,wz):
        w1, w2 = self.M_inverse @ np.array([Vx, wz])
        w3, w4 = 0,0
        return np.array([w1, w2,w3,w4])
    
class MecanumKinematics(Kinematics):
    def __init__(self, L,R,W):
        super().__init__(L,R,W)
        #the distance from the center of the robot's Y-axis to the centre of each wheel
        Lx = W/2
        #the distance from the center of the robot's X-axis to the centre of each wheel
        Ly = L/2
        self.M_forward = np.array([[1, 1, 1, 1],
                                   [-1, 1, 1, -1],
                                   [-1/(Lx+Ly), 1/(Lx+Ly), -1/(Lx+Ly), 1/(Lx+Ly)]])
        
        self.M_inverse = np.array([[1,-1, -(Lx+Ly)],
                                   [1, 1, (Lx+Ly)],
                                   [1, 1, -(Lx+Ly)],
                                   [1,-1, (Lx+Ly)]])

    def forward(self,wheel_speeds):
        w1, w2, w3, w4 = wheel_speeds
        Vx, Vy, wz = (self.R/4) * (self.M_forward @ np.array([w1, w2, w3, w4]))
        return np.array([Vx,Vy, wz])

    def inverse(self,Vx,Vy,wz):
        w1, w2, w3,w4= (1/self.R) * (self.M_inverse @ np.array([Vx, Vy, wz]))
        return np.array([w1, w2, w3, w4])
               
class ThreeWheelOmniKinematics(Kinematics):
    def __init__(self, L,R,W):
        super().__init__(L,R,W)
        #calculate the distance from the center of the robot to the center of each wheel
        self.r = L /np.sqrt(3)

        self.M_forward = np.array([[-2/3,1/3,1/3],
                                   [0,-1/np.sqrt(3), 1/np.sqrt(3)],
                                   [1/(3*self.r),1/(3*self.r), 1/(3*self.r)]])
        
        self.M_inverse = np.linalg.inv(self.M_forward)

    def forward(self,wheel_speeds):
        w1, w2, w3, w4 = wheel_speeds
        Vx, Vy, wz = self.R * (self.M_forward @ np.array([w1, w2, w3]))
        return np.array([Vx,Vy, wz])
    
    def inverse(self,Vx,Vy,wz):
        w1, w2, w3 = (1/self.R) * (self.M_inverse @ np.array([Vx, Vy, wz]))
        w4 = 0
        return np.array([w1, w2, w3,w4])
        
class FourWheelOmniKinematics(Kinematics):
    def __init__(self, L,R,W):
        super().__init__(L,R,W)
        # Calculate the distance from the center of the robot to each wheel
        self.r = np.hypot(L / 2, W / 2)
        self.M_forward = np.array([[-np.sqrt(2)/4, np.sqrt(2)/4, np.sqrt(2)/4, -np.sqrt(2)/4],
                                   [np.sqrt(2)/4, np.sqrt(2)/4, np.sqrt(2)/4, np.sqrt(2)/4],
                                   [1/(4*self.r), -1/(4*self.r), 1/(4*self.r), -1/(4*self.r)]])

        self.M_inverse = np.linalg.pinv(self.M_forward)

    def forward(self,wheel_speeds):
        w1, w2, w3, w4 = wheel_speeds
        Vx, Vy, wz = self.R * (self.M_forward @ np.array([w1, w2, w3, w4]))
        return np.array([Vx,Vy, wz])
        
    def inverse(self,Vx,Vy,wz):
        w1, w2, w3, w4 = (1/self.R) * (self.M_inverse @ np.array([Vx, Vy, wz]))
        return np.array([w1, w2, w3, w4])