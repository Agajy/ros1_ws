import numpy as np

class SteeringCarPD:
    def __init__(self, Ts, Kp_v, Kd_v, Kp_psi, Kd_psi):
        
        # Sampling time
        self.Ts = Ts
        
        # Controller parameters
        self.Kp_v = Kp_v
        self.Kd_v = Kd_v
        self.Kp_psi = Kp_psi
        self.Kd_psi = Kd_psi
        
        # Previous epsilon_d (to compute the derivative)
        self.epsilon_d_prev = 0.0
        
        # Previous epsilon_psi (to compute the derivative)
        self.epsilon_psi_prev = 0.0

    
    def compute_control_action(self, x_k):     
        delta_x = x_k[0]
        delta_y = x_k[1]
        psi = x_k[2]
        
        # Throttle
        epsilon_d = - np.sqrt(delta_x**2 + delta_y**2)     
        dot_epsilon_d = (epsilon_d - self.epsilon_d_prev) / self.Ts
                
        v = self.Kp_v * epsilon_d + self.Kd_v * dot_epsilon_d
        
        v = self.saturation(v, 0.3)  # Limit the speed to 0.3 m/s
            
        # Steering 
        epsilon_psi = (np.arctan2(-delta_x,-delta_y) - (psi)+np.pi) % (2*np.pi)-np.pi
        dot_epsilon_psi = (epsilon_psi - self.epsilon_psi_prev) / self.Ts
        
        delta = self.Kp_psi * epsilon_psi + self.Kd_psi * dot_epsilon_psi
        delta = self.saturation(delta, 0.6)  # Limit the steering angle to 0.6 radians
        
        u_k = np.array([delta,-v]) # minus because otherwise it turns on and follow the trajectory wrong way round
        
        self.epsilon_psi_prev = epsilon_psi
        self.epsilon_d_prev = epsilon_d
        
        return u_k
    
    def saturation(self, x, limit):
        if x > limit:
            x = limit
        if x < -limit:
            x = -limit
        return x