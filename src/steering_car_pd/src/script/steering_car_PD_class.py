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

    
    def compute_control_action(self, x_k, ref_k):     
        
        # (Controller befined by Camille Leblanc)
        
        x_position = x_k[0]
        y_position = x_k[1]
        psi = x_k[2]
        
        x_ref = ref_k[0]
        y_ref = ref_k[1]
                
        # Throttle
        delta_x = x_position - x_ref
        delta_y = y_position - y_ref
        epsilon_d = - np.sqrt(delta_x**2 + delta_y**2)
        
        #print(f'epsilon_d={epsilon_d},delta_x={delta_x},delta_y={delta_y}')
        
        dot_epsilon_d = (epsilon_d - self.epsilon_d_prev) / self.Ts
                
        v = self.Kp_v * epsilon_d + self.Kd_v * dot_epsilon_d
        
        if v > 0.3: 
            v = 0.3
        if v < -0.3:
            v = -0.3
            
        # Steering 
        
        epsilon_psi = (np.arctan2(-delta_y,-delta_x) - (psi) + np.pi) % (2*np.pi) - np.pi
        
        dot_epsilon_psi = (epsilon_psi - self.epsilon_psi_prev) / self.Ts
        
        delta = self.Kp_psi * epsilon_psi + self.Kd_psi * dot_epsilon_psi
        if delta > 0.6:
            delta = 0.6
        if delta < -0.6:
            delta = -0.6
        
        u_k = np.array([delta,
                        -v]) # minus because otherwise it turns on and follow the trajectory wrong way round
        
        self.epsilon_psi_prev = epsilon_psi
        self.epsilon_d_prev = epsilon_d
        
        return u_k