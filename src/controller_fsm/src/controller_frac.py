#!/usr/bin/env python

"""
======================================================
 Fichier     : controller_frac.py
 Auteur      : Aurélien Garreau
 Créé en     : 2025
 Description : fonction pour le controleur fractional sliding mode fait par Marcone (ne fonctionne pas encore)
======================================================
"""

import numpy as np
from scipy.spatial.transform import Rotation as Rot
import numpy as np
from scipy.special import gamma


class Controller():
    '''
    image based fractional nonsingular fast terminal sliding mode controller
    for ugv trajectory tracking
    '''

    def __init__(self,
                 alpha_x, alpha_y, alpha_psi,
                 w_y, w_psi, sign_psi,
                ws1, ws2, alpha1, k,
                ws12, ws22, alpha12, k2):

        # for lateral controller
        self.alpha_y, self.alpha_psi = alpha_y, alpha_psi
        self.w_y= w_y
        self.w_psi = w_psi
        self.sign_psi = sign_psi
        self.ws1 = ws1
        self.ws2 = ws2
        self.alpha1 = alpha1 
        self.k = k

        # for longitudinal controller
        self.alpha_x = alpha_x
        self.ws12 = ws12
        self.ws22 = ws22
        self.alpha12 = alpha12
        self.k2 = k2

        self.dp_x_old = 0
        self.dp_y_old = 0
        self.dpsi_old = 0
        self.dp_dot_old = np.array([0,0,0,0], float) # dp_x_dot dp_y_dot dp_psi_dot

        self.started = False

    def fracDeriv(self, alpha, dx, dx_dot, dt):
        # first order approximation
        if alpha>0:
            total_sum_1 = -alpha*gamma(-alpha)*(dt**(-alpha))*dx/(gamma(1-alpha)**2.)
            total_sum_2 = alpha*gamma(1-alpha)*(dt**(1.-alpha))*dx_dot/(gamma(1.-alpha)*gamma(2.-alpha))
            fracder = total_sum_1+total_sum_2
            return fracder
        elif alpha<0:
            total_sum = (dt**(-alpha))*dx/(-alpha) - (dt**(1+(-alpha)))*dx_dot/(1+(-alpha))
            fracint = (1./gamma((-alpha)))*total_sum
            return fracint
        else:
            return 0.0
        
    def inner_prod(self,v1,v2):
        return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]

    def cross_prod(self,v1,v2):
        a = v1[1]*v2[2] - v1[2]*v2[1]
        b = v1[2]*v2[0] - v1[0]*v2[2]
        c = v1[0]*v2[1] - v1[1]*v2[0]
        return np.array([a,b,c])
    
    def euler_ang_dif(self,angle1,angle2):
        q1 = np.array([np.cos(angle1/2), 0*np.sin(angle1/2), 0*np.sin(angle1/2), 1.*np.sin(angle1/2)])
        q2 = np.array([np.cos(angle2/2), 0*np.sin(angle2/2), 0*np.sin(angle2/2), 1.*np.sin(angle2/2)])
        
        q2_conj = q2
        q2_conj[1:] = -q2_conj[1:]

        q1q2conj = np.array([0,0,0,0],float)
        q1q2conj[0] = q1[0]*q2_conj[0] - self.inner_prod(q1[1:],q2_conj[1:])
        q1q2conj[1:] = q1[0]*q2_conj[1:] + q2_conj[0]*q1[1:] + self.cross_prod(q1[1:],q2_conj[1:])

        return np.arctan2(2*q1q2conj[0]*q1q2conj[3]+q1q2conj[1]*q1q2conj[2], 1.-2.*(q1q2conj[2]**2.+q1q2conj[3]**2.))


    def vel_from_kinematics(self, curr_velocity, curr_steering):
        # Find ugv velocities from kinematic equations
        lr = 0.14
        beta_ugv = np.arctan2(lr*np.tan(curr_steering),(0.28))
        psi_ugv_dot = curr_velocity*np.cos(beta_ugv)*np.tan(curr_steering)/(0.28)
        ugv_velocity_y = curr_velocity*np.sin(beta_ugv)
        ugv_velocity_x = curr_velocity*np.cos(beta_ugv)
        ugv_w = curr_velocity*np.cos(beta_ugv)*np.tan(curr_steering)/(0.28)
        return ugv_velocity_x, ugv_velocity_y, ugv_w
    
    def longit_controller(self, dp_x, dp_x_dot, dp_x_frac_f, dp_x_frac_f_dot):
        f1 = lambda value, alpha: np.sign(value)*abs(value)**alpha
        f2 = lambda value, alpha: alpha*abs(value)**(alpha-1)

        if abs(dp_x)>0:
            s = self.ws12*dp_x_dot + self.ws22*dp_x_frac_f 
            w5 = 0.2
            acc = self.k2*np.clip(s,-np.inf,np.inf) + dp_x_frac_f_dot*w5
        else:
            acc = 0.0

        return acc
    
    def func_psi(self, dpsi, dp_x, dp_y):
        return dpsi/(self.sign_psi**(abs(dp_x)+abs(dp_y)))
    
    def func_psi_dot(self, dpsi, dp_x, dp_y, dpsi_dot, dp_x_dot, dp_y_dot):
        return (dpsi_dot - np.log(self.sign_psi)*dpsi*(np.sign(dp_x)*dp_x_dot + np.sign(dp_y)*dp_y_dot))/(self.sign_psi**(abs(dp_x)+abs(dp_y)))

    
    def lateral_controller(self, dp_x, dp_y, dp_x_dot, dp_y_dot, dpsi, dpsi_dot, 
                           e_frac,e_frac_dot,
                           curr_steering, curr_velocity, acc, dp_x_dot_abs, fw,fw_dot,fw_ddot, acc_abs,
                           ugv_velocity_x, ugv_w
                           ):
        ##########################
        # ugv dynamic parameters
        Caf = 1.3*1e3
        Car = Caf
        m = 3.
        lf = 0.14
        lr = 0.14
        Iz = 0.02
        g = 9.81
        ###########################

        if(abs(ugv_velocity_x)>0.01):
            ln_who = np.log(self.sign_psi)
            e_abs_dot = np.sign(dp_x)*dp_x_dot + np.sign(dp_y)*dp_y_dot
            c3 = 1./(self.sign_psi**(np.abs(dp_x) + np.abs(dp_y)))
            c1 = (-2.*ln_who*dpsi_dot*e_abs_dot + \
                   dpsi*(ln_who**2)*(e_abs_dot**2) - \
                   dpsi*ln_who*np.sign(dp_x)*acc
                   )*c3
            c2 = self.w_y + dpsi*ln_who*np.sign(dp_y)*self.w_psi*c3

            # error
            e_dot = dp_y_dot*self.w_y + fw_dot*self.w_psi

            acc_cinetic = 10.0
            div = (acc_cinetic + 0.3)*(c2+lf*self.w_psi*c3/Iz)
            s = self.ws1*e_dot + self.ws2*e_frac
            bound1,bound2 = np.inf,np.inf
            ws5 = self.ws2/self.ws1
            if div>0:
                steering = (self.k*np.clip(s, -bound1,bound1))/div + ((ws5)*np.clip(e_frac_dot,-bound2, bound2) + self.w_psi*c1 + ugv_velocity_x*ugv_w*c2)/div
            else:
                steering = curr_steering
            scalar = 1
        else:
            steering = curr_steering

        return steering
    
    def run(self, dp_x, dp_y, dpsi, curr_steering, curr_velocity, dt):

        assert not (np.isnan(dp_x) or np.isnan(dp_y) or np.isnan(dpsi)), " lateral_controll 1"
        factor = 0.3 #0.3

        if not self.started:
            self.dp_x_old = dp_x
            self.dp_y_old = dp_y
            self.dpsi_old = dpsi
            self.dp_x_dot_old = 0.0
            self.X = np.matrix([dp_x, dp_y, dpsi, 0.0, 0.0, 0.0, 0.0, 0.0]).T
            dp_x_dot, dp_y_dot, dpsi_dot = 0.0, 0.0, 0.0
            dp_x_dot_abs = 0.0
            self.fw_dot_old = 0.0
        else:
            dp_x = dp_x*factor + self.dp_x_old*(1.-factor)
            dp_y = dp_y*factor + self.dp_y_old*(1.-factor)
            dpsi = dpsi*factor + self.dpsi_old*(1.-factor)

            dp_x_dot = factor*(dp_x-self.dp_x_old)/dt + self.dp_dot_old[0]*(1.-factor)
            dp_y_dot = factor*(dp_y-self.dp_y_old)/dt + self.dp_dot_old[1]*(1.-factor)
            dpsi_dot = factor*self.euler_ang_dif(dpsi,self.dpsi_old)/dt + self.dp_dot_old[2]*(1.-factor)
            dp_x_dot_abs = factor*(abs(dp_x)-abs(self.dp_x_old))/dt + self.dp_dot_old[3]*(1.-factor)


        ugv_velocity_x, ugv_velocity_y, ugv_w = self.vel_from_kinematics(curr_velocity, curr_steering)

        # Compute weighting function,
        fw = self.func_psi(dpsi, dp_x, dp_y)
        fw_dot = self.func_psi_dot(dpsi, dp_x, dp_y, dpsi_dot, dp_x_dot, dp_y_dot)
        fw_ddot = (fw_dot - self.fw_dot_old)/dt
        self.fw_dot_old = fw_ddot


        dp_x_frac_f = self.fracDeriv(self.alpha_x-1.0, np.sign(dp_x)*abs(dp_x)**(self.alpha12), dp_x_dot, dt)
        dp_x_frac_f_dot = self.fracDeriv(self.alpha_x, np.sign(dp_x)*abs(dp_x)**(self.alpha12), dp_x_dot, dt)
        e_lat = dp_y*self.w_y + fw*self.w_psi
        e_lat_dot = dp_y_dot*self.w_y + fw_dot*self.w_psi
        e_frac = self.fracDeriv(self.alpha_y-1.0, np.sign(e_lat)*abs(e_lat)**(self.alpha1), e_lat_dot, dt)
        e_frac_dot = self.fracDeriv(self.alpha_y, np.sign(e_lat)*abs(e_lat)**(self.alpha1), e_lat_dot, dt)

        # Inputs for lateral controller
        
        self.dp_x_old = dp_x
        self.dp_y_old = dp_y
        self.dpsi_old = dpsi
        self.dp_x_dot_old = dp_x_dot
        self.dp_dot_old = np.array([dp_x_dot,dp_y_dot,dpsi_dot, dp_x_dot_abs])

        self.started = True

        # assert not (np.isnan(dp_x_dot) or np.isnan(dp_y_dot) or np.isnan(dpsi_dot)), " lateral_controller 2"

        acc = self.longit_controller(dp_x, dp_x_dot, dp_x_frac_f, dp_x_frac_f_dot)
        # acc = 0.5

        dp_ddx_abs = (abs(dp_x_dot)-abs(self.dp_x_dot_old))/dt
        dp_ddx = (dp_x_dot-self.dp_x_dot_old)/dt
        steering = self.lateral_controller(dp_x, dp_y, dp_x_dot, dp_y_dot, dpsi, dpsi_dot, 
                                                    e_frac,e_frac_dot,
                                                    curr_steering, curr_velocity, dp_ddx, dp_x_dot_abs, 
                                                    fw,fw_dot,fw_ddot, dp_ddx_abs,
                                                    ugv_velocity_x, ugv_w)

            
        return acc, steering