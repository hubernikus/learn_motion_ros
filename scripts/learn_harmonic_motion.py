'''
Learning motion 

@author lukashuber
@date 2019-04-09

// This algorithm is based on paper 
TITLE Progressive Automation of Periodic Movements>>
AUTHOR Fotios Dimeas, Theodora Kastritsi, Dimitris Papageorgiou, and Zoe Doulgeri
YEAR 2019
CONFERENCE IROS

'''
import numpy as np
import numpy.linalg as LA

import matplotlib.pyplot as plt

class Harmonic_Motion_Learner():
    #scale_force = lambda_1
    # power_force = 3
    # lambda_2 = 1 # lambda_2
    # power_offest = 1
    # minimum_gain = 0.1
    # gain_scaling = 1
    
    def __init__(self, lambda_1=1, power_force=3, lambda_2=1, power_offest=1, minimum_gain=0.1, gain_scaling=1):
        dim=2 # x & y? == mj
        # TODO extend for joint space
        self.lambda_1 = lambda_1
        self.power_force = power_force
        self.lambda_2 = lambda_2
        self.power_offest = power_offest
        self.minimum_gain = minimum_gain
        self.gain_scaling = gain_scaling
        
        # stiffness = np.array()

        self._automation_level = np.zeros(dim) # (3)
        self._freqs = np.zeros(dim)
        self._phases = np.zeros(dim)

        self.n_fourier = 300
        
        self.amplitudes = np.zeros(2, dim, self.n_fourier) # a & b

        self.dt = 0.1
        
        self.force_input = np.zeros(dim, self.n_fourier)
        self.ft_force = np.zeros(dim, self.n_fourier)
        self.position = np.zeros(dim, self.n_fourier)
        self.ft_position = np.zeros(dim, self.n_fourier)

        self.velocity = 0
        self.acceleration = 0

        # Test
        self.freq_margin = 0.01
        self.Omega =  np.zeros(dim, self.n_fourier)

        # self.g = 
        self.p_d = np.zeros(dim)
        self.weights = np.zeros(dim) # w
        # self.z = np.zeros(dim)
        self.z = 0
        self.Phi = 0 # (11)
        
        self.n_basis = 10
        self.Psi = np.zeros(self.n_basis)
        self.Psi_sum = 0
        
        self.width_basis = np.zeros(self.n_basis)
        self.cent_basis = np.zeros(self.n_basis)
        

        self.alpha_y = 1
        self.beta_y = 1
        self.r = 1 # Amplitude control parameter

        self.P = np.zeros(m,m)

        self.lambda_0 = 2 # Forgetting factor

        self.g = np.zeros(dim) # anchor point

    def evaluate_basis_function(self):
        self.Psi = np.exp(self.width_basis*(np.cos(self.Phi-self.cent_basis )))

    def load(self):
        print('Loading')
    
    def update(self, interaction_force=0, tracking_error=0, a_coupling=1):
        dt_automation_level = (_automation_level/gain_scaling + minimum_gain) * (1 - LA.norm(interaction_force)**power_force - LA.norm(tracking_error)/lambda_2) # (4)

        if _automation_level<=0: # (3)
            dt_automation_level = np.max([dt_automation_level, 0])
        elif _automation_level>=1:
            dt_automation_level = np.min([dt_automation_level, 0])

        err_vec = position-tracking_error # (7)
        Err_matr = np.diag(ee_vec) # (7)
        dt_freqs = (1-automation_level)*(self.freqs-a_coupling*Err_matr*np.sin(self.freqs)) # (5)
        dt_phases = -(1-automation_level)*a_coupling*Err_matr*np.sin(self.freqs) # (7)

        c_vals = np.arange(self.n_fourier)

        p_hat = np.sum(self.amplitudes[0,:,:]*np.tile(np.cos(c_vals*self.freqs), (dim, 1)) +
                       self.amplitudes[0,:,:]*np.tile(np.sin(c_vals*self.freqs), (dim, 1)), axis=2) # (8)

        self.ft_force = np.rfft(self.force_input, axis=1)
        omega_ind =  np.where(self.ft_force > self.freq_margin)
        if omega_ind.shape[0]:
            omega_ind[0]
        else:
            omega_ind = 0
        self.Omega = 1./(self.dt*self.n_fourier)*omega_ind # (1)
        
        dt_amplitudes = np.zeros(2, dim, self.n_fourier)
        for dd in range(dim):
            dt_amplitudes[0, dd, :] = (1-self._automation_level)*self.learning_constant*np.cos(c_vals*self._freqs)*err_vec # (9)
            dt_amplitudes[1, dd, :] = (1-self._automation_level)*self.learning_constant*np.sin(c_vals*self._freqs)*err_vec # (10)

        dt_phi = self.Omega # (11)
        
        self.evaluate_basis_function()
        dt_z = self.Omega*(self.a_y*(self.beta_y*(g-self.p_d)-self.z)+self.r*np.sum(self.w*self.Psi)/np.sum(self.Psi)) # (12)
        dt_p_d = self.Omega*self.z # (13)
        
        
        f_d = self.acceleartion/self.Omega**2 - self.alpah_y*(self.beta_y*(g-self.position[:,0])-self.velocity/self.Omega) # (18)
        e_r = (1-self._automation_level)*(f_d - self.weights) # (17)

        for ii in range(dim): # TODO Remove double loop for speed
            for jj in range(dim):
                self.P[ii,jj] = 1/self.lambda_0 * (self.P[ii,jj]-self.P[ii,jj]**2/(self.lambda_0/self.Psi[jj]+self.P[ii,jj])) # (16)

        # TODO update P before!
        self.weights += Psi * np.diag(P)*e_r # (15)
        
        # Integrate
        self._automation_level += self.dt*dt_automation_level
        self._freqs += self.dt*dt_freqs
        self._phases += self.dt*dt_phases
        self._z += self.dt*dt_z

        self._amplitudes += self.dt*dt_amplitudes
        self.phi += self.dt*dt_phi

        # Get position / acceleration
        # new_vel = 


