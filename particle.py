import numpy as np
import random_numbers as rn


class Particle(object):
    """Data structure for storing particle information (state and weight)"""
    def __init__(self, x=0.0, y=0.0, theta=0.0, weight=0.0):
        self.x = x
        self.y = y
        self.theta = np.mod(theta, 2.0*np.pi)
        self.weight = weight

    def getX(self):
        return self.x
        
    def getY(self):
        return self.y
        
    def getTheta(self):
        return self.theta
        
    def getWeight(self):
        return self.weight

    def setX(self, val):
        self.x = val

    def setY(self, val):
        self.y = val

    def setTheta(self, val):
        self.theta = np.mod(val, 2.0*np.pi)

    def setWeight(self, val):
        self.weight = val


def estimate_pose(particles_list):
    """Estimate the pose from particles by computing the average position and orientation over all particles. 
    This is not done using the particle weights, but just the sample distribution."""
    x_sum = 0.0
    y_sum = 0.0
    cos_sum = 0.0
    sin_sum = 0.0
     
    for particle in particles_list:
        x_sum += particle.getX()
        y_sum += particle.getY()
        cos_sum += np.cos(particle.getTheta())
        sin_sum += np.sin(particle.getTheta())
        
    flen = len(particles_list)
    if flen != 0:
        x = x_sum / flen
        y = y_sum / flen
        theta = np.arctan2(sin_sum/flen, cos_sum/flen)
    else:
        x = x_sum
        y = y_sum
        theta = 0.0
        
    return Particle(x, y, theta)
     
     
def move_particle(particle: Particle, delta_x, delta_y, delta_theta):
    particle.setX(particle.getX() + delta_x)
    particle.setY(particle.getY() + delta_y)
    particle.setTheta(particle.getTheta() + delta_theta)
    add_uncertainty_single(particle, 2, 0.1)

def turn(particle: Particle, delta_theta):
    particle.setTheta(particle.getTheta() + delta_theta)
    add_uncertainty_single(particle, 0, 0.1)
    
def forward(particle: Particle, delta_x, delta_y):
    particle.setX(particle.getX() + delta_x)
    particle.setY(particle.getY() + delta_y)
    add_uncertainty_single(particle, 2, 0)

def move_particles(particles_list: list[Particle], move_delta, pd_noise=[3 , 0.1]):
    for particle in particles_list:
        particle.setX(particle.getX() + move_delta[0])
        particle.setY(particle.getY() + move_delta[1])
        particle.setTheta(particle.getTheta() + move_delta[2])
        particle.x += rn.randn(0.0, pd_noise[0])
        particle.y += rn.randn(0.0, pd_noise[0])
        particle.theta = np.mod(particle.theta + rn.randn(0.0, pd_noise[1]), 2.0 * np.pi) 
    
    
def noise(particles_list, pd_noise=[3 , 0.1]):#pd_noise=[1 , 0.035]
    # vi tilføjer kun lille smule noise
    add_uncertainty(particles_list, pd_noise[0], pd_noise[1])
    
def add_uncertainty_single(particle, sigma, sigma_theta):
    """Add some noise to each particle in the list. Sigma and sigma_theta is the noise
    variances for position and angle noise."""
    particle.x += rn.randn(0.0, sigma)
    particle.y += rn.randn(0.0, sigma)
    particle.theta = np.mod(particle.theta + rn.randn(0.0, sigma_theta), 2.0 * np.pi) 

def add_uncertainty_von_mises_single(particle, sigma, theta_kappa):
    """Add some noise to each particle in the list. Sigma and theta_kappa is the noise
    variances for position and angle noise."""
    particle.x += rn.randn(0.0, sigma)
    particle.y += rn.randn(0.0, sigma)
    particle.theta = np.mod(rn.rand_von_mises(particle.theta, theta_kappa), 2.0 * np.pi) - np.pi
    
def add_uncertainty(particles_list, sigma, sigma_theta):
    """Add some noise to each particle in the list. Sigma and sigma_theta is the noise
    variances for position and angle noise."""
    for particle in particles_list:
        particle.x += rn.randn(0.0, sigma)
        particle.y += rn.randn(0.0, sigma)
        particle.theta = np.mod(particle.theta + rn.randn(0.0, sigma_theta), 2.0 * np.pi) 


def add_uncertainty_von_mises(particles_list, sigma, theta_kappa):
    """Add some noise to each particle in the list. Sigma and theta_kappa is the noise
    variances for position and angle noise."""
    for particle in particles_list:
        particle.x += rn.randn(0.0, sigma)
        particle.y += rn.randn(0.0, sigma)
        particle.theta = np.mod(rn.rand_von_mises(particle.theta, theta_kappa), 2.0 * np.pi) - np.pi
        
est_pos = None
est_pos_old = None
est_dir = None
est_dir_old = None
est_Count = 0
def accepltable_robot_pos_estimate(particles_list):
    pos_diff_lim = 3
    pos_var_lim = 10
    dir_diff_lim = 0.1
    est_Count_lim = 10
    global est_pos
    global est_pos_old
    global est_dir
    global est_dir_old
    global est_Count 
    pose = estimate_pose(particles_list)
    est_pos = np.array([pose.getX(), pose.getY()])
    est_dir = pose.getTheta()
    if (est_pos_old is None) or (est_dir_old is None): 
        est_pos_old = est_pos
        est_dir_old = est_dir
        return (False, 600)
    else:
        particle_dist = []
        for p in particles_list: 
            particle_dist.append(np.linalg.norm(np.array([p.getX(), p.getY()]) - est_pos))
        pos_var = np.var(particle_dist)
        pos_diff = abs(est_pos - est_pos_old)
        dir_diff = abs(est_dir - est_dir_old)
        
        if est_Count == est_Count_lim: est_Count = 0 
        if pos_diff[0] < pos_diff_lim and pos_diff[1] < pos_diff_lim:
            est_Count += 1
        else:
            est_Count = 0
        
        print(f"pos_var = {pos_var}")
        print(f"pos_diff =  = {pos_diff}")
        print(f"dir_diff = {dir_diff}")
        print(f"est_Count= {est_Count}")
        
        result = pos_var < pos_var_lim and dir_diff < dir_diff_lim and est_Count == est_Count_lim
        
        est_pos_old = est_pos
        est_dir_old = est_dir
        return (result, pos_var)