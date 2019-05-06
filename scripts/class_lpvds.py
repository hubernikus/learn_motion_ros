#!/usr/bin/env python2

import numpy as np
import numpy.linalg as LA
from numpy import pi

import yaml


class DynamicalSystem():
    def __init__(self):
        print("Define empty base-constructor")

    def get_ds(self):
        return [0,0]

    def is_attractor_reached(self):
        return False


class LVP_DS(DynamicalSystem):
    def __init__(self,  A_g, b_g, Mu=[], Priors=[], Sigma=[],file_name=""):

        if len(file_name):
            yaml_dict = yaml.load(open('filename'))

            self.Mu = yaml_dict['Mu']
            self.Priors = yaml_dict['Priors']
            self.Sigma = yaml_dict['Sigma']

            self.n_gaussians = yaml_dict['K']
            self.n_dimensions = yaml_dict['M']

            self.starting_point_teaching = yaml_dict['x0_all']
            self.mean_starting_point = np.mean(self.starting_point_teaching, axis=1)

            self.attractor = yaml_dict['attractor']

            # print("TODO -- import lvpDS from file")
        else:
            # TODO - check for values
            self.Mu     = Mu
            self.Priors = Priors
            self.Sigma  = Sigman

            # Auxiliary Variables
            self.n_dimensions = np.array(x).shape[0] # 
            self.n_gaussians = len(self.Priors)

            self.mean_starting_point = np.ones((dim))
            self.attractor = np.zeros((dim))

        # Posterior Probabilities per local DS


    def is_attractor_reached(self, x, margin_attr=0.1):
        return np.sqrt((x-self.attractor)**2) < margin_attr 

    def get_ds(self, x, normalization_type='norm'):
        n_datapoints = np.array(x).shape[1]

        beta_k_x = self.posterior_probs_gmm(x, normalization_type)

        # Output Velocity
        x_dot = np.zeros((N,M))
        for i in range(np.array(x).shape[1]):
            # Estimate Global Dynamics component as LPV
            if np.array(b_g).shape[1] > 1:
                f_g = np.zeros((self.n_datapoings, self.n_gaussians))
                for k in range(self.n_gaussians):
                    f_g[:,k] = beta_k_x[k,i] * (A_g[:,:,k]*x[:,i] + b_g[:,k])

                f_g = np.sum(f_g, axis=1)
            else
                # Estimate Global Dynamics component as Linear DS
                f_g = (A_g*x[:,i] + b_g)

            x_dot[:,i] = f_g          
        return x_dot


    def posterior_probs_gmm(self, x, normalization_type='norm'):
        n_datapoints = np.array(x).shape[1]

        # Compute mixing weights for multiple dynamics
        Px_k = np.zeros((self.n_gaussians, n_datapoints))

        # Compute probabilities p(x^i|k)
        for k in range(self.n_gaussians):
            Px_k[k,:] = self.ml_gaussPDF(x, self.Mu[:,k], self.Sigma[:,:,k]) + eps

        ### Compute posterior probabilities p(k|x) -- FAST WAY --- ###
        alpha_Px_k = np.tile(self.Priors.T, (1, n_datapoints))*Px_k

        if normalization_type=='norm':
            Pk_x = alpha_Px_k / np.tile(np.sum(alpha_Px_k, axis=0), (self.n_gaussians, 1))
        elif normalization_type=='un-norm':
            Pk_x = alpha_Px_k

        return Pk_x


    def ml_gaussPDF(self, Data):
        #ML_GAUSSPDF
        # This def computes the Probability Density Def (PDF) of a
        # multivariate Gaussian represented by means and covariance matrix.
        #
        # Author:	Sylvain Calinon, 2009
        #			http://programming-by-demonstration.org
        #
        # Inputs -----------------------------------------------------------------
        #   o Data:  D x N array representing N datapoints of D dimensions.
        #   o Mu:    D x 1 array representing the centers of the K GMM components.
        #   o Sigma: D x D x 1 array representing the covariance matrices of the 
        #            K GMM components.
        # Outputs ----------------------------------------------------------------
        #   o prob:  1 x N array representing the probabilities for the 
        #            N datapoints.     
        # (nbVar,nbData) = np.array(Data).shape

        #      (D x N) - repmat((D x 1),1,N)
        #      (D x N) - (D x N)
        n_datapoints = np.array(Data).shape[1]

        Mus  = np.tile(self.Mu, (1, n_datapoints))
        Data = (Data - Mus).T

        # Data = (N x D)

        realmin = np.finfo(np.double).tiny

        # (N x 1)
        prob = np.sum((Data*LA.inv(self.Sigma)).*Data, axis=1)
        prob = np.exp(-0.5*prob) / np.sqrt((2*pi)**self.n_dimensions * (np.abs(LA.det(self.Sigma))+realmin)) + realmin

        return prob

