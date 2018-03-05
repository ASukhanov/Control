__version__ = 'v01 2018-03-04' # spring-system using TF, pid_spring_mass accepts mbk 0,0,k and pid 0,i,0

import numpy as np
from scipy import signal

def spring_mass_system_ss(mbk):
    '''state space representation'''
    m,b,k = mbk
    ssA = [[0., 1.], [-k/m, -b/m]]
    ssB = [[0.], [1./m]]
    ssC = [[1., 0.]]
    ssD = 0.
    return signal.lti(ssA,ssB,ssC,ssD)

def spring_mass_system(mbk):
    '''transfer function representation'''
    m,b,k = mbk
    #print('mbk:'+str(mbk))
    #if m == 0. and  b == 0.:
    #    return signal.lti([1.], k)
    return signal.lti([1.], mbk)

def spring_mass_zeta(mbk):
    return mbk[1]/2./np.sqrt(mbk[0]*mbk[2])

def spring_mass_omega(mbk):
    return np.sqrt(mbk[2]/mbk[0])

def spring_mass_zpk(mbk):
    return signal.tf2zpk([1.],[m,b,k])

def pid_spring_mass(mbk,pid):
    if pid[1] == 0.:
        controller_type = 'P/PD'
        num = [pid[2], pid[0]]
        den = [mbk[0], mbk[1] + pid[2], mbk[2] + pid[0]]
    elif pid[2] == 0.:
            controller_type = 'PI/I'
            num = [pid[0], pid[1]] if pid[0] else [pid[1]]
            den = [mbk[0], mbk[1], mbk[2] + pid[0], pid[1]]
    else:
        controller_type = 'PID'
        num = [pid[2], pid[0], pid[1]]
        den = [mbk[0], mbk[1] + pid[2], mbk[2] + pid[0], pid[1]]
    print('controller:'+controller_type+', zeros,poles,gain:\n '+str(signal.tf2zpk(num,den)))
    return signal.lti(num,den)


