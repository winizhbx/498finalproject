#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

def kalman_xy(x, P, measurement, R,
              motion = np.matrix('0. 0. 0. 0.').T,
              Q = np.matrix(np.eye(4))):
    """
    Parameters:    
    x: initial state 4-tuple of location and velocity: (x0, x1, x0_dot, x1_dot)
    P: initial uncertainty convariance matrix
    measurement: observed position
    R: measurement noise 
    motion: external motion added to state vector x
    Q: motion noise (same shape as P)
    """
    return kalman(x, P, measurement, R, motion, Q,
                  F = np.matrix('''
                      1. 0. 1. 0.;
                      0. 1. 0. 1.;
                      0. 0. 1. 0.;
                      0. 0. 0. 1.
                      '''),
                  H = np.matrix('''
                      1. 0. 0. 0.;
                      0. 1. 0. 0.'''))

def kalman(x0, P0, measurement, R, motion, Q, F, H):
    
    # PREDICT x, P based on motion
    x_pred = F*x0 + motion
    P_pred = F*P0*F.T + Q
    H_dim =  H.shape[0]
    
    # UPDATE x, P based on measurement m    
    # distance between measured and current position-belief
    y = np.matrix(measurement).T - H * x_pred
    S = H * P_pred * H.T + R  # residual convariance
    K = P_pred * H.T * S.I    # Kalman gain
    x_hat = x_pred + K*y.reshape(H_dim, 1)
    I = np.matrix(np.eye(F.shape[0])) # identity matrix
    P_hat = (I - K*H)*P_pred

    #print x_hat, '\n', P_hat, 'y', y

    return x_hat, P_hat

def extend_kalman(x0, P0, measurement, R, motion, Q, F = np.eye(2)): # for distance d = sqrt(x^2 + y^2)
    
    # PREDICT x, P based on motion
    x_pred = np.matmul(F, x0) + motion
    P_pred = np.matmul(np.matmul(F, P0), F.T) + Q
    state_dim = x0.shape[0]

    # UPDATE x, P based on measurement m    
    # distance between measured and current position-belief
    d = np.sqrt(x_pred[0,0]**2+x_pred[1,0]**2)
    H = np.array([x_pred[0,0]/d, x_pred[1,0]/d])
    y = measurement - np.matmul(H, x_pred)
    # print "err", y
    S = np.matmul(np.matmul(H, P_pred), np.transpose(H)) + R # residual convariance
    K = np.matmul(P_pred, np.transpose(H)).reshape(state_dim, 1) * np.linalg.inv(S)    # Kalman gain
    x_hat = x_pred + K*y.reshape(-1, 1)
    I = np.matrix(np.eye(F.shape[0])) # identity matrix
    P_hat = (I - K*H)*P_pred

    # print x_hat, '\n'

    return x_hat, P_hat

def demo_kalman_xy(true_xy=np.array([[0,0]]), KF_init=np.array([[0,0]]), Q=np.zeros((4,4)), R=np.array([[10,0],[0,10]]), N=30, step_mode=0):
    
    print "Regular KF demo."

    x = np.matrix('0. 0. 0. 0.').T 
    P = np.matrix(np.eye(4))*1000 # initial uncertainty

    N = 30
    true_x = [true_xy[0,0]]
    true_y = [true_xy[0,1]]
    observed_x = [true_xy[0,0]]
    observed_y = [true_xy[0,0]]
    kalman_x = [KF_init[0,0]]
    kalman_y = [KF_init[0,1]]
    #R = np.array([[1,0],[0,1]])
    #Q = np.zeros((4, 4))
    Q[2,2] = 0.1
    Q[3,3] = 0.1
    P_list = []
    P_list.append(P)

    for i in range(N):
        motion = np.array([np.random.normal(1, 0.5), np.random.normal(1, 0.5)])
        true_x.append(true_x[i]+motion[0])
        true_y.append(true_y[i]+motion[1])
        observed_x.append(true_x[i+1] + np.random.normal(0, 1))
        observed_y.append(true_y[i+1] + np.random.normal(0, 1))
        meas = [observed_x[i+1], observed_y[i+1]]

        x, P = kalman_xy(x, P, meas, R, motion=np.matrix([0,0,0,0]).T, Q=np.zeros((4,4)))
        kalman_x.append(x[0][0])
        kalman_y.append(x[1][0])
        P_list.append(P)


    if step_mode == 1:
        a = plt.subplot(111)
        for i in range(len(observed_x)):
            #print P_list[i][0,0]
            e = Ellipse((kalman_x[i], kalman_y[i]), np.sqrt(P_list[i][0,0]), np.sqrt(P_list[i][1,1]), 0)
            e.set_alpha(0.1)
            a.add_artist(e)
    plt.plot(observed_x, observed_y, 'ro')
    plt.plot(kalman_x, kalman_y, 'go-')
    plt.plot(true_x, true_y, 'bo-')
    plt.show()

def demo_extend_kalman_xy(true_xy=np.array([[1,1]]), KF_init=np.array([[1,1]]), Q=np.eye(2)*0.5, R=np.array([[1]]), N=30, step_mode=0):

    print "Extended KF demo."

    x = np.array([[1., 1.]]).T 
    P = np.matrix(np.eye(2))*10 # initial uncertainty

    true_x = [true_xy[0,0]]
    true_y = [true_xy[0,1]]
    observed_x = [true_xy[0,0]]
    observed_y = [true_xy[0,0]]
    kalman_x = [KF_init[0,0]]
    kalman_y = [KF_init[0,1]]
    #R = np.array([[1]])
    #Q = np.eye(2)
    #Q[0,0] = 0.5
    #Q[1,1] = 0.5
    P_list = []
    P_list.append(P)
    # print Q

    for i in range(N):
        motion = np.array([np.random.normal(1, 0.5), np.random.normal(1, 0.5)])
        true_x.append(true_x[i]+motion[0])
        true_y.append(true_y[i]+motion[1])
        observed_x.append(true_x[i+1] + np.random.normal(0, 0.1))
        observed_y.append(true_y[i+1] + np.random.normal(0, 0.5))
        meas = np.sqrt(observed_x[i+1]**2 + observed_y[i+1]**2)

        x, P = extend_kalman(x, P, meas, R, motion=np.array([[motion[0],motion[1]]]).T, Q=Q)
        kalman_x.append(x[0][0])
        kalman_y.append(x[1][0])
        P_list.append(P)
        # print "true", true_x[i+1], true_y[i+1]
    
    if step_mode == 1:
        a = plt.subplot(111)
        for i in range(len(observed_x)):
            #print P_list[i][0,0]
            e = Ellipse((kalman_x[i], kalman_y[i]), np.sqrt(P_list[i][0,0]), np.sqrt(P_list[i][1,1]), 0)
            e.set_alpha(0.1)
            a.add_artist(e)
    plt.plot(observed_x, observed_y, 'ro')
    plt.plot(kalman_x, kalman_y, 'go-')
    plt.plot(true_x, true_y, 'bo-')
    plt.show()

#demo_kalman_xy()
#demo_extend_kalman_xy()