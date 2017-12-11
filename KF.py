#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt

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
    '''
    Parameters:
    x: initial state
    P: initial uncertainty convariance matrix
    measurement: observed position (same shape as H*x)
    R: measurement noise (same shape as H)
    motion: external motion added to state vector x
    Q: motion noise (same shape as P)
    F: next state function: x_prime = F*x
    H: measurement function: position = H*x

    Return: the updated and predicted new values for (x, P)

    See also http://en.wikipedia.org/wiki/Kalman_filter

    This version of kalman can be applied to many different situations by
    appropriately defining F and H 
    '''
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

    #print x, '\n', P, 'y', y

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
    print "err", y
    S = np.matmul(np.matmul(H, P_pred), np.transpose(H)) + R # residual convariance
    K = np.matmul(P_pred, np.transpose(H)).reshape(state_dim, 1) * np.linalg.inv(S)    # Kalman gain
    x_hat = x_pred + K*y.reshape(-1, 1)
    I = np.matrix(np.eye(F.shape[0])) # identity matrix
    P_hat = (I - K*H)*P_pred

    print x_hat, '\n'

    return x_hat, P_hat

def demo_kalman_xy():
    x = np.matrix('0. 0. 0. 0.').T 
    P = np.matrix(np.eye(4))*1000 # initial uncertainty

    N = 40
    true_x = [0]
    true_y = [0]
    observed_x = [0]
    observed_y = [0]
    kalman_x = [0]
    kalman_y = [0]
    R = np.array([[1,0],[0,1]])
    Q = np.zeros((4, 4))
    Q[2,2] = 0.09
    Q[3,3] = 0.09

    '''
    '''
    for i in range(N):
        motion = np.array([np.random.normal(2, 0.3), np.random.normal(1, 0.3)])
        true_x.append(true_x[i]+motion[0])
        true_y.append(true_y[i]+motion[1])
        observed_x.append(true_x[i+1] + np.random.normal(0, 1))
        observed_y.append(true_y[i+1] + np.random.normal(0, 1))
        meas = [observed_x[i+1], observed_y[i+1]]

        x, P = kalman_xy(x, P, meas, R, motion=np.matrix([0,0,0,0]).T, Q=np.zeros((4,4)))
        kalman_x.append(x[0][0])
        kalman_y.append(x[1][0])
    plt.plot(observed_x, observed_y, 'ro')
    plt.plot(kalman_x, kalman_y, 'go-')
    plt.plot(true_x, true_y, 'bo-')
    plt.show()

def demo_extend_kalman_xy():
    x = np.array([[1., 1.]]).T 
    P = np.matrix(np.eye(2))*1000 # initial uncertainty

    N = 44
    true_x = [1]
    true_y = [1]
    observed_x = [1]
    observed_y = [1]
    kalman_x = [1]
    kalman_y = [1]
    R = np.array([[0.01]])
    Q = np.zeros((2, 2))*0.01
    print Q

    for i in range(N):
        motion = np.array([np.random.normal(1, 0.1), np.random.normal(0.5, 0.1)])
        true_x.append(true_x[i]+motion[0])
        true_y.append(true_y[i]+motion[1])
        observed_x.append(true_x[i+1] + np.random.normal(0, 0.1))
        observed_y.append(true_y[i+1] + np.random.normal(0, 0.1))
        meas = np.sqrt(observed_x[i+1]**2 + observed_y[i+1]**2)

        x, P = extend_kalman(x, P, meas, R, motion=np.array([[motion[0],motion[1]]]).T, Q=Q)
        kalman_x.append(x[0][0])
        kalman_y.append(x[1][0])
        print "true", true_x[i+1], true_y[i+1]
    plt.plot(observed_x, observed_y, 'ro')
    plt.plot(kalman_x, kalman_y, 'go-')
    plt.plot(true_x, true_y, 'bo-')
    plt.show()
#demo_kalman_xy()
demo_extend_kalman_xy()