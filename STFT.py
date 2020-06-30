#!/usr/bin/python3.7

import rospy
import numpy as np
import pickle as pk
from scipy import signal
from std_msgs.msg import String


def averaged_stft_matrix(data, m, nwf, nwt):

    u = data[m, :]
    t, f, z = signal.stft(u, nperseg=2*np.round(np.sqrt(len(u))))
    y = np.abs(z)
    nf = len(f)  # number of frequency windows before calculating averages
    nt = len(t)  # number of time windows before calculating averages
    df = int(nf/nwf)  # new (set) length of frequency window (in samples)
    dt = int(nt/nwt)  # new (set) length of time window (in samples)
    A = np.zeros((nwf, nwt))
    for i in range(nwf):
        for j in range(nwt):
            A[i, j] = np.mean(y[i*df:(i+1)*df, j*dt:(j+1)*dt])
    return A


def stft_features(data, nwf, nwt):

    F = np.zeros(200)
    for m in range(8):  # 8 EMG channels
        F[m*25:(m+1)*25] = np.ndarray.flatten(averaged_stft_matrix(data, m, nwf, nwt))
    return F


def talker(xPCA):
    pub = rospy.Publisher('STFTtalker', String, queue_size=1)
    rate = rospy.Rate(10)  # 10 msgs/sec
    msglst = []
    for a in range(xPCA.shape[1]):
        msglst.append(repr(xPCA[0, a])+' ')
    msg = ''.join(msglst)
    pub.publish(msg)
    rate.sleep()


def calcFeatures(data):
    dataS = data.data
    dataTab = np.zeros((8, 1500))
    dataTab = np.fromstring(bytes(dataS, 'utf-8'), dtype=np.float64, sep=' ').reshape(dataTab.shape)

    # STFT features
    xRaw = stft_features(dataTab, 5, 5)
    # Scaling and PCA
    # !Remember to set the correct absolute path to imported .pkl files!
    scaler = pk.load(open("/home/pi/catkin_ws/src/ROSbotHand/scaler.pkl", 'rb'))
    x_Norm = scaler.transform(xRaw.reshape(1, -1))
    pca = pk.load(open("/home/pi/catkin_ws/src/ROSbotHand/pca.pkl", 'rb'))
    xPCA = pca.transform(x_Norm)
    try:
        talker(xPCA)
    except rospy.ROSInterruptException:
        pass


def listener():
    rospy.init_node("STFT_node")
    rospy.Subscriber("ADCtalker", String, calcFeatures)
    rospy.spin()


if __name__ == '__main__':
    print("Start ADC node")
    listener()
    


