#!/usr/bin/python

import rospy
import numpy as np
import pickle as pk
from scipy import signal
from Rasia_Prototyp.msg import zeros
from sklearn.preprocessing import StandardScaler
from std_msgs.msg import String


def averaged_stft_matrix(data, m, nwf, nwt):

    u = data[m, :]
    t, f, z = signal.stft(u, nperseg=2*np.round(np.sqrt(len(u))))
    y = np.abs(z)
    nf = len(f)  # ilosc okien czestotliwosciowych przed usrednieniem
    nt = len(t)  # ilosc okien czasowych przed usrednieniem
    df = int(nf/nwf)  # zadana dlugosc okna czestotliwosciowego
    dt = int(nt/nwt)  # zadana dlugosc okna czasowego
    A = np.zeros((nwf, nwt))
    for i in range(nwf):
        for j in range(nwt):
            A[i, j] = np.mean(y[i*df:(i+1)*df, j*dt:(j+1)*dt])
    return A


def stft_features( data, nwf, nwt):

    F = np.zeros(200)
    for m in range(8):  # petla po 8 kanalach EMG
        F[m*25:(m+1)*25] = np.ndarray.flatten(averaged_stft_matrix(data, m, nwf, nwt))
    return F


	
def talker(xPCA):
	pub = rospy.Publisher('STFTtalker', String)
    rate = rospy.Rate(10) # 10 msgs/sec
    while not rospy.is_shutdown():
		stft_str = xPCA	# tu trzeba zamienic na tablice 1x131
		rospy.loginfo(stft_str)
		rate.sleep()	

def calcFeatures(data):
    # cechy stft przed PCA
    xRaw = stft_features(data, 5, 5)
    # Normalizacja
    x_Arr = np.vstack(xRaw)
    x_Norm = StandardScaler().fit_transform(x_Arr)
    pca = pk.load(open("pca.pkl", 'rb'))
    xPCA = pca.transform(x_Norm).reshape(-1)
    #return xPCA
    try:
		talker(xPCA)
	except eospy.ROSInterruptException:
		pass
    

def listener():
	rospy.init_node("STFT_NODE")
	rospy.Subscriber("ADCtalker", String, calcFeatures)
	rospy.spin()
	# wpisac do odpowiedniej macierzy
	

if __name__ == '__main__':
    print "Start STFT node"
    listener()
    #!rospy.init_node("STFT_NODE")
    #!rospy.Subscriber("ADC_NODE", zeros, calcFeatures)

    #!rospy.spin()
    

