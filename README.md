# ROSbotHand
ROS package to control cybernetic hand developed in Wroclaw University of Science and Technology, Faculty of Electronics in the field of Robotics.

Current version works on Raspberry Pi 4 B
with Raspbian with kernel 4.19
and ROS distribution: melodic
and uses 2.7 and 3.7 versions of Python
-- any changes in the enviroment may require
   minor changes in dependencies and configuration

## To work properly, the package needs kNN classifier model, scaler model, and PCA feature selection model, imported from pickle files:

classifier: modelPZEZMS.pkl
scaler:     scaler.pkl
PCA:        pca.pkl

These files can be generated using script from repository at:
https://github.com/Diarmuid1812/RMclassifHand

NOTE: in case of EOF error while loading pickle files, generating the models directly on Raspberry Pi
      should resolve the problem.

# Package uses 8-Channel 12-Bit ADC for Raspberry Pi Hat, made by Seeed

product at producer's site:
https://wiki.seeedstudio.com/8-Channel_12-Bit_ADC_for_Raspberry_Pi-STM32F030/

# Todo-list
TODO: restructure prototype,
      add proper messaging -- communication by arrays instead of String,
      delete unused parts,
      add moveset for all move classes,

