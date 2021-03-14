
import numpy as np

tst_dict = {
    'left_arm.shoulder_pitch': [1,2,3], 
    'left_arm.shoulder_roll': [10,11,12], 
    'left_arm.arm_yaw': [20, 21, 22], 
}

pp = [ [1,10,20], [2, 11, 21], [3,12,22] ]

Y = np.array(list(tst_dict.values())).T

print(Y)
print('\n', Y[0])

print('\n', np.array(pp))
print('\n', pp[0])