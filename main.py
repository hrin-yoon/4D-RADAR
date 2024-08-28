import numpy as np
import os


path = f"/home/zeta/Can_do/test_data/sprdr_00038.npy"
npy =  np.load(path)

total_len = len(npy)
count =  0

for i in npy:
    print(i)
    count  += 1
    print(count)