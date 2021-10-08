import matplotlib.pyplot as plt
import sys,os
import numpy as np
sys.path.append(os.path.realpath('../AI3603_HW1'))

import newton

def test_newton():
    x = np.array([16,23,37])
    y = np.array([17,32,37])
    # get the divided difference coef
    a_s = newton.divided_diff(x, y)[0, :]
    
    # evaluate on new data points
    x_new = np.arange(16, 40, .1)
    y_new = newton.newton_poly(a_s, x, x_new)


    plt.style.use('seaborn-poster')
    plt.figure(figsize = (12, 8))
    plt.plot(x, y, 'bo')
    plt.plot(x_new, y_new)
    plt.savefig('out/test_newton.png')

if __name__ == '__main__':
    test_newton()
