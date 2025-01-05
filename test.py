import numpy as np
from matplotlib import pyplot as plt

def traj_poly2(s0,stf,sd0,sdtf,sdd0,sddtf,t0,tf,step=1):
    #arranging time
    t = np.arange(t0,tf+step,step)

    #solving for equation
    coef = np.zeros((6,1)) #we are looking for this
    param = np.asarray([[s0],[stf],[sd0],[sdtf],[sdd0],[sddtf]])
    mat = np.asarray([
        [t0**5,t0**4,t0**3,t0**2,t0,1],
        [tf**5,tf**4,tf**3,tf**2,tf,1],
        [5*t0**4,4*t0**3,3*t0**2,2*t0,1,0],
        [5*tf**4,4*tf**3,3*tf**2,2*tf,1,0],
        [20*t0**3,12*t0**2,6*t0,2,0,0],
        [20*tf**3,12*tf**2,6*tf,2,0,0]
        ])
    mat_i = np.linalg.inv(mat) #inverse
    coef = np.matmul(mat_i,param) #acquiring A B C D E F

    #using equation
    zeros = np.zeros(t.shape)
    ones = np.ones(t.shape)
    twos = ones*2
    mat = np.asarray([ #the original equation
        [(t)**5,(t)**4,(t)**3,(t)**2,(t),ones],
        [5*(t)**4,4*(t)**3,3*(t)**2,2*(t),ones,zeros],
        [20*(t)**3,12*(t)**2,6*(t),twos,zeros,zeros]
    ])
    coef_tensor=(np.repeat(coef,t.size,axis=1))
    coef_tensor=np.reshape(coef_tensor,(coef_tensor.shape[0],1,coef_tensor.shape[1]))
    # d = np.tensordot(mat,coef_tensor,axes=[1, 0]).diagonal(axis1=1, axis2=3) #alternative way
    res = np.einsum('mnr,ndr->mdr', mat, coef_tensor)

    time  = t
    possi = res[0,0,:]
    speed = res[1,0,:]
    accel = res[2,0,:]

    return (time,possi,speed,accel)


#Call function
y = traj_poly2(40,0,0,0,0,0,0,10,0.1)
plt.subplot(3,2,1)
plt.plot(y[0],y[1],'r')
plt.title('Position')
plt.subplot(3,2,3)
plt.plot(y[0],y[2],'g')
plt.title('Speed')
plt.subplot(3,2,5)
plt.plot(y[0],y[3],'b')
plt.title('Acceleration')

y = traj_poly2(40,0,0,0,0,0,10,20,0.1)
plt.subplot(3,2,2)
plt.plot(y[0],y[1],'r')
plt.title('Position delayed')
plt.subplot(3,2,4)
plt.plot(y[0],y[2],'g')
plt.title('Speed delayed')
plt.subplot(3,2,6)
plt.plot(y[0],y[3],'b')
plt.title('Acceleration delayed')