#Random file from my computer, just want to test upload functionality

import numpy as np
import matplotlib.pyplot as plt

#Integrates phi_k*phi_j between -d and d
def integratePhis(j,k, d, dx):
    xPoints = np.linspace(-d,d,int(2*d/dx))
    sum = 0
    for x in xPoints:
        
        if j%2 == 0:
            phiJ = np.sin(j*np.pi*x/2)
        else:
            phiJ = np.cos(j*np.pi*x/2)
        if k%2 == 0:
            phiK = np.sin(k*np.pi*x/2)
        else:
            phiK = np.cos(k*np.pi*x/2)

        sum += phiJ*phiK*dx

    return sum

#Returns the index of the eigenvalue with the smallest magnitude
def indexOfMinimum(eigenvalues):
    index = 0
    magnitude = 1000
    for i in range(len(eigenvalues)):
        if np.abs(eigenvalues[i]) < magnitude:
            magnitude = np.abs(eigenvalues[i])
            index = i
    return index

#Gets the ground state eigenvector
def groundStateEigenvector(d,dx,N,v):
    
    A = np.empty(shape=(N,N))
    
    for j in range(1,N+1):
        for k in range(1,N+1):
            entry = v * integratePhis(j,k,d,dx)
            if j == k:
                entry += ((k*np.pi)**2)/8
            A[j-1][k-1] = entry

    eigenvalues = np.linalg.eig(A)[0]
    eigenvectors = np.linalg.eig(A)[1]
    c = np.matrix.transpose(eigenvectors)[indexOfMinimum(eigenvalues)]
    return c
#-------------------------------------------------------------------------------
N=5
V = [4,6,8,10,12,14]
D= [.2,.3,.4,.5,.6,.7]
L=1
dx = .0001

xPoints = np.linspace(-1,1,int(2*L/dx))

plt.figure(figsize=(7,7))
plt.title(r"$\Psi^2$ with Varying Potential $V_o$") 
plt.xlabel("x") 
plt.ylabel(r"$\Psi^2$")
color = 0
for v in V:
    c=groundStateEigenvector(.5,dx,N,v)
    yPoints = []

    for x in xPoints:
        y = 0
        for k in range(1,N+1):
            if k %2 == 0:
                y+= c[k-1]*np.sin(k*np.pi*x/2)
            else:
                y+= c[k-1]*np.cos(k*np.pi*x/2)
        yPoints.append(np.abs(y)**2)
    plt.plot(xPoints, yPoints, color = (0,0,1-2*color/10))
    color +=1
plt.show()

#Graph varying box width
plt.title(r"$\Psi^2$ with Varying Potential Width $d$") 
color = 0
for d in D:
    c=groundStateEigenvector(d,dx,N,10)
    yPoints = []

    for x in xPoints:
        y = 0
        for k in range(1,N+1):
            if k %2 == 0:
                y+= c[k-1]*np.sin(k*np.pi*x/2)
            else:
                y+= c[k-1]*np.cos(k*np.pi*x/2)
        yPoints.append(np.abs(y)**2)
    plt.plot(xPoints, yPoints, color = (1-color/10,0,1-color/10))
    color +=1

plt.show()