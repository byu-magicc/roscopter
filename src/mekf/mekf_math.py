# Name: Jerel Nielsen
# Desc: Algebra for "Derivation of the Relative Multiplicative Kalman Filter" by David Wheeler and Daniel Koch
# Date: 22 Mar 2017

from sympy import *

# define variables
pn = Symbol('pn')
pe = Symbol('pe')
pd = Symbol('pd')
qx = Symbol('qx')
qy = Symbol('qy')
qz = Symbol('qz')
qw = Symbol('qw')
u = Symbol('u')
v = Symbol('v')
w = Symbol('w')
p = Symbol('p')
q = Symbol('q')
r = Symbol('r')
p_hat = Symbol('p_hat')
q_hat = Symbol('q_hat')
r_hat = Symbol('r_hat')
ax_hat = Symbol('ax_hat')
ay_hat = Symbol('ay_hat')
az_hat = Symbol('az_hat')
bgx = Symbol('bgx')
bgy = Symbol('bgy')
bgz = Symbol('bgz')
bax = Symbol('bax')
bay = Symbol('bay')
baz = Symbol('baz')
ygx = Symbol('ygx')
ygy = Symbol('ygy')
ygz = Symbol('ygz')
yax = Symbol('yax')
yay = Symbol('yay')
yaz = Symbol('yaz')
G = Symbol('G')
rho = Symbol('rho')
phi = Symbol('phi')
theta = Symbol('theta')
psi = Symbol('psi')
sigma_ax2 = Symbol('sigma_ax2')
sigma_ay2 = Symbol('sigma_ay2')
sigma_az2 = Symbol('sigma_az2')

# define vectors (b is for bold)
pb = Matrix([[pn], [pe], [pd]]) # position
qb = Matrix([[qx], [qy], [qz], [qw]]) # body orientation
vb = Matrix([[u], [v], [w]]) # body velocity
omegab = Matrix([[p], [q], [r]]) # body angular rates
Gb = Matrix([[0], [0], [G]]) # gravity
I3 = Matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
Pz = Matrix([[0, 0, 0], [0, 0, 0], [0, 0, 1]])
ygb = Matrix([[ygx], [ygy], [ygz]]) # gyro measurement
yab = Matrix([[yax], [yay], [yaz]]) # accelerometer measurement
omegab_hat = Matrix([[p_hat], [q_hat], [r_hat]]) # gyro best estimate
dvb_hat = Matrix([[ax_hat], [ay_hat], [az_hat]]) # accel best estimate
k = Matrix([[0], [0], [1]])
beta_a = Matrix([[bax], [bay], [baz]])

'''================================================================='''
# functions
'''================================================================='''

# create skew symmetric matrix from vector eq. 5
def skew(a):
    return Matrix([[      0, -a[2,0],  a[1,0]], 
                   [ a[2,0],       0, -a[0,0]], 
                   [-a[1,0],  a[0,0],       0]])

# get complex portion of quaternion eq. 3
def quatBar(q):
    return q[0:3,0]

# quaternion multiplication eq. 4
def quatMul(p, q):
    # unpack elements of p
    px = p[0,0]
    py = p[1,0]
    pz = p[2,0]
    pw = p[3,0]

    # perform multiplication
    P = Matrix([[ pw, -pz,  py,  px],
                [ pz,  pw, -px,  py],
                [-py,  px,  pw,  pz],
                [-px, -py, -pz,  pw]])

    return P*q

# create 3x3 rotation matrix from quaternion
def quatRot(q):
    # unpack the quaternion
    qbar = quatBar(q)
    qw = q[3,0]

    # compute rotation
    return (2*qw**2 - 1)*I3 - 2*qw*skew(qbar) + 2*qbar*qbar.T

'''================================================================='''
'''================================================================='''
'''================================================================='''

# compute some additional matrices and vectors
Rqb = quatRot(qb)

# eq. 105
omegab_hatq = Matrix([[omegab_hat[0,0]], [omegab_hat[1,0]], [omegab_hat[2,0]], [0]])
f = zeros(16,1)
f[0:3,:] = Rqb.T*vb
f[3:7,:] = 0.5*quatMul(qb,omegab_hatq)
f[7:10,:] = skew(vb)*omegab_hat + Rqb*Gb + k*k.T*dvb_hat

# eq. 107
F = zeros(15,15)
F[0:3,3:6] = -Rqb.T*skew(vb)
F[0:3,6:9] = Rqb.T
F[3:6,3:6] = -skew(omegab_hat)
F[3:6,9:12] = -I3
F[6:9,3:6] = skew(Rqb*Gb)
F[6:9,6:9] = -skew(omegab_hat)
F[6:9,9:12] = -skew(vb)
F[6:9,12:15] = -k*k.T

# eq. 108
G = zeros(15,6)
G[3:6,0:3] = -I3
G[6:9,0:3] = -skew(vb)
G[6:9,3:6] = -k*k.T

# eq. 115
H_acc = zeros(3,15)
H_acc[:,6:9] = -skew(omegab_hat)
H_acc[:,9:12] = -skew(vb)
H_acc[:,12:15] = I3

# eq. 118
J_acc = zeros(3,6)
J_acc[:,0:3] = -skew(vb)
J_acc[:,3:6] = I3

# eq. 114
h_hat_acc = skew(vb)*omegab_hat - yaz*k + beta_a

# to display this using ascii-art, open terminal and run "isympy" to start the
# IPython console using Sympy. Then run "run filename.py"

print('\nf = '), print(simplify(f))
print('\nF = '), print(simplify(F))
print('\nG = '), print(simplify(G))
print('\nH_acc = '), print(simplify(H_acc))
print('\nJ_acc = '), print(simplify(J_acc))
print('\nh_hat_acc = '), print(simplify(h_hat_acc))
