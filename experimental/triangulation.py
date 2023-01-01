import numpy as np

X1 = np.array([0, 0, 5, 1]).T
X2 = np.array([-2, 0, 5, 1]).T

T21 = np.array([[1, 0, 0, -2],
                [0, 1, 0, 0],
                [0, 0, 1, 0]])

f = 2
K = np.array([[f, 0, 0],
              [0, f, 0],
              [0, 0, 1]])

P1 = np.zeros((3, 4))
P1[:3, :3] = np.eye(3)
P1 = K @ P1
P2 = K @ T21

print('P1')
print(P1)

print('P2')
print(P2)

x1_hat = P1 @ X1
x1 = x1_hat / x1_hat[-1]
x2_hat = P2 @ X1
x2 = x2_hat / x2_hat[-1]

A = np.zeros((4, 4))
A[0] = x1[1] * P1[2] - P1[1]
A[1] = P1[0] - x1[0] * P1[2]
A[2] = x2[1] * P2[2] - P2[1]
A[3] = P2[0] - x2[0] * P2[2]

print(A)

u, s, vh = np.linalg.svd(A, full_matrices=False)

s_diag = np.eye(len(s))
for i in range(4):
  s_diag[i] = s_diag[i] * s[i]

# print('u')
# print(u)
# print(u.shape)

# print('s')
# print(s)
# print(s.shape)

# print('s_diag')
# print(s_diag)
# print(s_diag.shape)

print('vh')
print(vh)
print(vh.shape)

print(u @ s_diag @ vh.T)

X = vh[-1, :]
X = X / X[-1]
print('X')
print(X)