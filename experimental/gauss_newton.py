import numpy as np

def func(x):
  assert len(x) == 5

  return x[0] ** 2 + 2 * x[1] + 3 * x[2]**3 + 3 * x[3] + x[4]

# True value (to be estimated)
x_gt = np.random.rand(5, 1) * 10

x_noisy = x_gt + np.random.rand(5, 1)*3
print(x_gt)
print(x_noisy)

# Fixed

A = np.random.rand(10, 5)
y = A @ x_gt

# Non-linear least-squares
# argmin(1/2 * |E(x)|^2)  --- (1)

# Taylor series expansion
# E(x1) ~= E(x0) + J_{x0}*(x1 - x0) + 0.5*H_{x0}*(x1-x0)^2 + ...

# Newton's method
# Take the derivative of (1) with respect to x 
# and with 2nd order Taylor series approximation, we get
# dE(x1) / dx1 = J_{x0} + H_{x0}*(x1 - x0)  --- (2)
# Therefore, its minimum will be
# dE(x1) / dx1 = 0
# J_{x0} + H_{x0}*delta_x = 0       [delta_x = x1 - x0]
# delta_x = H_{x0}^{-1} J_{x0}


# Gauss-Newton method
