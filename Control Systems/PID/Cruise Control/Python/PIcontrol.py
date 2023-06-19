import numpy as np
import control
import matplotlib.pyplot as plt

m = 1000  # Mass
b = 50    # Damping coefficient
r = 10    # Reference input

# Transfer function of the plant (cruise control system)

s = control.TransferFunction.s
P_cruise = 1 / (m * s + b)

Kp = 800     # Proportional gain
Ki = 40      # Integral gain
C = control.TransferFunction([Kp, Ki], [1, 0])  # Create the controller transfer function

# Closed-loop transfer function
T = control.feedback(C * P_cruise, 1)

t = np.arange(0, 20, 0.1)
t, y = control.step_response(r * T, T=t)

plt.plot(t, y)
plt.axis([0, 20, 0, 10])
plt.xlabel('Time')
plt.ylabel('Output')
plt.title('Step Response')
plt.grid(True)
plt.show()
