import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# Load dataset
A = pd.read_csv('/home/anushtup/git-repos/assignments/data-science/hw2/ovariancancer_obs.csv', header=None)
A = A.astype(float).values  # Convert to float and then to numpy array
print(A)

# Load b
b = pd.read_csv('/home/anushtup/git-repos/assignments/data-science/hw2/updated_file.csv')
# b = b['Status'].values  # Select the 'Status' column
b = b['Status'].values  # Select the 'Status' column

# Solve Ax=b using SVD
U, S, VT = np.linalg.svd(A, full_matrices=0.)
x = VT.T @ np.linalg.inv(np.diag(S)) @ U.T @ b

# Plotting
plt.figure(figsize=(12, 6))
plt.plot(b, color='k', linewidth=2, label='Target Data')
plt.plot(A@x, '-o', color='r', label='Regression')

# Adding labels and legend
plt.title('Target Data and Regression Analysis')
plt.xlabel('Sample Index')
plt.ylabel('Values')
plt.legend()
plt.grid(True)

# Alternative solution using pseudoinverse
x_alt = np.linalg.pinv(A) @ b

# Plotting the alternative solution
plt.plot(A@x_alt, '--', color='g', label='Alternative Regression')

plt.legend()
plt.show()