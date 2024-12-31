import numpy as np

# Given data matrix A
A = np.array([[91, 82, 82],
              [74, 84, 90],
              [90, 90, 90],
              [71, 91, 87],
              [78, 86, 93],
              [88, 78, 94]])

# Step 2: Standardize the data
Z = A - np.mean(A, axis=0)

# Step 3: Calculate the covariance matrix
C = np.cov(Z, rowvar=False) 

# Step 4: Eigenvalues and eigenvectors
eigenvalues, eigenvectors = np.linalg.eig(C)

# Step 5: Sort the eigenvalues and eigenvectors
idx = np.argsort(eigenvalues)[::-1] 
eigenvalues = eigenvalues[idx]
eigenvectors = eigenvectors[:, idx]

# Select the top 2 eigenvectors
W = eigenvectors[:, :2]
for i in range(W.shape[1]):
    if W[0, i] < 0:
        W[:, i] *= -1

# Step 6: Project the data
Z_reduced = np.dot(Z, W)

# Display the reduced data
print('Reduced Data SVD:')
print(Z_reduced)