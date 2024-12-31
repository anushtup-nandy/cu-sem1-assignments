import numpy as np
from sklearn.decomposition import PCA

# Given data matrix A
A = np.array([[91, 82, 82],
              [74, 84, 90],
              [90, 90, 90],
              [71, 91, 87],
              [78, 86, 93],
              [88, 78, 94]])

# Step 1: Subtract the mean 
Z = A - np.mean(A, axis=0)

# Step 2: Apply PCA 
pca = PCA(n_components=2)
Z_reduced = pca.fit_transform(Z)

# Step 3: Display the reduced data
print('Reduced Data PCA:')
print(Z_reduced)

