import numpy as np
import matplotlib.pyplot as plt
from matplotlib.image import imread
import os

#Load Image
A = imread(os.path.join('anushtup.jpg'))
X = np.mean(A, -1)  # Convert RGB to grayscale

#Take the SVD
U, S, VT = np.linalg.svd(X, full_matrices=False)
S = np.diag(S)

#3x2 plot grid
fig, axes = plt.subplots(3, 2, figsize=(12, 18))
r_values = [5, 20, 50, 100, 150, 200]  #Increasing number of singular values

for i, r in enumerate(r_values):
    #Approximate image with the first r singular values
    Xapprox = U[:, :r] @ S[0:r, :r] @ VT[:r, :]
    
    #Plot the approximated image in the left column
    ax_img = axes[i // 2, i % 2]
    ax_img.imshow(Xapprox, cmap='gray')
    ax_img.set_title(f'Approximation with {r} singular values')

plt.subplots_adjust(wspace=0.5, hspace=0.5)

plt.tight_layout()
plt.savefig('./question1_sol.png')