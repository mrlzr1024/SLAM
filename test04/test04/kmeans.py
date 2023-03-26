import numpy as np
import matplotlib.pyplot as plt

from sklearn.cluster import KMeans
from sklearn.datasets import make_blobs

X, y = make_blobs(1500)

fig = plt.figure()
print(X)

    # ax = fig.add_subplot(1,2,i+1)
y = KMeans(3).fit_predict(X)
plt.scatter(X[:, 0], X[:, 1], c=y)

plt.show()
