from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt
import numpy as np

fig = plt.figure()

ax = fig.add_subplot(111, projection='3d')

L = 1

x = [L, -L, 0, 0]
y = [0, 0, L, -L]
z = [0, 0, 1, 1]

#vertices = np.array([[(-L, 0,0)],[(L, 0, 0)],[(0, -L, 0)],[(0, L, 0)]])
#vertices = [list(zip(x,y,z))]

vertices_xAxis = [[(-L, 0, 0), (L, 0, 0), (L, 0, 0.25), (-L, 0, 0.25)]]
vertices_yAxis = [[(0, -L, 0), (0, L, 0), (0, L, 0.25), (0, -L, 0.25)]]

print(vertices_xAxis)

poly = Poly3DCollection(vertices_xAxis, alpha=0.5)
poly2 = Poly3DCollection(vertices_yAxis, alpha=0.5, color="red")
ax.add_collection3d(poly)
ax.add_collection3d(poly2)

plt.show()

"""
points = np.array([[-1,-1,-1],
                  [1,-1,-1],
                  [1,1,-1],
                  [-1,1,-1],
                  [-1,-1,1],
                  [1,-1,1],
                  [1,1,1],
                  [-1,1,1]])

Z = points
Z = 10.0 * Z
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
r = [-1, 1]
X,Y = np.meshgrid(r, r)
ax.scatter3D(Z[:, 0], Z[:, 1], Z[:, 2])
verts = [[Z[0],Z[1],Z[2],Z[3]],
 [Z[4],Z[5],Z[6],Z[7]],
 [Z[0],Z[1],Z[5],Z[4]],
 [Z[2],Z[3],Z[7],Z[6]],
 [Z[1],Z[2],Z[6],Z[5]],
 [Z[4],Z[7],Z[3],Z[0]]]
ax.add_collection3d(Poly3DCollection(verts, facecolors='cyan', linewidths=1, edgecolors='r', alpha=.20))
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()
"""