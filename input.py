### example ###

from MaxFlow import *

valid_data = np.array([[0, 10, 0, 8, 0, 0],
                [0, 0, 5, 2, 0, 0],
                [0, 0, 0, 0, 0, 7],
                [0, 1, 0, 0, 10, 0],
                [0, 0, 8, 0, 0, 10],
                [5, 0, 0, 0, 0, 0]])

EdmondKarp_graph = Graph(valid_data)

print("Resultat : \n")
print(Graph(valid_data).EdmondKarp())