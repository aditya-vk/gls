from IPython import embed
import argparse
import numpy
import networkx as nx

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='script for generating obstacle files')

    args = parser.parse_args()

    indices = [100, 500, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000]

    for idx in indices:

        G = nx.Graph()

        directory = "/home/adityavk/workspaces/lab-ws/src/planning_dataset/"
        default_vertex_location = directory + 'vertices_' + str(idx) + '.txt'
        default_edges_location = directory + 'edges_' + str(idx) + '.txt'
        edgeStates_location = directory + 'edges_viz_' + str(idx) + '.txt'
        graph_save_location = directory + 'graph_se2_apartment_' + str(idx) + '.graphml'

        # Collect vertices.
        halton = numpy.loadtxt(default_vertex_location)
        n = numpy.shape(halton)[0]

        # Add vertices
        print("Adding vertices")
        for i in range(n):
            print(i)
            state = ''
            for j in range(2):
                state += str(halton[i][j]) + ' '
            state += str(halton[i][-1])

            G.add_node(i, state = state)

        # Collect edges.
        haltonEdges = numpy.loadtxt(default_edges_location)

        # Adding Edges
        print("Adding edges")
        for i in range(numpy.shape(haltonEdges)[0]):
            dist = haltonEdges[i][-1]
            G.add_edge(int(haltonEdges[i][0]), int(haltonEdges[i][1]), length = str(dist))

        print('Average degree: ', G.number_of_edges() * 2.0 / n)
        print('Connected: ', nx.is_connected(G))
        print('Number of Connected Components: ', nx.number_connected_components(G))
        nx.write_graphml(G, graph_save_location)
