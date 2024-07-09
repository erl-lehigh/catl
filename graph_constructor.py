import networkx as nx

G = nx.complete_graph(5)
nx.write_gexf(G, "test.gexf")