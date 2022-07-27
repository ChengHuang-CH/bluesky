import osmnx as ox
import matplotlib.pyplot as plt
from shapely.ops import triangulate

local_file = '../../../data/osm/cranfield.osm'
G = ox.graph_from_xml(local_file)
buildings = ox.geometries_from_xml(local_file, tags={'building': True})

bulding_names = buildings['name'].tolist()
building_geos = buildings['geometry'].tolist()

index = bulding_names.index('Building 83')

polygon = building_geos[index]
fig, ax = ox.plot_footprints(buildings)

x, y = polygon.exterior.xy
coords = list(polygon.exterior.coords)
print(x, y)
# plt.plot(x, y)
# plt.show()

triangles = triangulate(polygon, tolerance=1.0, edges=False)

for triangle in triangles:
    plt.plot(*triangle.exterior.xy)
# print([triangle.wkt for triangle in triangles])

plt.show()
