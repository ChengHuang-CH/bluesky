import osmnx as ox
import matplotlib.pyplot as plt
from shapely.ops import triangulate

# ==============================================================
import numpy as np
from shapely.geometry import Polygon
from shapely.ops import triangulate
from shapely.ops import voronoi_diagram
import shapely.wkt
import geopandas as gpd
from geovoronoi import voronoi_regions_from_coords
import matplotlib.pyplot as plt


# https://gis.stackexchange.com/questions/316697/delaunay-triangulation-algorithm-in-shapely-producing-erratic-result
def to_triangles(polygon):
    poly_points = []

    gdf_poly_exterior = gpd.GeoDataFrame({'geometry': [polygon.buffer(-0.0000001).exterior]}).explode(index_parts=True).reset_index()
    for geom in gdf_poly_exterior.geometry:
        poly_points += np.array(geom.coords).tolist()

    try:
        polygon.interiors[0]
    except:
        poly_points = poly_points
    else:
        gdf_poly_interior = gpd.GeoDataFrame({'geometry': [polygon.interiors]}).explode(index_parts=True).reset_index()
        for geom in gdf_poly_interior.geometry:
            poly_points += np.array(geom.coords).tolist()

    poly_points = np.array([item for sublist in poly_points for item in sublist]).reshape(-1, 2)

    poly_shapes, pts = voronoi_regions_from_coords(poly_points, polygon)
    gdf_poly_voronoi = gpd.GeoDataFrame({'geometry': poly_shapes}).explode(index_parts=True).reset_index()
    gdf_poly_voronoi.plot()

    tri_geom = []
    for geom in gdf_poly_voronoi.geometry:
        inside_triangles = [tri for tri in triangulate(geom) if tri.centroid.within(polygon)]
        tri_geom += inside_triangles

    gdf_poly_triangles = gpd.GeoDataFrame({'geometry': tri_geom})

    gdf_poly_exterior.plot()
    if 'gdf_poly_interior' in locals():
        gdf_poly_interior.plot()
    gdf_poly_triangles.plot()

    # plt.show()


polygon_1 = Polygon([(0, 0), (0, 3), (5, 3), (2, 4), (6, 4), (6, 0)])
polygon_2 = Polygon(
    [(3.0, 0.0), (2.0, 0.0), (2.0, 0.75), (2.5, 0.75), (2.5, 0.6), (2.25, 0.6), (2.25, 0.2), (3.0, 0.2), (3.0, 0.0)])

# to_triangles(polygon_2)

# ==============================================================

local_file = '../../../data/osm/cranfield.osm'
G = ox.graph_from_xml(local_file)
buildings = ox.geometries_from_xml(local_file, tags={'building': True})

bulding_names = buildings['name'].tolist()
building_geos = buildings['geometry'].tolist()

index = bulding_names.index('Building 83')

polygon = building_geos[index]
fig, ax = ox.plot_footprints(buildings)

to_triangles(polygon)

x, y = polygon.exterior.xy
coords = list(polygon.exterior.coords)
# print(x, y)
# plt.plot(x, y)
# plt.show()


# triangles = triangulate(polygon)
# print([triangle for triangle in triangles])
# for triangle in triangles:
#     plt.plot(*triangle.exterior.xy)

# print([triangle.wkt for triangle in triangles])
# triangles_gdf = gpd.GeoDataFrame()
# triangles_gdf.geometry = triangles
# triangles_gdf.plot()

plt.show()
