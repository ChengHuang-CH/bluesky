import json
import numpy as np
import pyproj
import scipy.spatial.transform
import time

# camera model
class CameraModel:
    def __init__(self):
        # extrinsic
        self.extrinsic = {'latitude': 0, 'longitude': 0, 'altitude': 0,
                          'pitch': 0, 'yaw': 0, 'roll': 0}
        # intrinsics
        self.intrinsics = {'img_width': 1920,
                           'img_height': 1080,
                           'fov': 60,
                           'range': None}


# radar model
class RadarModel:
    def __init__(self):
        # extrinsic
        self.extrinsic = {'latitude': 0, 'longitude': 0, 'altitude': 0,
                          'pitch': 0, 'yaw': 0, 'roll': 0}
        # intrinsics
        self.intrinsics = {'horizontal_fov': 90,
                           'vertical_fov': 10,
                           'range': 100}


# lidar model
class LidarModel:
    def __init__(self):
        # extrinsic
        self.extrinsic = {'latitude': 0, 'longitude': 0, 'altitude': 0,
                          'pitch': 0, 'yaw': 0, 'roll': 0}
        # intrinsics
        self.intrinsics = {'horizontal_fov': 360,
                           'upper_fov': 10,
                           'lower_fov': -30,
                           'range': 100}


def save():
    """
    save sensor mode parameters
    @return:
    """
    pole01 = [52.070355, -0.628010]  # lat, lon
    pole02 = [52.070038, -0.628239]
    pole08 = [52.067914, -0.628254]
    pole09 = [52.067503, -0.628109]
    pole10 = [52.067125, -0.628101]
    pole11 = [52.066892, -0.628137]
    pole12 = [52.066839, -0.628404]
    pole13 = [52.066724, -0.628315]
    pole14 = [52.066628, -0.628583]
    pole15 = [52.066560, -0.629586]
    pole16 = [52.066400, -0.629972]
    pole17 = [52.066326, -0.630516]
    pole18 = [52.066230, -0.631189]
    pole19 = [52.066500, -0.631487]

    sensors = {'camera10': {'model': 'camera',
                            'extrinsic': {'latitude': pole10[0], 'longitude': pole10[1], 'altitude': 4.0,
                                          'pitch': 0, 'yaw': -100, 'roll': 0},
                            'intrinsic': {'img_width': 1920, 'img_height': 1080, 'fov': 40, 'range': None}
                            },  # 52.066889, -0.628129
               'camera12': {'model': 'camera',
                            'extrinsic': {'latitude': pole12[0], 'longitude': pole12[1], 'altitude': 4.0,
                                          'pitch': 0, 'yaw': -140, 'roll': 0},
                            'intrinsic': {'img_width': 1920, 'img_height': 1080, 'fov': 40, 'range': None}
                            },
               'camera13': {'model': 'camera',
                            'extrinsic': {'latitude': pole13[0], 'longitude': pole13[1], 'altitude': 4.0,
                                          'pitch': 0, 'yaw': 70, 'roll': 0},
                            'intrinsic': {'img_width': 1920, 'img_height': 1080, 'fov': 40, 'range': None}
                            },
               'camera14': {'model': 'camera',
                            'extrinsic': {'latitude': pole14[0], 'longitude': pole14[1], 'altitude': 4.0,
                                          'pitch': 0, 'yaw': -160, 'roll': 0},
                            'intrinsic': {'img_width': 1920, 'img_height': 1080, 'fov': 40, 'range': None}
                            },
               'radar11': {'model': 'radar',
                           'extrinsic': {'latitude': pole11[0], 'longitude': pole11[1], 'altitude': 4.0,
                                         'pitch': 0, 'yaw': -140, 'roll': 0},
                           'intrinsic': {'horizontal_fov': 30, 'vertical_fov': 10, 'range': 150}
                           },

               'lidar11': {'model': 'lidar',
                           'extrinsic': {'latitude': pole11[0], 'longitude': pole11[1], 'altitude': 4.0,
                                         'pitch': 0, 'yaw': 90, 'roll': 0},
                           'intrinsic': {'horizontal_fov': 270, 'upper_fov': 10, 'lower_fov': -30, 'range': 100}
                           },
               'radar14': {'model': 'radar',
                           'extrinsic': {'latitude': pole14[0], 'longitude': pole14[1], 'altitude': 4.0,
                                         'pitch': 0, 'yaw': 60, 'roll': 0},
                           'intrinsic': {'horizontal_fov': 30, 'vertical_fov': 10, 'range': 150}
                           },
               'radar18': {'model': 'radar',
                           'extrinsic': {'latitude': pole18[0], 'longitude': pole18[1], 'altitude': 4.0,
                                         'pitch': 0, 'yaw': 20, 'roll': 0},
                           'intrinsic': {'horizontal_fov': 30, 'vertical_fov': 10, 'range': 150}
                           },
               'camera19': {'model': 'camera',
                            'extrinsic': {'latitude': pole19[0], 'longitude': pole19[1], 'altitude': 8.0,
                                          'pitch': 0, 'yaw': -10, 'roll': 0},
                            'intrinsic': {'img_width': 1920, 'img_height': 1080, 'fov': 30, 'range': None}
                            },
               }
    with open('../../../data/osm/sensor_config.json', 'w') as f:
        json.dump(sensors, f)


def load():
    """
    load sensor mode parameters
    @return:
    """
    with open('../../../data/osm/sensor_config.json', 'r') as data_file:
        data_loaded = json.load(data_file)

    names = list(data_loaded.keys())
    print(data_loaded[names[0]])
    print(data_loaded)


def test_sensor_boundary():
    import numpy as np
    import math
    import matplotlib.pyplot as plt

    # inputs
    radius = 100.0  # m - the following code is an approximation that stays reasonably accurate for distances < 100km
    centerLat = 30.0  # latitude of circle center, decimal degrees
    centerLon = -100.0  # Longitude of circle center, decimal degrees

    R = 6378137

    yaw = 0
    fov = 40

    angle_left = yaw - fov / 2
    angle_right = yaw + fov / 2

    # parameters
    N = 10  # number of discrete sample points to be generated along the circle

    theta = np.linspace(angle_left, angle_right, N)

    # generate points
    circlePoints = []
    for k in range(N):
        # compute
        angle = np.deg2rad(theta[k])

        dx = radius * np.cos(angle)
        dy = radius * np.sin(angle)
        point = []
        point.append(centerLat + (180 / np.pi) * (dy / 6378137))
        point.append(centerLon + (180 / np.pi) * (dx / 6378137) / np.cos(centerLat * np.pi / 180))
        # add to list
        circlePoints.append(point)

    circlePoints = np.asarray(circlePoints)

    print(circlePoints)

    plt.plot(centerLat, centerLon, marker="o", color='r')
    plt.plot(circlePoints[:, 0], circlePoints[:, 1], marker="o")
    for i in range(N):
        plt.plot(circlePoints[i, 0], circlePoints[i, 1], marker="o")
    plt.show()


def geodetic2enu(lat, lon, alt, lat_org, lon_org, alt_org):
    """
    convert the gps point to ENU point reference to a local point.
    """
    transformer = pyproj.Transformer.from_crs(
        {"proj": 'latlong', "ellps": 'WGS84', "datum": 'WGS84'},
        {"proj": 'geocent', "ellps": 'WGS84', "datum": 'WGS84'},
    )
    x, y, z = transformer.transform(lon, lat, alt, radians=False)
    x_org, y_org, z_org = transformer.transform(lon_org, lat_org, alt_org, radians=False)
    vec = np.array([[x - x_org, y - y_org, z - z_org]]).T

    rot1 = scipy.spatial.transform.Rotation.from_euler('x', -(90 - lat_org),
                                                       degrees=True).as_matrix()  # angle*-1 : left handed *-1
    rot3 = scipy.spatial.transform.Rotation.from_euler('z', -(90 + lon_org),
                                                       degrees=True).as_matrix()  # angle*-1 : left handed *-1

    rotMatrix = rot1.dot(rot3)

    enu = rotMatrix.dot(vec).T.ravel()
    return enu.T


def test_transform():
    sensor_point = [52.066723, -0.628317, 4]
    target_point = [52.066885, -0.628135, 6]

    reference_point = [52.070378, -0.628175, 0]

    sensor_pt = geodetic2enu(sensor_point[0], sensor_point[1], sensor_point[2],
                             reference_point[0], reference_point[1], reference_point[2])

    target_pt = geodetic2enu(target_point[0], target_point[1], target_point[2],
                             reference_point[0], reference_point[1], reference_point[2])

    sensor_pitch, sensor_yaw, sensor_roll = 0, np.deg2rad(90), 0  # r_y, r_z, r_x

    pitch, yaw, roll = sensor_pitch, sensor_yaw, sensor_roll

    # http://brainvoyager.com/bv/doc/UsersGuide/CoordsAndTransforms/SpatialTransformationMatrices.html
    rx = np.array([[1, 0, 0], [0, np.cos(roll), np.sin(roll)], [0, -np.sin(roll), np.cos(roll)]])
    ry = np.array([[np.cos(pitch), 0, -np.sin(pitch)], [0, 1, 0], [np.sin(pitch), 0, np.cos(pitch)]])
    rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])

    rotation_matrix = rz.dot(ry).dot(rx)

    translation = np.array([[sensor_pt[0]], [sensor_pt[1]], [sensor_pt[2]]])
    sensor2world = np.hstack((rotation_matrix, translation))
    sensor2world = np.vstack((sensor2world, np.array([[0, 0, 0, 1]])))

    target = np.array([[target_pt[0]], [target_pt[1]], [target_pt[2]], [1]])

    world2sensor = np.linalg.inv(sensor2world)

    trans = np.dot(world2sensor, target)
    print(trans)


if __name__ == '__main__':
    save()
    load()
    # t0 = time.time()
    # test_transform()
    # t1 = time.time()
    # print(t1-t0)
