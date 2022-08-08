import open3d as o3d
import open3d.visualization.gui as gui
import json
import numpy as np

app = gui.Application.instance
app.initialize()
window = app.create_window("Sensor Stream", 1080, 420)
em = window.theme.font_size

with open('../../../data/osm/sensor_config.json', 'r') as data_file:
    data_loaded = json.load(data_file)

sensor_names = list(data_loaded.keys())
n_sensors = len(sensor_names)

print(f'{n_sensors} sensors loaded!')

# _3d_scn1 = gui.SceneWidget()
# _3d_scn1.scene = o3d.visualization.rendering.Open3DScene(window.renderer)
# _3d_scn2 = gui.SceneWidget()
# _3d_scn2.scene = o3d.visualization.rendering.Open3DScene(window.renderer)
#
# window.add_child(_3d_scn1)
# window.add_child(_3d_scn2)

_3d_wins = [gui.SceneWidget() for _ in range(n_sensors)]
for i in range(n_sensors):
    _3d_wins[i].scene = o3d.visualization.rendering.Open3DScene(window.renderer)
    _3d_wins[i].scene.set_background([1.0, 1.0, 1.0, 1.0])
    _3d_wins[i].scene.show_axes(True)
    bounds = _3d_wins[i].scene.bounding_box
    center = bounds.get_center()
    _3d_wins[i].setup_camera(60, bounds, center)
    _3d_wins[i].look_at([0, 0, 0], [0, 0, 100], [0, 1, 0])

for i in range(n_sensors):
    window.add_child(_3d_wins[i])


def on_layout(context=None):
    frame = window.content_rect
    grid_col = 3
    # https://stackoverflow.com/questions/2356501/how-do-you-round-up-a-number
    grid_row = n_sensors // grid_col + (n_sensors % grid_col > 0)

    grid_w = frame.width / grid_col
    grid_h = frame.height / grid_row

    # _3d_scn1.frame = gui.Rect(frame.x + 0 * grid_w, frame.y + 0 * grid_h, grid_w, grid_h)
    # _3d_scn2.frame = gui.Rect(frame.x + 1 * grid_w + 1, frame.y + 0 * grid_h, grid_w, grid_h)

    for i in range(n_sensors):
        row_i = i // grid_col
        col_i = i % grid_col
        _3d_wins[i].frame = gui.Rect(frame.x + col_i * grid_w + 1,
                                     frame.y + row_i * grid_h + 1,
                                     grid_w - 1,
                                     grid_h - 1)


def camera_container(sensor_i_data):
    # 0,1080 ----------------------------- 1920, 1080
    #  |                                      |
    #  |                                      |
    # 0,0  ------------------------------- 1920, 0

    width = sensor_i_data['intrinsic']['img_width'] / 10  # 1920
    height = sensor_i_data['intrinsic']['img_height'] / 10  # 1080

    points = [[0, 0, 0], [0, height, 0], [width, height, 0], [width, 0, 0]]
    lines = [[0, 1], [1, 2], [2, 3], [3, 0]]
    colors = [[0, 0, 1] for _ in range(len(lines))]

    camera_bound_lineset = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines),
    )
    camera_bound_lineset.colors = o3d.utility.Vector3dVector(colors)

    o3d_cam_focus = [width / 2, height / 2]

    return camera_bound_lineset, o3d_cam_focus


def radar_container(sensor_i_data):
    #  fov, range
    radius = sensor_i_data['intrinsic']['range']

    alt = 0  # set alt to 0 for plotting

    fov = sensor_i_data['intrinsic']['horizontal_fov']

    assert fov <= 360

    angle_left = - fov / 2
    angle_right = fov / 2

    # parameters
    N = fov * 2  # number of discrete sample points to be generated along the circle
    theta = np.linspace(angle_left, angle_right, N)

    # generate points
    circlePoints = []

    if fov != 360:
        circlePoints.append([0, 0, 0])  # append center points at first

    for k in range(N):
        # compute
        angle = np.deg2rad(theta[k])
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        point = [x, y, 0]
        # add to list
        circlePoints.append(point)

    if fov != 360:
        circlePoints.append([0, 0, alt])  # append center points at last

    lines = []
    for i in range(len(circlePoints) - 1):
        lines.append([i, i + 1])

    colors = [[0, 0, 1] for _ in range(len(lines))]

    radar_bound_lineset = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(circlePoints),
        lines=o3d.utility.Vector2iVector(lines),
    )
    radar_bound_lineset.colors = o3d.utility.Vector3dVector(colors)

    angle = np.deg2rad(theta[0])
    width = radius * np.cos(angle)
    o3d_cam_focus = [radius / 2, 0]

    return radar_bound_lineset, o3d_cam_focus


def _on_mouse_view2d(event):
    return gui.Widget.EventCallbackResult.CONSUMED


line_mat_thick = o3d.visualization.rendering.MaterialRecord()
line_mat_thick.shader = "unlitLine"
line_mat_thick.line_width = 2

mat = o3d.visualization.rendering.MaterialRecord()
mat.shader = 'defaultUnlit'
mat.point_size = 8.0

camera_bound_lineset0, o3d_cam_focus = camera_container(data_loaded['camera10'])
_3d_wins[0].scene.add_geometry(f'{sensor_names[0]}', camera_bound_lineset0, line_mat_thick)
_3d_wins[0].look_at([o3d_cam_focus[0], o3d_cam_focus[1], 0], [o3d_cam_focus[0], o3d_cam_focus[1], 100], [0, 1, 0])
_3d_wins[0].set_on_mouse(_on_mouse_view2d)


def uv2xy(u_v_z):
    x = u_v_z[0]
    y = 108 - u_v_z[1]
    z = u_v_z[2]
    return [x, y, z]


xyz = [uv2xy(uvz) for uvz in [[50, 50, 0], [190, 100, 0]]]
target_point_set = o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(xyz))
target_point_set.colors = o3d.utility.Vector3dVector([[1, 0, 0], [0, 1, 1]])
_3d_wins[0].scene.add_geometry(f'points', target_point_set, mat)

radar_bound_lineset0, cam_focus = radar_container(data_loaded['radar11'])
_3d_wins[2].scene.add_geometry(f'{sensor_names[2]}', radar_bound_lineset0, line_mat_thick)
_3d_wins[2].look_at([cam_focus[0], cam_focus[1], 0], [cam_focus[0], cam_focus[1], 90], [0, 1, 0])

rad_point = [100, -10.1]  # m, deg
# polar to xy
x = rad_point[0] * np.cos(np.deg2rad(rad_point[1]))
y = - rad_point[0] * np.sin(np.deg2rad(rad_point[1]))
rad_point_set = o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector([[x, y, 0]]))
rad_point_set.colors = o3d.utility.Vector3dVector([[1, 0, 0]])
_3d_wins[2].scene.add_geometry(f'rad_points', rad_point_set, mat)
_3d_wins[2].set_on_mouse(_on_mouse_view2d)

window.set_on_layout(on_layout)
app.run()
