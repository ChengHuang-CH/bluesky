import open3d as o3d
import open3d.visualization.gui as gui
import json

app = gui.Application.instance
app.initialize()
window = app.create_window("Sensor Stream", 720, 480)
em = window.theme.font_size

with open('../../../data/osm/sensor_config.json', 'r') as data_file:
    data_loaded = json.load(data_file)

sensor_names = list(data_loaded.keys())
# n_sensors = len(sensor_names)
n_sensors = 2

print(f'{n_sensors} sensors loaded!')

_3d_wins = [gui.SceneWidget()] * n_sensors

for i in range(n_sensors):
    _3d_wins[i].scene = o3d.visualization.rendering.Open3DScene(window.renderer)
    _3d_wins[i].scene.set_background([1.0, 1.0, 1.0, 1.0])
    # _3d_wins[i].scene.show_axes(True)
    # bounds = _3d_wins[i].scene.bounding_box
    # center = bounds.get_center()
    # _3d_wins[i].setup_camera(60, bounds, center)
    # _3d_wins[i].look_at([0, 0, 0], [0, 0, 10], [0, 1, 0])

    # _3d_wins.append(_3d)

for i in range(n_sensors):
    window.add_child(_3d_wins[i])


def on_layout(context=None):
    frame = window.content_rect
    grid_col = 3
    # https://stackoverflow.com/questions/2356501/how-do-you-round-up-a-number
    grid_row = n_sensors // grid_col + (n_sensors % grid_col > 0)

    grid_w = frame.width / grid_col
    grid_h = frame.height / grid_row

    for i in range(n_sensors):
        row_i = i // grid_col
        col_i = i % grid_col
        print(row_i, col_i)
        _3d_wins[i].frame = gui.Rect(frame.x + col_i * grid_w,
                                     frame.y + row_i * grid_h,
                                     grid_w,
                                     grid_h)


window.set_on_layout(on_layout)
app.run()
