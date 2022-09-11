import open3d as o3d
import open3d.visualization.gui as gui
import numpy as np


class Vertiports:
    def __init__(self, app, em, window, m, coord_scale, _3d, line_mat, line_mat_thick, cdir):
        self.app = app
        self.em = em
        self.m = m
        self.cdir = cdir
        self.coord_scale = coord_scale
        self._3d = _3d
        self.line_mat = line_mat
        self.line_mat_thick = line_mat_thick
        self.window = window

        self.line_mat_ = o3d.visualization.rendering.MaterialRecord()
        self.line_mat_.shader = "unlitLine"
        self.line_mat_.line_width = 10

        self.vertiport = gui.VGrid(2)

        self.create_button = gui.Button('Create')
        self.create_button.vertical_padding_em = 0
        self.create_button.enabled = True
        self.create_button.set_on_clicked(self._on_create_new_window)

        self.vertiport.add_child(self.create_button)

    def _on_create_new_window(self):
        self.subwindow_vertiport = self.app.create_window("Vertiports Configuration", 700, 500)

        self.vertiport_view = gui.SceneWidget()
        self.vertiport_view.scene = o3d.visualization.rendering.Open3DScene(self.subwindow_vertiport.renderer)
        self.vertiport_view.scene.set_background([0.5, 0.5, 0.5, 1.0])

        self.config_view = gui.Vert(0.5 * self.em, gui.Margins(0, 0.5 * self.em, 0, 0))
        self.config_view.background_color = o3d.visualization.gui.Color(0.21, 0.22, 0.25, 1.0)

        self.subwindow_vertiport.add_child(self.vertiport_view)
        self.subwindow_vertiport.add_child(self.config_view)
        self.subwindow_vertiport.set_on_layout(self._config_on_layout)

        # circlePoints, lines, colors = self.gen_pad(0, 0, 2)
        # circle0_set = o3d.geometry.LineSet(
        #     points=o3d.utility.Vector3dVector(circlePoints),
        #     lines=o3d.utility.Vector2iVector(lines),
        # )
        # circle0_set.colors = o3d.utility.Vector3dVector(colors)
        # self.vertiport_view.scene.add_geometry(f'circle', circle0_set, self.line_mat_)

    def _config_on_layout(self, context=None):
        r = self.subwindow_vertiport.content_rect
        # print(frame.x, frame.y, frame.width, frame.height)  # default: 0 0 1080 720
        self.vertiport_view.frame = gui.Rect(r.x, r.y, r.width / 3 * 2, r.height)
        self.config_view.frame = gui.Rect(r.x + r.width / 3 * 2 + 1, r.y, r.width / 3, r.height)

    def gen_pad(self, center_x, center_y, radius):
        N = 100  # number of discrete sample points to be generated along the circle
        theta = np.linspace(0, 360, N)

        r_out = radius
        r_in = radius / 2

        # generate points
        circleOut = []
        circleIn = []
        for k in range(N):
            # compute
            angle = np.deg2rad(theta[k])

            dx_out = r_out * np.cos(angle)
            dy_out = r_out * np.sin(angle)
            x_out = center_x + dx_out
            y_out = center_y + dy_out

            dx_in = r_in * np.cos(angle)
            dy_in = r_in * np.sin(angle)
            x_in = center_x + dx_in
            y_in = center_y + dy_in

            # add to list
            circleOut.append([x_out, y_out, 0])
            circleIn.append([x_in, y_in, 0])

        v_points = [[center_x + r_in / 2, center_y + r_in / 2, 0],
                    [center_x, center_y - r_in / 2, 0],
                    [center_x - r_in / 2, center_y + r_in / 2, 0]
                    ]

        lines_out = [[i, i + 1] for i in range(len(circleOut) - 1)]
        lines_in = [[i + len(circleOut), i + 1 + len(circleOut)] for i in range(len(circleIn) - 1)]
        lines_v = [[i + len(circleOut + circleIn), i + 1 + len(circleOut + circleIn)] for i in range(len(v_points) - 1)]

        colors_out = [[1.0, 1.0, 1.0] for _ in range(len(lines_out))]
        colors_in = [[1.0, 1.0, 0.0] for _ in range(len(lines_in))]
        colors_v = [[1.0, 1.0, 1.0] for _ in range(len(lines_v))]

        circlePoints = circleOut + circleIn + v_points
        lines = lines_out + lines_in + lines_v
        colors = colors_out + colors_in + colors_v

        return circlePoints, lines, colors
