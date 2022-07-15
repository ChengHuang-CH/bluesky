import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
from mpl_toolkits.basemap import Basemap
import time

import bluesky as bs
from bluesky import stack
from bluesky import settings
from bluesky.navdatabase import Navdatabase


class BlueSky3dUI:
    def __init__(self):
        # ===============================================================
        # opensky setting configuration
        settings.init(cfgfile=None)
        self.navdb = Navdatabase()
        self.firx0, self.firy0 = self.navdb.firlat0.tolist(), self.navdb.firlon0.tolist()
        self.firx1, self.firy1 = self.navdb.firlat1.tolist(), self.navdb.firlon1.tolist()

        self.firlat = [self.firx0[0]] + self.firx1
        self.firlon = [self.firy0[0]] + self.firy1

        self.wpid, self.wplat, self.wplon, self.wpelev = self.navdb.wpid, self.navdb.wplat, \
                                                         self.navdb.wplon, self.navdb.wpelev
        self.aptid, self.aptname, self.aptlat, self.aptlon = self.navdb.aptid, self.navdb.aptname, \
                                                             self.navdb.aptlat, self.navdb.aptlon
        stack.stack(f'IC')

        # ===============================================================

        self._init_coastline()

        # create open3d window
        self.app = gui.Application.instance
        self.app.initialize()
        self.window = self.app.create_window("BlueSky3D", 1080, 720)
        self.em = self.window.theme.font_size

        self.mat = o3d.visualization.rendering.MaterialRecord()
        self.mat.shader = "defaultLit"
        self.mat.base_color = [1.0, 1.0, 1.0, 1.0]

        # cube for demo
        self.cube = o3d.geometry.TriangleMesh.create_box(width=1.0,
                                                         height=1.0,
                                                         depth=0.01)
        self.cube.compute_vertex_normals()

        self._3d = gui.SceneWidget()
        self._3d.scene = o3d.visualization.rendering.Open3DScene(self.window.renderer)
        self._3d.scene.set_background([0.1, 0.1, 0.25, 1.0])
        self._3d.scene.show_axes(True)

        self._3d.scene.add_geometry("coastlines", self.line_set, self.mat)
        bbox = o3d.geometry.AxisAlignedBoundingBox([-1, -1, -1],
                                                   [1, 1, 1])
        self.bounds = self._3d.scene.bounding_box
        self.center = self.bounds.get_center()

        self._3d.setup_camera(60, self.bounds, self.center)  # field_of_view, model_bounds, center_of_rotation

        self.cam_x0, self.cam_y0, self.cam_z0 = self.xpt, self.ypt, 500
        self.cam_x, self.cam_y, self.cam_z = self.cam_x0, self.cam_y0, self.cam_z0
        self.cam_up = [0, 1, 0]
        # set camera y up, in camera view:
        #   ^ y up
        #   |
        #   |
        # z(x) -----> x
        #  right-hand coordinate system

        # look_at(center, eye, up): sets the camera view so that the camera is located at ‘eye’,
        # pointing towards ‘center’, and oriented so that the up vector is ‘up’
        self._3d.look_at([self.cam_x, self.cam_y, 0], [self.cam_x, self.cam_y, self.cam_z], self.cam_up)

        self._panel = gui.Vert()
        self._panel.background_color = o3d.visualization.gui.Color(0.21, 0.22, 0.25, 1.0)

        self.collapse = gui.CollapsableVert("Panel", 0 * self.em, gui.Margins(0 * self.em, 0, 0, 0))

        # sub-panel: 2d-3d switch
        self.vgrid = gui.VGrid(2)
        self.vgrid.add_child(gui.Label("3D View"))
        self.switch = gui.ToggleSwitch("2D View")
        self.switch.set_on_clicked(self._on_switch)
        self.vgrid.add_child(self.switch)
        self.collapse.add_child(self.vgrid)

        # sub-panel: bluesky control
        self.tabs = gui.TabControl()
        self.tab1 = gui.Vert()

        self.checkbox_wpt = gui.Checkbox("WPT")
        self.checkbox_wpt.set_on_checked(self._on_check_wpt)
        self.tab1.add_child(self.checkbox_wpt)

        self.checkbox_apt = gui.Checkbox("APT")
        self.checkbox_apt.set_on_checked(self._on_check_apt)
        self.tab1.add_child(self.checkbox_apt)

        self.checkbox_fir = gui.Checkbox("FIR")
        self.checkbox_fir.set_on_checked(self._on_check_fir)
        self.tab1.add_child(self.checkbox_fir)

        self.tabs.add_tab("Display", self.tab1)

        self.tab2 = gui.Vert()
        # self.tab2.add_stretch()
        self.tabs.add_tab("Mode", self.tab2)
        self.collapse.add_child(self.tabs)

        # sub-panel: file open
        self._fileedit = gui.TextEdit()
        filedlgbutton = gui.Button("...")
        filedlgbutton.horizontal_padding_em = 0.5
        filedlgbutton.vertical_padding_em = 0
        filedlgbutton.set_on_clicked(self._on_filedlg_button)

        # (Create the horizontal widget for the row. This will make sure the
        # text editor takes up as much space as it can.)
        fileedit_layout = gui.Horiz()
        fileedit_layout.add_child(gui.Label("Model file"))
        fileedit_layout.add_child(self._fileedit)
        fileedit_layout.add_fixed(0.25 * self.em)
        fileedit_layout.add_child(filedlgbutton)
        # add to the top-level (vertical) layout
        self.tab2.add_child(fileedit_layout)

        self._panel.add_child(self.collapse)

        self.window.add_child(self._panel)
        self.window.add_child(self._3d)

        self.window.set_on_layout(self.on_layout)

        self.view3d = True

        self.prev_x, self.prev_y = 0, 0

        self.wpt_labels = []
        self.wpt_points = None
        self.apt_labels = []
        self.apt_points = None
        self.fir_lineset = None

    def _init_coastline(self):
        self.coord_scale = 10000
        # -------------------------COASTLINE DATA--------------------------------------
        print('loading coastline data ...')

        self.m = Basemap(projection='merc', llcrnrlat=-80, urcrnrlat=80, llcrnrlon=-180, urcrnrlon=180, lat_ts=20,
                         resolution='i')
        coast = self.m.drawcoastlines()
        coordinates = np.vstack(coast.get_segments())
        coordinates = np.hstack((coordinates, np.zeros((coordinates.shape[0], 1))))

        # # convert to map projection coords.
        # # Note that lon,lat can be scalars, lists or numpy arrays.
        # xpt,ypt = m(lon,lat)  # degree
        # # set inverse=True for converting xy back to lat/lon
        # lons, lats  = m(coordinates[:, 0], coordinates[:, 1], inverse=True)
        coordinates[:, 0], coordinates[:, 1] = coordinates[:, 0] / self.coord_scale, coordinates[:,
                                                                                     1] / self.coord_scale

        x_y_h = list(coordinates)

        print("    ", len(x_y_h), " coastlines added.")

        lines = []
        for i in range(len(x_y_h) - 1):  # delete remote connected lines based on nearest distance
            d_x = x_y_h[i + 1][0] - x_y_h[i][0]
            d_y = x_y_h[i + 1][1] - x_y_h[i][1]
            if np.abs(d_x) < 20 and np.abs(d_y) < 20:
                lines.append([i, i + 1])

        colors = [[0.5, 0.5, 0.5] for _ in range(len(lines))]
        self.line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(x_y_h),
            lines=o3d.utility.Vector2iVector(lines),
        )
        self.line_set.colors = o3d.utility.Vector3dVector(colors)

        lat, lon = 52.073532, -0.607121
        xpt, ypt = self.m(lon, lat)
        self.xpt, self.ypt = xpt / self.coord_scale, ypt / self.coord_scale

    def _on_check_wpt(self, checked):
        if checked:
            self.wpt_labels = []
            points = []
            t0 = time.time()
            for i, name in enumerate(self.wpid):
                if i < 10000:
                    xpt, ypt = self.m(self.wplon[i], self.wplat[i])
                    xpt, ypt = xpt / self.coord_scale, ypt / self.coord_scale
                    points.append([xpt, ypt, 0])
                    label3d = self._3d.add_3d_label([xpt, ypt, 0], name)
                    label3d.color = gui.Color(1.0, 1.0, 1.0, 0.7)
                    self.wpt_labels.append(label3d)

            self.wpt_points = o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(points))
            colors = [[1.0, 1.0, 1.0] for _ in range(len(points))]
            self.wpt_points.colors = o3d.utility.Vector3dVector(colors)
            self._3d.scene.add_geometry("wpt_points", self.wpt_points, self.mat)

            t1 = time.time()
            print(f'{t1 - t0:.3f} s for draw 10000 labels')
        else:
            t0 = time.time()
            for wpt_label in self.wpt_labels:
                self._3d.remove_3d_label(wpt_label)
            if self.wpt_points is not None:
                self._3d.scene.remove_geometry("wpt_points")
            t1 = time.time()
            print(f'{t1 - t0:.3f} s for delete 10000 labels')

    def _on_check_apt(self, checked):
        if checked:
            self.apt_labels = []
            t0 = time.time()
            points = []
            for i, idname in enumerate(self.aptid):
                xpt, ypt = self.m(self.aptlon[i], self.aptlat[i])
                xpt, ypt = xpt / self.coord_scale, ypt / self.coord_scale
                points.append([xpt, ypt, 0])
                label3d = self._3d.add_3d_label([xpt, ypt, 0], idname)
                label3d.color = gui.Color(0.5, 0.5, 0.5, 0.7)
                self.apt_labels.append(label3d)

            self.apt_points = o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(points))
            colors = [[0.5, 0.5, 1.0] for _ in range(len(points))]
            self.apt_points.colors = o3d.utility.Vector3dVector(colors)
            self._3d.scene.add_geometry("apt_points", self.apt_points, self.mat)

            t1 = time.time()
            print(f'{t1 - t0:.3f} s for draw {len(self.aptid)} labels')
        else:
            t0 = time.time()
            for wpt_label in self.apt_labels:
                self._3d.remove_3d_label(wpt_label)
            if self.apt_points is not None:
                self._3d.scene.remove_geometry("apt_points")
            t1 = time.time()
            print(f'{t1 - t0:.3f} s for delete {len(self.aptid)} labels')

    def _on_check_fir(self, checked):
        if checked:
            points = []
            for i in range(len(self.firlat)):
                xpt, ypt = self.m(self.firlon[i], self.firlat[i])
                points.append([xpt / self.coord_scale, ypt / self.coord_scale, 0])

            lines = []
            for i in range(len(points) - 1):
                lines.append([i, i + 1])

            colors = [[0.5, 0.5, 1.0] for _ in range(len(lines))]
            self.fir_lineset = o3d.geometry.LineSet(
                points=o3d.utility.Vector3dVector(points),
                lines=o3d.utility.Vector2iVector(lines),
            )
            self.fir_lineset.colors = o3d.utility.Vector3dVector(colors)
            self._3d.scene.add_geometry("fir_lineset", self.fir_lineset, self.mat)
        else:
            if self.fir_lineset is not None:
                self._3d.scene.remove_geometry("fir_lineset")

    def _on_filedlg_button(self):
        filedlg = gui.FileDialog(gui.FileDialog.OPEN, "Select file",
                                 self.window.theme)
        filedlg.add_filter(".scn", "Scenario files, .scn")
        filedlg.add_filter("", "All files")
        filedlg.set_on_cancel(self._on_filedlg_cancel)
        filedlg.set_on_done(self._on_filedlg_done)
        self.window.show_dialog(filedlg)

    def _on_filedlg_cancel(self):
        self.window.close_dialog()

    def _on_filedlg_done(self, path):
        self._fileedit.text_value = path
        self.window.close_dialog()

    def on_layout(self, context=None):
        frame = self.window.content_rect
        # print(frame.x, frame.y, frame.width, frame.height)
        em = self.window.theme.font_size
        panel_width = 10 * em  # 20 * em
        panel_rect = gui.Rect(frame.get_right() - panel_width, frame.y,
                              panel_width, frame.height - frame.y)
        self._panel.frame = panel_rect
        self._3d.frame = gui.Rect(frame.x, frame.y, panel_rect.x - frame.x,
                                  frame.height - frame.y)

    def _on_switch(self, is_on):
        if not is_on:
            self.view3d = True
            print("Switch to 3D view")
            self._3d.look_at([self.cam_x, self.cam_y, 0], [self.cam_x, self.cam_y, self.cam_z], self.cam_up)
            self._3d.set_view_controls(gui.SceneWidget.Controls.ROTATE_CAMERA)

        else:
            self.view3d = False
            self.cam_x, self.cam_y, self.cam_z = self.cam_x0, self.cam_y0, self.cam_z0
            print("Switch to 2D view")
            self._3d.look_at([self.cam_x, self.cam_y, 0], [self.cam_x, self.cam_y, self.cam_z], self.cam_up)
            self._3d.set_view_controls(gui.SceneWidget.Controls.FLY)
            self._3d.set_on_key(self._on_key_view2d)
            self._3d.set_on_mouse(self._on_mouse_view2d)

    def _on_key_view2d(self, event):
        if not self.view3d:  # 2d view control
            print(event.type, event.key, event.type == gui.KeyEvent.Type.DOWN, event.key == gui.KeyName.A.value)

            if event.type == gui.KeyEvent.Type.DOWN and event.key == gui.KeyName.A.value:
                self._3d.look_at([self.cam_x, self.cam_y, 0], [self.cam_x, self.cam_y, self.cam_z], self.cam_up)
                self.cam_x -= 1
            if event.type == gui.KeyEvent.Type.DOWN and event.key == gui.KeyName.D.value:
                self._3d.look_at([self.cam_x, self.cam_y, 0], [self.cam_x, self.cam_y, self.cam_z], self.cam_up)
                self.cam_x += 1

            return gui.Widget.EventCallbackResult.CONSUMED

        else:
            return gui.Widget.EventCallbackResult.IGNORED

    def _on_mouse_view2d(self, event):
        if not self.view3d:  # 2d view control-- only keep translation and prohibit rotation in 2d view
            # print(event.type, event.x, event.y)
            if event.type == gui.MouseEvent.Type.MOVE:
                self.prev_x, self.prev_y = event.x, event.y

            if event.type == gui.MouseEvent.Type.DRAG:
                dx, dy = event.x - self.prev_x, event.y - self.prev_y
                self.prev_x, self.prev_y = event.x, event.y

                # o ----------> (event.x )                          ^ (y)
                # |                                                 | __________
                # |    (open3d mouse coordinate)   ====>>           ||  screen  |
                # |                                                 ||__________|
                # v (event.y )                                      o ----------> (x)
                #                                                  /
                #                                                 / (z)
                d_x, d_y = dx, -dy

                self.cam_x -= d_x
                self.cam_y -= d_y

                self._3d.look_at([self.cam_x, self.cam_y, 0], [self.cam_x, self.cam_y, self.cam_z], self.cam_up)

            if event.type == gui.MouseEvent.Type.WHEEL:
                # print(event.wheel_dx, event.wheel_dy)
                self.cam_z += event.wheel_dy * 20
                if self.cam_z < 1:
                    self.cam_z = 1

                self._3d.look_at([self.cam_x, self.cam_y, 0], [self.cam_x, self.cam_y, self.cam_z], self.cam_up)

            # note: use CONSUMED instead of IGNORED to avoid camera shaking and overwrite the functionality.
            return gui.Widget.EventCallbackResult.CONSUMED

        else:
            return gui.Widget.EventCallbackResult.IGNORED

    # ========================================================================
    # view control from bluesky command such as: PAN ZOOM etc.
    def show_file_dialog(self):
        return self._on_filedlg_button()


if __name__ == '__main__':
    blue_sky_3d = BlueSky3dUI()
    blue_sky_3d.app.run()
