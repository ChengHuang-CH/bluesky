import open3d as o3d
import open3d.visualization.gui as gui
import json
import numpy as np
import scipy.spatial.transform
import pyproj
import os

import bluesky as bs
from bluesky.tools.misc import tim2txt


class Sensor:
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

        self.point_mat = o3d.visualization.rendering.MaterialRecord()
        self.point_mat.shader = 'defaultUnlit'
        self.point_mat.point_size = 8.0

        self.reference_point = [52.070378, -0.628175, 0]  # reference point for ENU coordinate frame

        self.subwindow_sensor_config = None
        self.subwindow_sensor_render = None

        self.subwindow_opened = False

        self.sensor_control = gui.Vert(0.5 * self.em, gui.Margins(0, 0, 0, 0))
        self.sensor_data = {}

        self.checkboxes = gui.Horiz(spacing=6)

        self.checkbox_sensor = gui.Checkbox("Show")
        self.checkbox_sensor.enabled = False
        self.checkbox_sensor.checked = False
        self.checkbox_sensor.set_on_checked(self._on_check_sensor)

        self.checkboxes.add_child(self.checkbox_sensor)

        self.sensor_data_button = gui.Button('Show Stream')
        self.sensor_data_button.vertical_padding_em = 0
        self.sensor_data_button.enabled = False
        self.sensor_data_button.set_on_clicked(self._on_show_sensor_button)

        self.checkboxes.add_child(self.sensor_data_button)

        self.sensor_control.add_child(self.checkboxes)

        self.sensor_list = gui.ListView()
        self.sensor_list.set_items([])
        # self.sensor_list.selected_index = self.sensor_list.selected_index + 2  # initially is -1, so now 1
        self.sensor_list.set_max_visible_items(3)
        self.sensor_list.set_on_selection_changed(self._on_list)
        self.sensor_control.add_child(self.sensor_list)

        # load and save existed sensor configuration
        self.save_load = gui.Horiz(spacing=6)

        self.load_button = gui.Button('Load')
        self.load_button.vertical_padding_em = 0
        self.load_button.set_on_clicked(self._on_load_button)

        self.create_button = gui.Button('Create')
        self.create_button.vertical_padding_em = 0
        self.create_button.enabled = True
        self.create_button.set_on_clicked(self._on_create_new_window)

        self.record_button = gui.Button('Record')
        self.record_button.vertical_padding_em = 0
        self.record_button.enabled = False
        self.record_button.background_color = gui.Color(0.0, 0.2, 0.0)
        self.record = False  # initially set button to record
        self.record_button.set_on_clicked(self._on_save_stop_button)

        self.save_load.add_child(self.load_button)
        self.save_load.add_child(self.create_button)
        self.save_load.add_child(self.record_button)

        self.sensor_control.add_child(self.save_load)

        # add, delete, revise sensor config
        self.translation_vgrid = gui.VGrid(2)
        self.translation_vgrid.add_child(gui.Label("lat-lon-alt"))
        self.translation_vedit = gui.VectorEdit()
        self.translation_vedit.vector_value = [0.0, 0.0, 0.0]
        # self.rot_vedit.set_on_value_changed(self._on_vedit)
        self.translation_vgrid.add_child(self.translation_vedit)

        self.rotation_vgrid = gui.VGrid(2)
        self.rotation_vgrid.add_child(gui.Label("pit-yaw-rol"))
        self.rotation_vedit = gui.VectorEdit()
        self.rotation_vedit.vector_value = [0.0, 0.0, 0.0]
        # self.rot_vedit.set_on_value_changed(self._on_vedit)
        self.rotation_vgrid.add_child(self.rotation_vedit)

        self.sensor_control.add_child(self.translation_vgrid)
        self.sensor_control.add_child(self.rotation_vgrid)

        self.sensor_line_set = []

        self.current_lat = None
        self.current_lon = None
        self.current_alt = None
        self.current_hdg = None
        self.ntraf = None

        self.file_to_save = None

    def _on_check_sensor(self, is_checked):
        if is_checked:
            # plot sensor range bounding boxes
            sensors_name = list(self.sensor_data.keys())
            for sensor_name in sensors_name:
                circlePoints, sensor_lines = self.generate_sensor_range(self.sensor_data[sensor_name])

                sensor_type = self.sensor_data[sensor_name]['model']

                if sensor_type == 'camera':
                    color = [0.14, 0.8, 0.64]
                elif sensor_type == 'radar':
                    color = [0.048, 0.332, 0.868]
                elif sensor_type == 'lidar':
                    color = [0.94, 0.94, 0.08]
                else:
                    color = [1.0, 1.0, 1.0]

                colors = [color for _ in range(len(sensor_lines))]
                sensor_i_set = o3d.geometry.LineSet(
                    points=o3d.utility.Vector3dVector(circlePoints),
                    lines=o3d.utility.Vector2iVector(sensor_lines),
                )
                sensor_i_set.colors = o3d.utility.Vector3dVector(colors)

                self.sensor_line_set.append(sensor_i_set)
                self._3d.scene.add_geometry(f'sensor_{sensor_name}', sensor_i_set, self.line_mat)

        else:
            sensors_name = list(self.sensor_data.keys())
            for sensor_name in sensors_name:
                self._3d.scene.remove_geometry(f'sensor_{sensor_name}')
            for sensor_line_set in self.sensor_line_set:
                sensor_line_set.clear()
                sensor_line_set = None

    def _on_list(self, new_val, is_dbl_click):

        extrinsic = self.sensor_data[new_val]['extrinsic']
        self.translation_vedit.vector_value = [extrinsic['latitude'], extrinsic['longitude'], extrinsic['altitude']]
        self.rotation_vedit.vector_value = [extrinsic['pitch'], extrinsic['yaw'], extrinsic['roll']]

        print(new_val, self.translation_vedit.vector_value)

    def _on_save_stop_button(self):
        if not self.record:  # start recording. the set button text to stop for next click
            self.record = True
            self.record_button.background_color = gui.Color(0.2, 0.0, 0.0)
            self.record_button.text = 'Stop'
        else:
            # stop recording. then reset button to start
            self.record = False
            self.record_button.background_color = gui.Color(0.0, 0.2, 0.0)
            self.record_button.text = 'Record'

    def _on_create_new_window(self):
        # show new window
        #

        self.subwindow_sensor_config = self.app.create_window("Sensor Configuration", 700, 500)

        self.sensor_config_list_view = gui.Vert(0.5 * self.em, gui.Margins(0, 0.5 * self.em, 0, 0))
        self.sensor_config_list_view.background_color = o3d.visualization.gui.Color(0.21, 0.22, 0.25, 1.0)

        self.sensor_model_list_view = gui.Vert(0.5 * self.em, gui.Margins(0, 0.5 * self.em, 0, 0))
        self.sensor_model_list_view.background_color = o3d.visualization.gui.Color(0.25, 0.24, 0.35, 1.0)

        self.subwindow_sensor_config.add_child(self.sensor_config_list_view)
        self.subwindow_sensor_config.add_child(self.sensor_model_list_view)
        self.subwindow_sensor_config.set_on_layout(self.sensors_config_on_layout)

        # config list view ------------------------------------------------------------
        self.config_title = gui.Label("Sensor Configuration List")
        self.config_title.text_color = gui.Color(1.0, 0.5, 0.0)

        self.sensor_config_list_view.add_child(self.config_title)

        # sensor model list view ------------------------------------------------------
        self.sensor_model_title = gui.Label("Sensor Model List")
        self.sensor_model_title.text_color = gui.Color(1.0, 0.5, 0.0)

        self.sensor_model_list_view.add_child(self.sensor_model_title)

        self.sensor_model_proxy = gui.WidgetProxy()
        self.sensor_model_list_view.add_child(self.sensor_model_proxy)

        self.sensor_model_proxy1 = gui.WidgetProxy()
        self.sensor_model_list_view.add_child(self.sensor_model_proxy1)

        camera_test, rm_ = self.sensor_model_container("Camera PTZ")
        camera_test1, rm_1 = self.sensor_model_container("Camera PTZ1")

        self.sensor_model_proxy.set_widget(camera_test)
        self.sensor_model_proxy1.set_widget(camera_test1)

        rm_.set_on_clicked(self.switch_proxy)
        rm_1.set_on_clicked(self.switch_proxy1)

    def switch_proxy(self):
        self.sensor_model_proxy.set_widget(None)
        self.subwindow_sensor_config.set_needs_layout()

    def switch_proxy1(self):
        self.sensor_model_proxy1.set_widget(None)
        self.subwindow_sensor_config.set_needs_layout()

    def sensor_model_container(self, name):
        sensor_type = 'camera'

        constainer = gui.Vert(0.3 * self.em)

        split_line = gui.Label('_' * 35)

        model_name = gui.Label(f'sensor name: {name}')
        model_name.text_color = gui.Color(1.0, 1.0, 1.0)

        model_type = gui.Label(f'sensor type: {sensor_type}')
        model_type.text_color = gui.Color(1.0, 1.0, 1.0)

        add_ = gui.Button('<-- add to left list')
        add_.vertical_padding_em = 0
        add_.background_color = gui.Color(0.0, 0.5, 0.0)

        rm_ = gui.Button('delete this model')
        rm_.vertical_padding_em = 0
        rm_.background_color = gui.Color(0.8, 0.0, 0.0)

        intrinsic = gui.VGrid(2)
        paras = []
        if sensor_type == 'camera':
            paras.append(gui.Label('img_width'))
            paras.append(gui.Label(f'{1920}'))
            paras.append(gui.Label('img_height'))
            paras.append(gui.Label(f'{1080}'))
            paras.append(gui.Label('fov'))
            paras.append(gui.Label(f'{30}'))
            paras.append(gui.Label('range'))
            paras.append(gui.Label(f'{None}'))
        elif sensor_type == 'radar':
            paras.append(gui.Label('horizontal_fov'))
            paras.append(gui.Label(f'{30}'))
            paras.append(gui.Label('vertical_fov'))
            paras.append(gui.Label(f'{10}'))
            paras.append(gui.Label('range'))
            paras.append(gui.Label(f'{150}'))
        elif sensor_type == 'lidar':
            paras.append(gui.Label('horizontal_fov'))
            paras.append(gui.Label(f'{360}'))
            paras.append(gui.Label('upper_fov'))
            paras.append(gui.Label(f'{30}'))
            paras.append(gui.Label('lower_fov'))
            paras.append(gui.Label(f'{-10}'))
            paras.append(gui.Label('range'))
            paras.append(gui.Label(f'{150}'))
        else:
            print('Not implemented sensor model!')

        for para in paras:
            intrinsic.add_child(para)

        constainer.add_child(split_line)
        constainer.add_child(model_name)
        constainer.add_child(model_type)
        constainer.add_child(add_)
        constainer.add_child(intrinsic)
        constainer.add_child(rm_)

        return constainer, rm_

    def sensors_config_on_layout(self, context=None):
        r = self.subwindow_sensor_config.content_rect
        # print(frame.x, frame.y, frame.width, frame.height)  # default: 0 0 1080 720
        self.sensor_config_list_view.frame = gui.Rect(r.x, r.y, r.width / 3 * 2, r.height)
        self.sensor_model_list_view.frame = gui.Rect(r.x + r.width / 3 * 2 + 1, r.y, r.width / 3, r.height)

    def _on_show_sensor_button(self):
        # show new window for real-time data streaming
        self.subwindow_sensor_render = self.app.create_window("Sensor Stream", 1080, 420)

        self.subwindow_opened = True
        self.subwindow_sensor_render.set_on_close(self._on_redner_window_closing)

        self.sensor_names = list(self.sensor_data.keys())
        self.n_sensors = len(self.sensor_names)
        self._3d_wins = [gui.SceneWidget() for _ in range(self.n_sensors)]
        for i in range(self.n_sensors):
            self._3d_wins[i].scene = o3d.visualization.rendering.Open3DScene(self.subwindow_sensor_render.renderer)
            self._3d_wins[i].scene.set_background([0.5, 0.5, 0.5, 1.0])
            self._3d_wins[i].scene.show_axes(True)
            bounds = self._3d_wins[i].scene.bounding_box
            center = bounds.get_center()
            self._3d_wins[i].setup_camera(60, bounds, center)
            self._3d_wins[i].look_at([0, 0, 0], [0, 0, 100], [0, 1, 0])

        for i in range(self.n_sensors):
            self.subwindow_sensor_render.add_child(self._3d_wins[i])

        self.subwindow_sensor_render.set_on_layout(self._subwin_on_layout)

        # initialize sensors transformation matrix at first
        for _idx, sensor_name in enumerate(self.sensor_names):
            sensor_transform = self.sensor_transform(self.sensor_data[sensor_name])
            self.sensor_data[sensor_name]['world2sensor'] = sensor_transform

            # initialize sensor boundary
            if self.sensor_data[sensor_name]['model'] == 'camera':
                camera_bound_lineset, o3d_cam_focus = self.camera_container(self.sensor_data[sensor_name])
                self._3d_wins[_idx].scene.add_geometry(f'{sensor_name}_bound', camera_bound_lineset,
                                                       self.line_mat_thick)
                self._3d_wins[_idx].look_at([o3d_cam_focus[0], o3d_cam_focus[1], 0],
                                            [o3d_cam_focus[0], o3d_cam_focus[1], 100],
                                            [0, 1, 0])
                self._3d_wins[_idx].set_on_mouse(self._on_mouse_view2d)

            if self.sensor_data[sensor_name]['model'] == 'radar':
                radar_bound_lineset0, o3d_cam_focus = self.radar_container(self.sensor_data[sensor_name])
                self._3d_wins[_idx].scene.add_geometry(f'{sensor_name}_bound', radar_bound_lineset0,
                                                       self.line_mat_thick)
                self._3d_wins[_idx].look_at([o3d_cam_focus[0], o3d_cam_focus[1], 0],
                                            [o3d_cam_focus[0], o3d_cam_focus[1], 90],
                                            [0, 1, 0])
                self._3d_wins[_idx].set_on_mouse(self._on_mouse_view2d)

    def _subwin_on_layout(self, context=None):
        frame = self.subwindow_sensor_render.content_rect
        grid_col = 3
        # https://stackoverflow.com/questions/2356501/how-do-you-round-up-a-number
        grid_row = self.n_sensors // grid_col + (self.n_sensors % grid_col > 0)

        grid_w = frame.width / grid_col
        grid_h = frame.height / grid_row

        for i in range(self.n_sensors):
            row_i = i // grid_col
            col_i = i % grid_col
            self._3d_wins[i].frame = gui.Rect(frame.x + col_i * grid_w + 1,
                                              frame.y + row_i * grid_h + 1,
                                              grid_w - 1,
                                              grid_h - 1)

    def _on_redner_window_closing(self):
        self.subwindow_opened = False
        return True

    @staticmethod
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

    @staticmethod
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

    def _on_mouse_view2d(self, event):
        return gui.Widget.EventCallbackResult.CONSUMED

    def update_sensor_stream(self):
        """

        """
        self.current_lat = bs.traf.lat
        self.current_lon = bs.traf.lon
        self.current_alt = bs.traf.alt
        self.current_hdg = bs.traf.hdg
        self.ntraf = bs.traf.ntraf

        if self.ntraf > 0:
            aircraft_point = [self.current_lat[0], self.current_lon[0], self.current_alt[0]]

            for _idx, sensor_name in enumerate(self.sensor_names):
                # remove geometry of last frame at first
                try:
                    self._3d_wins[_idx].scene.remove_geometry(f'{sensor_name}_points')
                except:
                    pass

                # project targets into each sensor frame
                sensor_transform = self.sensor_data[sensor_name]['world2sensor']
                point_in_sensor = self.point_in_sensor_frame(aircraft_point, sensor_transform,
                                                             self.sensor_data[sensor_name])
                print(f'[{tim2txt(bs.sim.simt)}] point in {sensor_name}: {point_in_sensor}')

                if self.record:
                    # save data and append to files
                    print('recording')

                # camera
                if self.sensor_data[sensor_name]['model'] == 'camera' and point_in_sensor is not None:
                    # point_in_sensor = [u,v]
                    xyz = self.uv2xyz(point_in_sensor)
                    xyz = [xyz]

                    target_point_set = o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(xyz))
                    target_point_set.colors = o3d.utility.Vector3dVector([[1, 0, 0] for _ in range(len(xyz))])
                    self._3d_wins[_idx].scene.add_geometry(f'{sensor_name}_points', target_point_set, self.point_mat)

                if self.sensor_data[sensor_name]['model'] == 'radar' and point_in_sensor is not None:
                    # point_in_sensor = [range,theta]
                    xyz = self.polar2xyz(point_in_sensor)
                    xyz = [xyz]

                    target_point_set = o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(xyz))
                    target_point_set.colors = o3d.utility.Vector3dVector([[1, 0, 0] for _ in range(len(xyz))])
                    self._3d_wins[_idx].scene.add_geometry(f'{sensor_name}_points', target_point_set, self.point_mat)

            print('-' * 50)

    @staticmethod
    def uv2xyz(u_v):
        x = u_v[0] / 10
        y = 108 - u_v[1] / 10
        z = 0
        return [x, y, z]

    @staticmethod
    def polar2xyz(range_theta):
        x = range_theta[0] * np.cos(np.deg2rad(range_theta[1]))
        y = range_theta[0] * np.sin(np.deg2rad(range_theta[1]))
        z = 0  # plot view

        return [x, y, z]

    def _on_load_button(self):
        filedlg = gui.FileDialog(gui.FileDialog.OPEN, "Select file",
                                 self.window.theme)
        filedlg.add_filter(".json", "Sensor Config File (.json)")
        filedlg.add_filter("", "All files")
        filedlg.set_on_cancel(self._on_filedlg_cancel)
        filedlg.set_on_done(self._on_filedlg_done)
        self.window.show_dialog(filedlg)

    def _on_filedlg_cancel(self):
        self.window.close_dialog()

    def _on_filedlg_done(self, path):
        # self.export_name_edit.text_value = path
        self.window.close_dialog()

        with open(path, 'r') as data_file:
            self.sensor_data = json.load(data_file)

        sensor_names = list(self.sensor_data.keys())
        self.sensor_list.set_items(sensor_names)

        if sensor_names:
            self.checkbox_sensor.enabled = True  # enable checkbox after loading valid sensors
            self.sensor_data_button.enabled = True
            self.record_button.enabled = True

        os.chdir(self.cdir)  # remember to change the dit path back to avoid reset issue

    def _on_export_button(self):
        txt_name = self.export_name_edit.text_value
        try:
            with open(txt_name, 'w') as txtFile:
                txtFile.write(
                    f'{list(self.tran_vedit.vector_value) + list(self.rot_vedit.vector_value)}')
        except Exception as e:
            print(f'Input /path/file_name.txt at first!')

    def generate_sensor_range(self, sensor_i_data):

        radius = sensor_i_data['intrinsic']['range']
        if radius is None:
            radius = 200

        centerLat = sensor_i_data['extrinsic']['latitude']  # latitude of circle center, decimal degrees
        centerLon = sensor_i_data['extrinsic']['longitude']  # Longitude of circle center, decimal degrees

        center_x, center_y = self.m(centerLon, centerLat)
        center_x, center_y = center_x / self.coord_scale, center_y / self.coord_scale

        alt = 0  # set alt to 0 for plotting

        yaw = sensor_i_data['extrinsic']['yaw']
        if sensor_i_data['model'] == 'camera':
            fov = sensor_i_data['intrinsic']['fov']
        else:
            fov = sensor_i_data['intrinsic']['horizontal_fov']

        assert fov <= 360

        angle_left = yaw - fov / 2
        angle_right = yaw + fov / 2

        # parameters
        N = fov * 2  # number of discrete sample points to be generated along the circle

        theta = np.linspace(angle_left, angle_right, N)

        # generate points
        circlePoints = []

        if fov != 360:
            circlePoints.append([center_x, center_y, alt])  # append center points at first

        for k in range(N):
            # compute
            angle = np.deg2rad(theta[k])

            dx = radius * np.cos(angle)
            dy = radius * np.sin(angle)
            lat = centerLat + (180 / np.pi) * (dy / 6378137)
            lon = centerLon + (180 / np.pi) * (dx / 6378137) / np.cos(centerLat * np.pi / 180)

            x, y = self.m(lon, lat)
            x, y = x / self.coord_scale, y / self.coord_scale
            point = [x, y, 0]

            # add to list
            circlePoints.append(point)

        if fov != 360:
            circlePoints.append([center_x, center_y, alt])  # append center points at last

        lines = []
        for i in range(len(circlePoints) - 1):
            lines.append([i, i + 1])

        return circlePoints, lines

    @staticmethod
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

    def sensor_transform(self, sensor_i_data):

        centerLat = sensor_i_data['extrinsic']['latitude']  # decimal degrees
        centerLon = sensor_i_data['extrinsic']['longitude']  # decimal degrees
        centerAlt = sensor_i_data['extrinsic']['altitude']  #

        sensor_point = [centerLat, centerLon, centerAlt]

        sensor_pt = self.geodetic2enu(sensor_point[0], sensor_point[1], sensor_point[2],
                                      self.reference_point[0], self.reference_point[1], self.reference_point[2])

        sensor_pitch, sensor_yaw, sensor_roll = sensor_i_data['extrinsic']['pitch'], \
                                                sensor_i_data['extrinsic']['yaw'], \
                                                sensor_i_data['extrinsic']['roll']  # r_y, r_z, r_x

        pitch, yaw, roll = np.deg2rad(sensor_pitch), np.deg2rad(sensor_yaw), np.deg2rad(sensor_roll)

        # http://brainvoyager.com/bv/doc/UsersGuide/CoordsAndTransforms/SpatialTransformationMatrices.html
        rx = np.array([[1, 0, 0], [0, np.cos(roll), np.sin(roll)], [0, -np.sin(roll), np.cos(roll)]])
        ry = np.array([[np.cos(pitch), 0, -np.sin(pitch)], [0, 1, 0], [np.sin(pitch), 0, np.cos(pitch)]])
        rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])

        rotation_matrix = rz.dot(ry).dot(rx)

        translation = np.array([[sensor_pt[0]], [sensor_pt[1]], [sensor_pt[2]]])
        sensor2world = np.hstack((rotation_matrix, translation))
        sensor2world = np.vstack((sensor2world, np.array([[0, 0, 0, 1]])))

        world2sensor = np.linalg.inv(sensor2world)  # important!!

        return world2sensor

    def point_in_sensor_frame(self, lat_lon_alt, world2sensor, sensor_i_data):
        sensor_pt = self.geodetic2enu(lat_lon_alt[0], lat_lon_alt[1], lat_lon_alt[2],
                                      self.reference_point[0], self.reference_point[1], self.reference_point[2])
        sensor_pt = np.array([[sensor_pt[0]], [sensor_pt[1]], [sensor_pt[2]], [1]])  # 4x1

        pt_in_sensor = np.dot(world2sensor, sensor_pt)  # 3d; 4x1 [[x],[y],[z],[1]]

        # project to sensor plane
        if sensor_i_data['model'] == 'lidar':
            pt_in_sensor = [pt_in_sensor[0][0], pt_in_sensor[1][0], pt_in_sensor[2][0]]  # convert nested array to list
            range = sensor_i_data['intrinsic']['range']
            dist = np.sqrt(pt_in_sensor[0] ** 2 + pt_in_sensor[1] ** 2 + pt_in_sensor[2] ** 2)
            if dist > range:
                pt_in_sensor = None

        elif sensor_i_data['model'] == 'camera':

            # Build the K projection matrix:
            # K = [[Fx,  0, image_w/2],
            #      [ 0, Fy, image_h/2],
            #      [ 0,  0,         1]]
            image_w = sensor_i_data['intrinsic']['img_width']
            image_h = sensor_i_data['intrinsic']['img_height']
            fov = sensor_i_data['intrinsic']['fov']
            focal = image_w / (2.0 * np.tan(fov * np.pi / 360.0))

            # In this case Fx and Fy are the same since the pixel aspect
            # ratio is 1
            K = np.identity(3)
            K[0, 0] = K[1, 1] = focal
            K[0, 2] = image_w / 2.0
            K[1, 2] = image_h / 2.0

            # 3d camera coordinate to opencv coordinate:
            #            ^ z                       . z
            #            |                        /
            #            |              to:      +-------> x
            #            | . x                   |
            #            |/                      |
            # y <------- +                       v y
            # (x, y, z) --> (-y, -z, x)
            dist = np.sqrt(pt_in_sensor[0] ** 2 + pt_in_sensor[1] ** 2 + pt_in_sensor[2] ** 2)
            # print(f'dist: {dist}')

            point_in_camera_coords = np.array([
                pt_in_sensor[1] * -1,
                pt_in_sensor[2] * -1,
                pt_in_sensor[0]])

            # Finally we can use our K matrix to do the actual 3D -> 2D.
            points_2d = np.dot(K, point_in_camera_coords)

            # Remember to normalize the x, y values by the 3rd value.
            points_2d = np.array([
                points_2d[0, :] / points_2d[2, :],
                points_2d[1, :] / points_2d[2, :],
                points_2d[2, :]])

            # print(f'image plane: {points_2d}')

            points_2d = points_2d.T
            points_in_canvas_mask = \
                (points_2d[:, 0] > 0.0) & (points_2d[:, 0] < image_w) & \
                (points_2d[:, 1] > 0.0) & (points_2d[:, 1] < image_h) & \
                (points_2d[:, 2] > 0.0)
            points_2d = points_2d[points_in_canvas_mask]

            # Extract the screen coords (uv) as integers.
            u = points_2d[:, 0].astype(int)
            v = points_2d[:, 1].astype(int)

            u = list(u)
            v = list(v)

            if not u or not v or dist[0] > 300:
                pt_in_sensor = None

            # =========================================
            if u or v:  # for single point test
                u = u[0]
                v = v[0]

                pt_in_sensor = [u, v]
            # =============================================

            if dist[0] > 300:
                pt_in_sensor = None

        elif sensor_i_data['model'] == 'radar':
            # pt_in_sensor: 4x1 [[x],[y],[z],[1]]
            pt_in_sensor = pt_in_sensor.squeeze(axis=1)  # 4x1 [[x],[y],[z],[1]] -> 4 [x,y,z,1]
            x = pt_in_sensor[0]
            if pt_in_sensor[0] < 0.001:  # include negative
                pt_in_sensor = None
            else:
                r = np.sqrt(pt_in_sensor[0] ** 2 + pt_in_sensor[1] ** 2)
                theta = np.arctan(pt_in_sensor[1] / pt_in_sensor[0])
                theta = np.rad2deg(theta)

                horizontal_fov = sensor_i_data['intrinsic']['horizontal_fov']
                range = sensor_i_data['intrinsic']['range']

                pt_in_sensor = [r, theta]  # [meter, degree]

                if r > range or theta < -horizontal_fov / 2 or theta > horizontal_fov / 2:
                    r, theta = None, None  # out of detection region
                    pt_in_sensor = None

        return pt_in_sensor
