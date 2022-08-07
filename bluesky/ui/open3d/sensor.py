import open3d as o3d
import open3d.visualization.gui as gui
import json
import numpy as np
import os


class Sensor:
    def __init__(self, app, em, window, m, coord_scale, _3d, line_mat, cdir):
        self.app = app
        self.em = em
        self.m = m
        self.cdir = cdir
        self.coord_scale = coord_scale
        self._3d = _3d
        self.line_mat = line_mat
        self.window = window

        self.subwindow_sensor_config = None
        self.subwindow_sensor_render = None

        self.sensor_control = gui.Vert(0.5 * self.em, gui.Margins(0, 0, 0, 0))
        self.sensor_data = {}

        self.checkboxes = gui.Horiz(spacing=6)

        self.checkbox_sensor = gui.Checkbox("Show")
        self.checkbox_sensor.enabled = False
        self.checkbox_sensor.checked = False
        self.checkbox_sensor.set_on_checked(self._on_check_sensor)

        self.checkboxes.add_child(self.checkbox_sensor)

        self.sensor_data_button = gui.Button('Show Data')
        self.sensor_data_button.vertical_padding_em = 0
        self.sensor_data_button.enabled = True
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

        self.export_button = gui.Button('Save')
        self.export_button.vertical_padding_em = 0
        self.export_button.enabled = False
        # self.export_button.set_on_clicked(self._on_export_button)

        self.create_button = gui.Button('Create')
        self.create_button.vertical_padding_em = 0
        self.create_button.enabled = True
        self.create_button.set_on_clicked(self._on_create_new_window)

        self.save_load.add_child(self.load_button)
        self.save_load.add_child(self.export_button)
        self.save_load.add_child(self.create_button)

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

    def _on_check_sensor(self, is_checked):
        if is_checked:
            # plot sensor range bounding boxes
            sensors_name = list(self.sensor_data.keys())
            for sensor_i in sensors_name:
                circlePoints, sensor_lines = self.generate_sensor_range(self.sensor_data[sensor_i])

                sensor_type = self.sensor_data[sensor_i]['model']

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

                self._3d.scene.add_geometry(f'sensor_{sensor_i}', sensor_i_set, self.line_mat)

        else:
            sensors_name = list(self.sensor_data.keys())
            for sensor_i in sensors_name:
                self._3d.scene.remove_geometry(f'sensor_{sensor_i}')
            for sensor_line_set in self.sensor_line_set:
                sensor_line_set.clear()
                sensor_line_set = None

    def _on_list(self, new_val, is_dbl_click):

        extrinsic = self.sensor_data[new_val]['extrinsic']
        self.translation_vedit.vector_value = [extrinsic['latitude'], extrinsic['longitude'], extrinsic['altitude']]
        self.rotation_vedit.vector_value = [extrinsic['pitch'], extrinsic['yaw'], extrinsic['roll']]

        print(new_val, self.translation_vedit.vector_value)

    def _on_create_new_window(self):
        # show new window
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
        # show new window
        self.subwindow_sensor_render = self.app.create_window("Sensor Stream", 720, 480)
        self._panel = gui.Vert()
        self._panel.background_color = o3d.visualization.gui.Color(0.21, 0.22, 0.25, 1.0)
        self.subwindow_sensor_render.add_child(self._panel)

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
