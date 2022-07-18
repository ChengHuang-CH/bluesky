import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
from mpl_toolkits.basemap import Basemap
import time
import os
import subprocess

import bluesky as bs
from bluesky import stack
from bluesky import settings
from bluesky.navdatabase import Navdatabase
from bluesky.tools import geo, areafilter
from bluesky.tools.aero import ft, kts, nm
from bluesky.tools.misc import tim2txt

# colors
black = (0, 0, 0)
darkgrey = (25 / 255, 25 / 255, 48 / 255)
grey = (84 / 255, 84 / 255, 114 / 255)
darkblue = (25 / 255, 64 / 255, 100 / 255)
white = (1.0, 1.0, 1.0)
green = (0, 1.0, 0)
blue = (0, 0, 1.0)
red = (1.0, 0, 0)
cyan = (0, 150 / 255, 150 / 255)
lightgreyblue = (130 / 255, 150 / 255, 190 / 255)  # waypoint symbol color
lightgreygreen = (149 / 255, 215 / 255, 179 / 255)  # grid color
lightcyan = (0, 1.0, 1.0)  # FIR boundaries
amber = (255 / 255, 163 / 255, 71 / 255)  # Conflicting aircraft
magenta = (1.0, 0, 1.0)  # Used for route


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
        # stack.stack(f'IC')

        # ===============================================================
        self.cdir = os.getcwd()

        self._init_coastline()

        # create open3d window
        self.app = gui.Application.instance
        self.app.initialize()
        self.window = self.app.create_window("BlueSky3D", 1080, 720)
        self.em = self.window.theme.font_size

        self.mat = o3d.visualization.rendering.MaterialRecord()
        self.mat.shader = "defaultLit"
        self.mat.base_color = [1.0, 1.0, 1.0, 1.0]

        self.line_mat = o3d.visualization.rendering.MaterialRecord()
        self.line_mat.shader = "unlitLine"

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

        self.cam_x0, self.cam_y0, self.cam_z0 = self.xpt, self.ypt, 1000
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

        self.collapse = gui.CollapsableVert("Panel", 0.3 * self.em, gui.Margins(0, 0, 0, 0))

        # sub-panel: 2d-3d switch
        self.hgrid = gui.Horiz(2)
        self.hgrid.add_child(gui.Label("3D View"))
        self.switch = gui.ToggleSwitch("2D View")
        self.switch.set_on_clicked(self._on_switch)
        self.switch.is_on = True  # default 2d
        self._on_switch(is_on=True)  # default 2d
        self.hgrid.add_child(self.switch)
        self.collapse.add_child(self.hgrid)

        # sub-panel: bluesky control
        self.tabs = gui.TabControl()
        self.tab1 = gui.Vert()
        self.tabs.add_tab("Display", self.tab1)

        self.checkbox_wpt = gui.Checkbox("WPT")
        self.checkbox_wpt.set_on_checked(self._on_check_wpt)
        self.tab1.add_child(self.checkbox_wpt)

        self.checkbox_apt = gui.Checkbox("APT")
        self.checkbox_apt.set_on_checked(self._on_check_apt)
        self.tab1.add_child(self.checkbox_apt)

        self.checkbox_fir = gui.Checkbox("FIR")
        self.checkbox_fir.set_on_checked(self._on_check_fir)
        self.tab1.add_child(self.checkbox_fir)

        self.tab2 = gui.Vert()
        # self.tab2.add_stretch()
        self.tabs.add_tab("Mode", self.tab2)
        self.collapse.add_child(self.tabs)

        # file open
        self._fileedit = gui.TextEdit()
        filedlgbutton = gui.Button("...")
        filedlgbutton.horizontal_padding_em = 0.5
        filedlgbutton.vertical_padding_em = 0
        filedlgbutton.set_on_clicked(self._on_filedlg_button)

        # (Create the horizontal widget for the row. This will make sure the
        # text editor takes up as much space as it can.)
        fileedit_layout = gui.Horiz()
        fileedit_layout.add_child(gui.Label("SCN File"))
        fileedit_layout.add_child(self._fileedit)
        fileedit_layout.add_fixed(0.25 * self.em)
        fileedit_layout.add_child(filedlgbutton)
        # add to the top-level (vertical) layout
        self.tab2.add_child(fileedit_layout)

        # sub-panel: uam configuration
        self.uam_config_tab = gui.TabControl()
        self.uam_config = gui.Vert()
        self.uam_config_tab.add_tab("UAM configuration", self.uam_config)

        self.checkbox_osm = gui.Checkbox("Osm")
        # self.checkbox_osm.set_on_checked(self._on_check_osm)
        self.uam_config.add_child(self.checkbox_osm)

        self.checkbox_turbulence = gui.Checkbox("Turbulence")
        # self.checkbox_turbulence.set_on_checked(self._on_check_tur)
        self.uam_config.add_child(self.checkbox_turbulence)

        self.checkbox_ground = gui.Checkbox("Ground traffic")
        # self.checkbox_ground.set_on_checked(self._on_check_ground)
        self.uam_config.add_child(self.checkbox_ground)

        self.collapse.add_child(self.uam_config_tab)

        # sub-panel: info
        self.info_line = gui.TabControl()
        self.info = gui.VGrid(2)

        self.info.add_child(gui.Label("UTC"))
        self.utc = gui.Label("00:00:00")
        self.info.add_child(self.utc)

        self.info.add_child(gui.Label("sim t"))
        self.simt = gui.Label("0.000")
        self.info.add_child(self.simt)

        self.info.add_child(gui.Label("ntraf"))
        self.ntraf = gui.Label("0")
        self.info.add_child(self.ntraf)

        self.info.add_child(gui.Label("Freq"))
        self.freq = gui.Label("0")
        self.info.add_child(self.freq)

        self.info.add_child(gui.Label("#LOS"))
        self.los = gui.Label("0")
        self.info.add_child(self.los)

        self.info.add_child(gui.Label("Total LOS"))
        self.total_los = gui.Label("0")
        self.info.add_child(self.total_los)

        self.info.add_child(gui.Label("#Con"))
        self.con = gui.Label("0")
        self.info.add_child(self.con)

        self.info.add_child(gui.Label("Total Con"))
        self.total_con = gui.Label("0")
        self.info.add_child(self.total_con)

        self.info_line.add_tab("Info", self.info)
        self.collapse.add_child(self.info_line)

        # sub-panel: console
        self.consoleline = gui.TabControl()
        self.console = gui.Vert(3 * self.em)
        self.consoleline.add_tab("Console", self.console)
        self.collapse.add_child(self.consoleline)
        # text color
        self.red = gui.Color(1.0, 0.0, 0.0)
        self.orange = gui.Color(1.0, 0.5, 0.0)
        self.green = gui.Color(0.0, 1.0, 0.0)
        # keep 3 line history context
        self._lines_text = ['>> ', '>> ', '>> ']
        self._line1 = gui.Label(self._lines_text[0])
        self._line1.text_color = self.orange
        self._line2 = gui.Label(self._lines_text[1])
        self._line2.text_color = self.orange
        self._line3 = gui.Label(self._lines_text[2])
        self._line3.text_color = self.green
        self.tedit = gui.TextEdit()
        self.tedit.placeholder_text = ">> Edit me some text here"
        # on_text_changed fires whenever the user changes the text (but not if
        # the text_value property is assigned to).
        # self.tedit.set_on_text_changed(self._on_text_changed)
        # on_value_changed fires whenever the user signals that they are finished
        # editing the text, either by pressing return or by clicking outside of
        # the text editor, thus losing text focus.
        self.tedit.set_on_value_changed(self._on_value_changed)

        self.console.add_child(self._line1)
        self.console.add_child(self._line2)
        self.console.add_child(self._line3)
        self.console.add_child(self.tedit)

        self._panel.add_child(self.collapse)

        # console
        # self._console = gui.CollapsableVert("Console", 0 * self.em, gui.Margins(0 * self.em, 0, 0, 0))

        self.window.add_child(self._panel)
        # self.window.add_child(self._console)
        self.window.add_child(self._3d)

        self.window.set_on_layout(self.on_layout)

        self.is_done = False
        self.file_done = False
        self.file_path = ''

        self.window.set_on_close(self._on_main_window_closing)

        self.prev_x, self.prev_y = 0, 0

        self.wpt_labels = []
        self.wpt_points = None
        self.apt_labels = []
        self.apt_points = None

        self.fir_lineset = None
        self.poly_set = None
        self.selected_routes_set = None
        self.trails_set = None
        self.bgtrails_set = None
        self.traverse_wp_labels = []

        # User defined background objects
        self.objtype = []
        self.objcolor = []
        self.objdata = []
        self.objname = []

        # Keep track of sim dt to show average update rate on screen
        self.dts = []

        self.swnavdisp = False
        self.ndacid = ""
        self.ndlat = 0.0
        self.ndlon = 0.0
        self.ndhdg = 0.0

        self._create_aircraft_model()

        # Route drawing for which acid? "" =  None
        self.acidrte = ""
        self.rtewpid = []
        self.rtewplabel = []

    def _init_coastline(self):
        self.coord_scale = 1000
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
        coordinates[:, 0], coordinates[:, 1] = coordinates[:, 0] / self.coord_scale, \
                                               coordinates[:, 1] / self.coord_scale

        x_y_h = list(coordinates)

        print("    ", len(x_y_h), " coastlines added.")

        lines = []
        for i in range(len(x_y_h) - 1):  # delete remote connected lines based on nearest distance
            d_x = x_y_h[i + 1][0] - x_y_h[i][0]
            d_y = x_y_h[i + 1][1] - x_y_h[i][1]
            if np.abs(d_x) < 200 and np.abs(d_y) < 200:
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

    def _create_aircraft_model(self):
        # aircraft LineSet
        #   /\
        #  /  \
        # / __ \
        self.aircraft_points = [[0, 0, 0], [0.01, 0.02, 0], [0.02, 0, 0], [0.01, 0.01, 0]]
        self.aircraft_lines = [[0, 1], [1, 2], [2, 3], [3, 0]]
        self.aircraft_colors_green = [green for _ in range(len(self.aircraft_lines))]
        self.aircraft_colors_amber = [amber for _ in range(len(self.aircraft_lines))]

        self.aircraft_labels = []
        self.all_aircraft_lineset = None

    def update(self):
        """

        @return:
        """
        # Simulation: keep track of timestep
        # For measuring game loop frequency
        self.dts.append(bs.sim.simdt)
        if len(self.dts) > 20:
            del self.dts[0]

        # ------ reset --------------------
        # if self.aircraft_name_list:
        if self.all_aircraft_lineset is not None:  # remove aircraft object
            self._3d.scene.remove_geometry('all_aircraft')
            self.all_aircraft_lineset.clear()
            self.all_aircraft_lineset = None

        if self.selected_routes_set is not None:  # remove selected routes
            self._3d.scene.remove_geometry('selected_routes')
            self.selected_routes_set.clear()
            self.selected_routes_set = None

        for wp_label in self.traverse_wp_labels:  # remove traversed waypoints label of selected routes
            self._3d.remove_3d_label(wp_label)
        self.traverse_wp_labels = []

        for aft_label in self.aircraft_labels:  # remove aircraft labels
            self._3d.remove_3d_label(aft_label)
        self.aircraft_labels = []

        if self.trails_set is not None:  # remove selected routes
            self._3d.scene.remove_geometry('draw_trials')
            self.trails_set.clear()
            self.trails_set = None

        if self.bgtrails_set is not None:  # remove selected routes
            self._3d.scene.remove_geometry('draw_bgtrials')
            self.bgtrails_set.clear()
            self.bgtrails_set = None

        # ---------- Draw background trails ----------
        # if bs.traf.trails.active:
        #     bs.traf.trails.buffer()  # move all new trails to background
        #
        #     bglat0 = bs.traf.trails.bglat0.tolist()
        #     bgtrails_points = []
        #     bgtrails_lines = []
        #     bgtrails_colors = []
        #
        #     print(len(bs.traf.trails.bglon0), len(bs.traf.trails.bglat0),
        #           len(bs.traf.trails.bglon1), len(bs.traf.trails.bglat1))
        #     for i in range(len(bglat0)):
        #         x0, y0 = self.m(bs.traf.trails.bglon0[i], bs.traf.trails.bglat0[i])
        #         x1, y1 = self.m(bs.traf.trails.bglon1[i], bs.traf.trails.bglat1[i])
        #         x0, y0 = x0 / self.coord_scale, y0 / self.coord_scale
        #         x1, y1 = x1 / self.coord_scale, y1 / self.coord_scale
        #         bgtrails_points.append([x0, y0, 0])
        #         bgtrails_points.append([x1, y1, 0])
        #
        #         bgtrails_colors.append(bs.traf.trails.bgcol[i] / 255)
        #
        #     for i_ in range(0, len(bgtrails_points), 2):
        #         bgtrails_lines.append([i_, i_ + 1])
        #
        #     self.bgtrails_set = o3d.geometry.LineSet(
        #         points=o3d.utility.Vector3dVector(bgtrails_points),
        #         lines=o3d.utility.Vector2iVector(bgtrails_lines),
        #     )
        #     self.bgtrails_set.colors = o3d.utility.Vector3dVector(bgtrails_colors)
        #     self._3d.scene.add_geometry(f'draw_bgtrials', self.bgtrails_set, self.line_mat)

        # ---------- User defined objects update ------------
        if self.poly_set is not None:
            self._3d.scene.remove_geometry("poly_set")
            self.poly_set.clear()
            self.poly_set = None

        points = []
        lines = []
        colors = []
        first_pt_index = None
        last_pt_index = None
        # enumerate every input object
        for i in range(len(self.objtype)):
            # print(f'{i}: {self.objtype[i], self.objname[i], self.objdata[i]}')

            # Draw LINE or POLYGON with objdata = [lat0,lon0,lat1,lon1,lat2,lon2,..]
            if self.objtype[i] == 'LINE' or self.objtype[i] == "POLY" or self.objtype[i] == "POLYLINE":
                npoints = int(len(self.objdata[i]) / 2)
                # print(f'points num: {npoints}')
                x0, y0 = self.m(self.objdata[i][1], self.objdata[i][0])
                x0, y0 = x0 / self.coord_scale, y0 / self.coord_scale
                for j in range(1, npoints):
                    x1, y1 = self.m(self.objdata[i][j * 2 + 1], self.objdata[i][j * 2])
                    x1, y1 = x1 / self.coord_scale, y1 / self.coord_scale

                    points.append([x0, y0, 0])  # p_i
                    points.append([x1, y1, 0])  # p_i+1
                    x0, y0 = x1, y1

                    colors.append(self.objcolor[i])  # add color for each connection (p_i, p_i+1)

                if self.objtype[i] == "POLY":
                    # connect the first point and the last point of a POLY
                    points.append(points[-1])
                    points.append(points[-2 * npoints + 1])

                    colors.append(self.objcolor[i])  # add color for this first-last connection

            # Draw bounding box of objdata = [lat0,lon0,lat1,lon1]; up-l;
            elif self.objtype[i] == 'BOX':
                lat0 = min(self.objdata[i][0], self.objdata[i][2])
                lon0 = min(self.objdata[i][1], self.objdata[i][3])
                lat1 = max(self.objdata[i][0], self.objdata[i][2])
                lon1 = max(self.objdata[i][1], self.objdata[i][3])

                # (lon0, lat1) ----------- (lon1, lat1)
                #      |                         |
                #      |                         |
                #      |                         |
                # (lon0, lat0) ----------- (lon1, lat0)

                x0, y0 = self.m(lon0, lat1)
                x1, y1 = self.m(lon1, lat1)
                x2, y2 = self.m(lon1, lat0)
                x3, y3 = self.m(lon0, lat0)

                x0, y0 = x0 / self.coord_scale, y0 / self.coord_scale
                x1, y1 = x1 / self.coord_scale, y1 / self.coord_scale
                x2, y2 = x2 / self.coord_scale, y2 / self.coord_scale
                x3, y3 = x3 / self.coord_scale, y3 / self.coord_scale

                # connection1
                points.append([x0, y0, 0])
                points.append([x1, y1, 0])
                # connection2
                points.append([x1, y1, 0])
                points.append([x2, y2, 0])
                # connection3
                points.append([x2, y2, 0])
                points.append([x3, y3, 0])
                # connection4
                points.append([x3, y3, 0])
                points.append([x0, y0, 0])

                for _ in range(4):
                    colors.append(self.objcolor[i])

            # Draw circle with objdata = [latcenter,loncenter,radiusnm]
            elif self.objtype[i] == 'CIRCLE':
                pass
                # xm, ym = self.ll2xy(self.objdata[i][0], self.objdata[i][1])
                # xtop, ytop = self.ll2xy(self.objdata[i][0] + self.objdata[i][2] / 60., self.objdata[i][1])
                # radius = int(round(abs(ytop - ym)))
                # pg.draw.circle(self.radbmp, self.objcolor[i], (int(xm), int(ym)), radius, 1)

        # all types of object are put in the same list
        for i in range(0, len(points), 2):
            lines.append([i, i + 1])

        if len(points) > 0:
            self.poly_set = o3d.geometry.LineSet(
                points=o3d.utility.Vector3dVector(points),
                lines=o3d.utility.Vector2iVector(lines),
            )
            self.poly_set.colors = o3d.utility.Vector3dVector(colors)
            self._3d.scene.add_geometry("poly_set", self.poly_set, self.line_mat)

        # Reset background drawing switch
        self.redrawradbg = False

        # -------------- draw aircraft ---------------------------
        if bs.traf.ntraf > 0:  # number of aircraft>0
            aircraft_all_points = []
            aircraft_all_lines = []
            aircraft_all_colors = []

            # print(f'flight hdg: {bs.traf.hdg}')
            for i in range(bs.traf.ntraf):
                lat = bs.traf.lat[i]
                lon = bs.traf.lon[i]
                x, y = self.m(lon, lat)
                x, y = x / self.coord_scale, y / self.coord_scale

                alt = bs.traf.alt[i]
                hdg = bs.traf.hdg[i]

                self.ndcrs = 0.0  # # manual set
                isymb = int(round((bs.traf.hdg[i] - self.ndcrs) / 6.)) % 60

                if not bs.traf.cd.inconf[i]:  # not in conflict; green color
                    aircraft_i = o3d.geometry.LineSet(
                        points=o3d.utility.Vector3dVector(self.aircraft_points),
                        lines=o3d.utility.Vector2iVector(self.aircraft_lines),
                    )
                    aircraft_i.colors = o3d.utility.Vector3dVector(self.aircraft_colors_green)
                    text_color = gui.Color(0.0, 1.0, 0.0, 1.0)
                else:  # in conflict; yellow
                    aircraft_i = o3d.geometry.LineSet(
                        points=o3d.utility.Vector3dVector(self.aircraft_points),
                        lines=o3d.utility.Vector2iVector(self.aircraft_lines),
                    )
                    aircraft_i.colors = o3d.utility.Vector3dVector(self.aircraft_colors_amber)
                    text_color = gui.Color(255 / 255, 163 / 255, 71 / 255, 1.0)

                hdg_rad = np.deg2rad(-hdg)
                r = o3d.geometry.get_rotation_matrix_from_axis_angle(np.asarray([[0], [0], [hdg_rad]]))
                aircraft_i.rotate(r, aircraft_i.get_center())
                aircraft_i.translate(np.asarray([[x], [y], [0]]))
                aircraft_i.scale(self.cam_z, aircraft_i.get_center())

                # to save memory; combine all aircraft lines into one LineSet instead saving individual LineSet
                # get the tranformed points and then clear it
                trans_points = np.asarray(aircraft_i.points).tolist()
                trans_lines = (np.asarray(aircraft_i.lines) + i * 4).tolist()
                trans_colors = np.asarray(aircraft_i.colors).tolist()
                aircraft_all_points += trans_points
                aircraft_all_lines += trans_lines
                aircraft_all_colors += trans_colors

                aircraft_i.clear()
                aircraft_i = None

                # labels for each aircraft
                label_text = f'{bs.traf.id[i]} \n ' \
                             f'FL{int(round(bs.traf.alt[i] / (100. * ft)))} \n' \
                             f'{int(round(bs.traf.cas[i] / kts))}'
                label3d = self._3d.add_3d_label([x, y, 0], label_text)
                label3d.color = text_color
                self.aircraft_labels.append(label3d)

            self.all_aircraft_lineset = o3d.geometry.LineSet(
                points=o3d.utility.Vector3dVector(aircraft_all_points),
                lines=o3d.utility.Vector2iVector(aircraft_all_lines),
            )
            self.all_aircraft_lineset.colors = o3d.utility.Vector3dVector(aircraft_all_colors)
            self._3d.scene.add_geometry(f'all_aircraft', self.all_aircraft_lineset, self.line_mat)

        # -------------- draw selected routes --------
        if self.acidrte != "":
            selected_routes_points = []
            selected_routes_lines = []
            selected_routes_colors = []

            i = bs.traf.id2idx(self.acidrte)
            if i >= 0:
                for j in range(0, bs.traf.ap.route[i].nwp):
                    if j == 0:
                        x1, y1 = self.m(bs.traf.ap.route[i].wplon[j], bs.traf.ap.route[i].wplat[j])
                        x1, y1 = x1 / self.coord_scale, y1 / self.coord_scale
                    else:
                        x0, y0 = x1, y1
                        x1, y1 = self.m(bs.traf.ap.route[i].wplon[j], bs.traf.ap.route[i].wplat[j])
                        x1, y1 = x1 / self.coord_scale, y1 / self.coord_scale
                        selected_routes_points.append([x0, y0, 0])
                        selected_routes_points.append([x1, y1, 0])

                    if j >= len(self.rtewpid) or not self.rtewpid[j] == bs.traf.ap.route[i].wpname[j]:
                        # Waypoint name labels
                        # If waypoint label bitmap does not yet exist, make it

                        # Waypoint name and constraint(s), if there are any
                        txt = bs.traf.ap.route[i].wpname[j]

                        alt = bs.traf.ap.route[i].wpalt[j]
                        spd = bs.traf.ap.route[i].wpspd[j]

                        if alt >= 0. or spd >= 0.:
                            # Altitude
                            if alt < 0:
                                txt = txt + " -----/"

                            elif alt > 4500 * ft:
                                FL = int(round((alt / (100. * ft))))
                                txt = txt + " FL" + str(FL) + "/"

                            else:
                                txt = txt + " " + str(int(round(alt / ft))) + "/"

                            # Speed
                            if spd < 0:
                                txt = txt + "---"
                            else:
                                txt = txt + str(int(round(spd / kts)))

                        if j >= len(self.rtewpid):
                            self.rtewpid.append(txt)
                        else:
                            self.rtewpid[j] = txt
                    # labels for traversed waypoints
                    traverse_wp_label = self._3d.add_3d_label([x1, y1, 0], self.rtewpid[j])
                    traverse_wp_label.color = gui.Color(1.0, 1.0, 1.0, 1.0)
                    self.traverse_wp_labels.append(traverse_wp_label)

                    # Line from aircraft to active waypoint
                    if bs.traf.ap.route[i].iactwp == j:
                        x0, y0 = self.m(bs.traf.lon[i], bs.traf.lat[i])
                        x0, y0 = x0 / self.coord_scale, y0 / self.coord_scale
                        selected_routes_points.append([x0, y0, 0])
                        selected_routes_points.append([x1, y1, 0])

                for n_ in range(0, len(selected_routes_points), 2):
                    selected_routes_lines.append([n_, n_ + 1])
                for n_ in range(len(selected_routes_lines)):
                    selected_routes_colors.append(magenta)

                self.selected_routes_set = o3d.geometry.LineSet(
                    points=o3d.utility.Vector3dVector(selected_routes_points),
                    lines=o3d.utility.Vector2iVector(selected_routes_lines),
                )
                self.selected_routes_set.colors = o3d.utility.Vector3dVector(selected_routes_colors)
                self._3d.scene.add_geometry(f'selected_routes', self.selected_routes_set, self.line_mat)

        # -------
        # Draw aircraft trails which are on screen
        # if bs.traf.trails.active:
        #     lon0_list = bs.traf.trails.lon0.tolist()
        #     if len(lon0_list) > 1:  # not empty
        #         trail_points = []
        #         trail_lines = []
        #         trail_colors = []
        #         # print(len(bs.traf.trails.lon0), len(bs.traf.trails.lat0),
        #         #       len(bs.traf.trails.lon1), len(bs.traf.trails.lat1))
        #         for i in range(len(lon0_list)):
        #             x0, y0 = self.m(bs.traf.trails.lon0[i], bs.traf.trails.lat0[i])
        #             x1, y1 = self.m(bs.traf.trails.lon1[i], bs.traf.trails.lat1[i])
        #             x0, y0 = x0 / self.coord_scale, y0 / self.coord_scale
        #             x1, y1 = x1 / self.coord_scale, y1 / self.coord_scale
        #             trail_points.append([x0, y0, 0])
        #             trail_points.append([x1, y1, 0])
        #
        #             # trail_colors.append(bs.traf.trails.col[i] / 255)  # normalize to 0-1
        #
        #         for i_ in range(0, len(trail_points), 2):
        #             trail_lines.append([i_, i_ + 1])
        #         for i_ in range(len(trail_lines)):
        #             trail_colors.append(bs.traf.trails.col[i_] / 255)
        #
        #         self.trails_set = o3d.geometry.LineSet(
        #             points=o3d.utility.Vector3dVector(trail_points),
        #             lines=o3d.utility.Vector2iVector(trail_lines),
        #         )
        #         self.trails_set.colors = o3d.utility.Vector3dVector(trail_colors)
        #         self._3d.scene.add_geometry(f'draw_trials', self.trails_set, self.line_mat)

        # ------------ update information ------------
        self.utc.text = str(bs.sim.utc.replace(microsecond=0))
        self.simt.text = tim2txt(bs.sim.simt)
        self.ntraf.text = str(bs.traf.ntraf)
        self.freq.text = str(int(len(self.dts) / max(0.001, sum(self.dts))))
        self.los.text = str(len(bs.traf.cd.lospairs_unique))
        self.total_los = str(len(bs.traf.cd.lospairs_all))
        self.con = str(len(bs.traf.cd.confpairs_unique))
        self.total_con = str(len(bs.traf.cd.confpairs_all))

        self.window.set_needs_layout()  # Flags window to re-layout
        self.window.post_redraw()

    # ============================================================================
    # open3d gui

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
        self.file_done = False  # reset the selection status for next usage.
        self.file_path = ''  # reset the selection status for next usage.

        dirpath = settings.scenario_path
        os.chdir(dirpath)

        filedlg = gui.FileDialog(gui.FileDialog.OPEN, "Select file",
                                 self.window.theme)
        filedlg.add_filter(".scn", "Scenario files (.scn)")
        filedlg.add_filter("", "All files")
        filedlg.set_on_cancel(self._on_filedlg_cancel)
        filedlg.set_on_done(self._on_filedlg_done)
        self.window.show_dialog(filedlg)

    def _on_filedlg_cancel(self):
        # self.app.post_to_main_thread(self.window, self.window.close_dialog)
        self.window.close_dialog()
        self.file_done = True

    def _on_filedlg_done(self, path):
        self._fileedit.text_value = path
        self.window.close_dialog()

        self.file_done = True
        self.file_path = path

        # stack.stack(f'IC {path}')  # for manual file selection from gui
        os.chdir(self.cdir)  # remember to change the dit path back to avoid reset issue

    def on_layout(self, context=None):
        frame = self.window.content_rect
        # print(frame.x, frame.y, frame.width, frame.height)
        em = self.window.theme.font_size
        panel_width = 15 * em  # 20 * em
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

                self.cam_x -= d_x * (self.cam_z / 100)  # change speed adaptive from camera altitude
                self.cam_y -= d_y * (self.cam_z / 100)

                self._3d.look_at([self.cam_x, self.cam_y, 0], [self.cam_x, self.cam_y, self.cam_z], self.cam_up)

            if event.type == gui.MouseEvent.Type.WHEEL:
                # print(event.wheel_dx, event.wheel_dy)
                if self.cam_z > 1000:
                    zoom_speed = 100
                elif self.cam_z > 20:
                    zoom_speed = 20
                elif self.cam_z > 2:
                    zoom_speed = 1
                elif self.cam_z > 0.2:
                    zoom_speed = 0.1
                else:
                    zoom_speed = 0.01

                self.cam_z += event.wheel_dy * zoom_speed
                if self.cam_z < 0.12:
                    self.cam_z = 0.12
                if self.cam_z >= 10000:
                    self.cam_z = 10000  # adapt from the coordinate_scale

                # print(f'cam_z={self.cam_z} {event.wheel_dy} {zoom_speed}')

                self._3d.look_at([self.cam_x, self.cam_y, 0], [self.cam_x, self.cam_y, self.cam_z], self.cam_up)

            # note: use CONSUMED instead of IGNORED to avoid camera shaking and overwrite the functionality.
            return gui.Widget.EventCallbackResult.CONSUMED

        else:
            return gui.Widget.EventCallbackResult.IGNORED

    def _on_text_changed(self, new_text):
        print("edit:", new_text)

    def _console_lines(self, new_text):
        # move history context upward.
        self._lines_text = self._lines_text[1:3] + ['>> ' + new_text]
        self._line1.text = self._lines_text[0]
        self._line2.text = self._lines_text[1]
        self._line3.text = self._lines_text[2]

        if new_text.startswith(' Error'):
            self._line3.text_color = self.red
        else:
            self._line3.text_color = self.green

    def _on_value_changed(self, new_text):
        stack.stack(new_text)

        self._console_lines(new_text)
        # print("value:", new_text)
        self.tedit.text_value = ''

    def _on_main_window_closing(self):
        self.is_done = True
        return True

    # ========================================================================
    # view control from bluesky command such as: PAN ZOOM etc.
    def show_file_dialog(self):
        self._on_filedlg_button()

        # ! [important] the file dialog of open3d is working in the subprocess, so we use a bool to wait for the end
        # of the subprocess, and get the selected file path.
        while not self.file_done:
            pass

        print(f'select file path: {self.file_path}')
        return self.file_path

        # # pygame file dialog
        # path = opendialog()
        # print(f'select file path: {path}')
        # return path

    def stack(self, cmdline):
        self.echo(f'Unknown command: {cmdline}')

    def echo(self, msg='', flags=0):
        text = ''
        if msg:
            msgs = msg.split('\n')
            print(f'msgs: {msgs}')
            for m in msgs:
                text += f' {m} '
            if len(text) > 90:  # limit text length
                text = text[0:90] + '... [python console]'
            self._console_lines(text)
            # print(len(text))
        return

    def showssd(self, param):
        return False, "SSD visualization only available in QtGL GUI"

    def reset(self):
        self.objdel()  # Delete user defined objects

    def color(self, name, r, g, b):
        ''' Set custom color for aircraft or shape. '''
        if areafilter.hasArea(name):
            idx = self.objname.index(name)
            # color value needs to be scaled from 0-255 to 0-1 in open3d
            self.objcolor[idx] = (r / 255, g / 255, b / 255)
        else:
            return False, 'No object found with name ' + name

        self.redrawradbg = True  # redraw background
        return True

    def show_cmd_doc(self, cmd=''):
        # Show documentation on command
        if not cmd:
            cmd = 'Command-Reference'
        curdir = os.getcwd()
        os.chdir("data/html")
        htmlfile = cmd.lower() + ".html"
        if os.path.isfile(htmlfile):
            try:
                subprocess.Popen(htmlfile, shell=True)
            except:
                os.chdir(curdir)
                return False, "Opening " + htmlfile + " failed."
        else:
            os.chdir(curdir)
            return False, htmlfile + " is not yet available, try HELP PDF or check the wiki on Github."

        os.chdir(curdir)
        return True, "HTML window opened"

    def filteralt(self, *args):
        return False, 'Filteralt not implemented in open3d gui'

    def cmdline(self, text):
        pass

    def pan(self, *args):
        """Pan function:
               absolute: lat,lon;
               relative: ABOVE/DOWN/LEFT/RIGHT"""
        ctrlat, ctrlon = 52.073532, -0.607121
        if type(args[0]) == str:
            if args[0].upper() == "LEFT":
                self.cam_x -= 1
                self._3d.look_at([self.cam_x, self.cam_y, 0], [self.cam_x, self.cam_y, self.cam_z], self.cam_up)

            elif args[0].upper() == "RIGHT":
                self.cam_x += 1
                self._3d.look_at([self.cam_x, self.cam_y, 0], [self.cam_x, self.cam_y, self.cam_z], self.cam_up)

            elif args[0].upper() == "ABOVE" or args[0].upper() == "UP":
                self.cam_y -= 1
                self._3d.look_at([self.cam_x, self.cam_y, 0], [self.cam_x, self.cam_y, self.cam_z], self.cam_up)

            elif args[0].upper() == "DOWN":
                self.cam_y += 1
                self._3d.look_at([self.cam_x, self.cam_y, 0], [self.cam_x, self.cam_y, self.cam_z], self.cam_up)

            else:
                # to-do: implement PAN latlon/acid/airport/waypoint
                i = bs.navdb.getwpidx(args[0], ctrlat, ctrlon)
                if i < 0:
                    i = bs.navdb.getaptidx(args[0], ctrlat, ctrlon)
                    if i > 0:
                        lat = bs.navdb.aptlat[i]
                        lon = bs.navdb.aptlon[i]
                else:
                    lat = bs.navdb.wplat[i]
                    lon = bs.navdb.wplon[i]

                if i < 0:
                    return False, args[0] + "not found."

        else:
            if len(args) > 1:
                lat, lon = args[:2]
            else:
                return False

        x, y = self.m(lon, lat)
        x, y = x / self.coord_scale, y / self.coord_scale
        self.cam_x, self.cam_y = x, y
        self._3d.look_at([self.cam_x, self.cam_y, 0], [self.cam_x, self.cam_y, self.cam_z], self.cam_up)

        return True

    def zoom(self, factor, absolute=False):
        """Zoom function"""

        # Zoom factor: 2.0 means halving the display size in degrees lat/lon
        # ZOom out with e.g. 0.5
        self.cam_z = self.cam_z + (1 - factor) * 65
        self._3d.look_at([self.cam_x, self.cam_y, 0], [self.cam_x, self.cam_y, self.cam_z], self.cam_up)

        return

    def feature(self, sw, arg=""):
        # Switch/toggle/cycle radar screen features e.g. from SWRAD command
        # Coastlines

        return True  # Success

    def symbol(self):
        # self.swsep = not self.swsep
        return True

    def shownd(self, acid):
        if acid:
            self.ndacid = acid
        self.swnavdisp = not self.swnavdisp

    def getviewctr(self):
        ctrlon, ctrlat = self.m(self.cam_x, self.cam_y, inverse=True)
        return ctrlat, ctrlon

    def objappend(self, itype, name, data):
        """Add user defined objects"""
        if data is None:
            return self.objdel()

        self.objname.append(name)
        self.objtype.append(itype)
        if self.objtype[-1] == 1:
            self.objtype[-1] = "LINE"  # Convert to string

        self.objcolor.append(cyan)
        self.objdata.append(data)

        self.redrawradbg = True  # redraw background

        return

    def objdel(self):
        """Add user defined objects"""
        self.objname = []
        self.objtype = []
        self.objcolor = []
        self.objdata = []
        self.redrawradbg = True  # redraw background
        return

    def showroute(self, acid):  # Toggle show route for an aircraft id
        if self.acidrte == acid:
            self.acidrte = ""  # Click twice on same: route disappear
        else:
            self.acidrte = acid  # Show this route
        return True


if __name__ == '__main__':
    blue_sky_3d = BlueSky3dUI()
    blue_sky_3d.app.run()
