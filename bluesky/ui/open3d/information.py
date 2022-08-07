import open3d as o3d
import open3d.visualization.gui as gui


class Info:
    def __init__(self):
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
