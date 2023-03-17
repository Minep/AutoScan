import open3d as o3d

class Viewer3D(object):
    def __init__(self):
        self.main_vis = o3d.visualization.Visualizer()
        self.main_vis.create_window()
        self.mesh = None

    def tick(self):
        self.main_vis.update_renderer()
        self.main_vis.poll_events()

    def update_mesh(self, geometries):
        if self.mesh is not None:
            self.main_vis.remove_geometry(self.mesh)
        self.main_vis.add_geometry(geometries)
        self.mesh = geometries