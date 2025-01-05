import sys
import vtk
from PyQt5 import QtWidgets
from PyQt5.QtCore import QTimer
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import math
from ik_class import InvKin
import pandas as pd
import numpy as np
import math as m

# Your existing parameter values
base_length = 0.142
top_length = 0.067
d_b = 0.23
d_t = 0.25
l1 = 0.075
l2 = 0.23

# Initialize your InvKin instance
zamanov = InvKin(6.7, 23, 14.2, 25, 7.5, 18)
inputs = [0, 2, 0, 30, 0, 0]

# Perform the inverse kinematics calculations
zamanov.inv_k(inputs)
zamanov.treg_motion_time = 20
zamanov.tregectory_ploting()
zamanov.tregectory_ploting_task_3()
bottom_vertices = np.delete(np.array(zamanov.bottom_vertices()), 3, 1)
top_vertices_initial = np.delete(np.array(zamanov.top_vertices_global()), 3, 1)
top_vertices_dest = np.delete(np.array(zamanov.top_vertices_global(*inputs)), 3, 1)

class VTKRenderWindowInteractor(QtWidgets.QFrame):
    def __init__(self, parent=None):
        super(VTKRenderWindowInteractor, self).__init__(parent)

        self.vl = QtWidgets.QVBoxLayout()
        self.vtkWidget = QVTKRenderWindowInteractor(self)  # Use QVTKRenderWindowInteractor here
        self.vl.addWidget(self.vtkWidget)

        self.setLayout(self.vl)
        self.ren = vtk.vtkRenderer()
        self.vtkWidget.GetRenderWindow().AddRenderer(self.ren)
        self.iren = self.vtkWidget.GetRenderWindow().GetInteractor()
        self.set_trackball_interactor()

    def set_trackball_interactor(self):
        style = vtk.vtkInteractorStyleTrackballCamera()
        self.iren.SetInteractorStyle(style)

    def GetRenderWindow(self):
        return self.vtkWidget.GetRenderWindow()

    def GetInteractor(self):
        return self.iren

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)

        self.frame = VTKRenderWindowInteractor(self)
        self.setCentralWidget(self.frame)
        self.ren = self.frame.ren

        self.init_hexagons()
        self.init_highlight_points()
        self.setup_grid()  # Add grid visualization
        self.setup_animation()

        self.frame.iren.Initialize()
        self.frame.iren.Start()

    def setup_grid(self):
        # Define grid parameters
        grid_size = 10  # Number of grid lines in each direction
        grid_spacing = 1.0  # Spacing between grid lines
        grid_color = [0.7, 0.7, 0.7]  # Grid color

        # Create grid lines parallel to X-axis
        for i in range(-grid_size, grid_size + 1):
            line_source = vtk.vtkLineSource()
            line_source.SetPoint1(i * grid_spacing, -grid_size * grid_spacing, 0)
            line_source.SetPoint2(i * grid_spacing, grid_size * grid_spacing, 0)

            line_mapper = vtk.vtkPolyDataMapper()
            line_mapper.SetInputConnection(line_source.GetOutputPort())

            line_actor = vtk.vtkActor()
            line_actor.SetMapper(line_mapper)
            line_actor.GetProperty().SetColor(grid_color)
            self.ren.AddActor(line_actor)

        # Create grid lines parallel to Y-axis
        for i in range(-grid_size, grid_size + 1):
            line_source = vtk.vtkLineSource()
            line_source.SetPoint1(-grid_size * grid_spacing, i * grid_spacing, 0)
            line_source.SetPoint2(grid_size * grid_spacing, i * grid_spacing, 0)

            line_mapper = vtk.vtkPolyDataMapper()
            line_mapper.SetInputConnection(line_source.GetOutputPort())

            line_actor = vtk.vtkActor()
            line_actor.SetMapper(line_mapper)
            line_actor.GetProperty().SetColor(grid_color)
            self.ren.AddActor(line_actor)
            
    def init_hexagons(self):
        # Define initial and destination points for the moving hexagon
        self.initial_points = top_vertices_initial

        # print(top_vertices_dest)
        
        self.destination_points = top_vertices_dest

        self.moving_points = vtk.vtkPoints()
        for p in self.initial_points:
            self.moving_points.InsertNextPoint(p)

        self.moving_hexagon = vtk.vtkPolygon()
        self.moving_hexagon.GetPointIds().SetNumberOfIds(6)
        for i in range(6):
            self.moving_hexagon.GetPointIds().SetId(i, i)

        self.moving_polys = vtk.vtkCellArray()
        self.moving_polys.InsertNextCell(self.moving_hexagon)

        self.moving_polyData = vtk.vtkPolyData()
        self.moving_polyData.SetPoints(self.moving_points)
        self.moving_polyData.SetPolys(self.moving_polys)

        self.moving_mapper = vtk.vtkPolyDataMapper()
        self.moving_mapper.SetInputData(self.moving_polyData)

        self.moving_actor = vtk.vtkActor()
        self.moving_actor.SetMapper(self.moving_mapper)
        self.moving_actor.GetProperty().SetColor(0.0, 0.0, 1.0)  # Blue color for the moving hexagon
        self.moving_actor.GetProperty().SetOpacity(0.5)  # Set opacity to 50%

        self.ren.AddActor(self.moving_actor)

        # Define the points for the stationary hexagon
        self.stationary_points_list = bottom_vertices

        self.stationary_points = vtk.vtkPoints()
        for p in self.stationary_points_list:
            self.stationary_points.InsertNextPoint(p)

        self.stationary_hexagon = vtk.vtkPolygon()
        self.stationary_hexagon.GetPointIds().SetNumberOfIds(6)
        for i in range(6):
            self.stationary_hexagon.GetPointIds().SetId(i, i)

        self.stationary_polys = vtk.vtkCellArray()
        self.stationary_polys.InsertNextCell(self.stationary_hexagon)

        self.stationary_polyData = vtk.vtkPolyData()
        self.stationary_polyData.SetPoints(self.stationary_points)
        self.stationary_polyData.SetPolys(self.stationary_polys)

        self.stationary_mapper = vtk.vtkPolyDataMapper()
        self.stationary_mapper.SetInputData(self.stationary_polyData)

        self.stationary_actor = vtk.vtkActor()
        self.stationary_actor.SetMapper(self.stationary_mapper)
        self.stationary_actor.GetProperty().SetColor(0.0, 1.0, 0.0)  # Green color for the stationary hexagon
        self.stationary_actor.GetProperty().SetOpacity(0.5)  # Set opacity to 50%

        self.ren.AddActor(self.stationary_actor)

        # Draw lines connecting vertices of stationary and moving hexagons
        self.line_actors = []
        for i in range(6):
            line_source = vtk.vtkLineSource()
            line_source.SetPoint1(self.stationary_points_list[i])
            line_source.SetPoint2(self.initial_points[i])

            line_mapper = vtk.vtkPolyDataMapper()
            line_mapper.SetInputConnection(line_source.GetOutputPort())

            line_actor = vtk.vtkActor()
            line_actor.SetMapper(line_mapper)
            line_actor.GetProperty().SetColor(1.0, 0.0, 0.0)  # Red color for lines
            line_actor.GetProperty().SetOpacity(0.5)  # Set opacity to 50%

            self.ren.AddActor(line_actor)
            self.line_actors.append(line_actor)

        self.ren.SetBackground(1.0, 1.0, 1.0)  # Set background to white

        # Set the camera position between top and side view
        camera = self.ren.GetActiveCamera()
        camera.SetPosition(30, 30, 30)  # Position between top and side view
        camera.SetFocalPoint(0, 0, 0)  # Focus on the origin
        camera.SetViewUp(0, 0, 1)  # Set the view up direction
        self.ren.ResetCamera()

    def init_highlight_points(self):
        # Highlight the origin
        origin = [0.0, 0.0, 0.0]
        self.origin_sphere = vtk.vtkSphereSource()
        self.origin_sphere.SetCenter(origin)
        self.origin_sphere.SetRadius(0.5)

        origin_mapper = vtk.vtkPolyDataMapper()
        origin_mapper.SetInputConnection(self.origin_sphere.GetOutputPort())

        self.origin_actor = vtk.vtkActor()
        self.origin_actor.SetMapper(origin_mapper)
        self.origin_actor.GetProperty().SetColor(1.0, 0.0, 0.0)  # Red color
        self.origin_actor.GetProperty().SetOpacity(0.5)  # Set opacity to 50%

        self.ren.AddActor(self.origin_actor)

        # Highlight the vertices of the moving hexagon
        self.vertex_spheres = []
        self.vertex_actors = []
        for point in self.initial_points:
            sphere = vtk.vtkSphereSource()
            sphere.SetCenter(point)
            sphere.SetRadius(0.5)

            sphere_mapper = vtk.vtkPolyDataMapper()
            sphere_mapper.SetInputConnection(sphere.GetOutputPort())

            sphere_actor = vtk.vtkActor()
            sphere_actor.SetMapper(sphere_mapper)
            sphere_actor.GetProperty().SetColor(0.0, 1.0, 0.0)  # Green color
            sphere_actor.GetProperty().SetOpacity(0.5)  # Set opacity to 50%

            self.ren.AddActor(sphere_actor)

            self.vertex_spheres.append(sphere)
            self.vertex_actors.append(sphere_actor)

    def update_highlight_points(self):
        for i in range(6):
            self.vertex_spheres[i].SetCenter(self.moving_points.GetPoint(i))
            self.vertex_spheres[i].Update()

    def update_lines(self):
        for i in range(6):
            line_source = self.line_actors[i].GetMapper().GetInputConnection(0, 0).GetProducer()
            line_source.SetPoint1(self.stationary_points_list[i])
            line_source.SetPoint2(self.moving_points.GetPoint(i))
            line_source.Update()

    def setup_animation(self):
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.animate)
        self.timer.start(50)

        self.num_steps = 100
        self.current_step = 0

    def animate(self):
        if self.current_step < self.num_steps:
            t = self.current_step / self.num_steps
            for i in range(6):
                x = (1-t) * self.initial_points[i][0] + t * self.destination_points[i][0]
                y = (1-t) * self.initial_points[i][1] + t * self.destination_points[i][1]
                z = (1-t) * self.initial_points[i][2] + t * self.destination_points[i][2]
                self.moving_points.SetPoint(i, x, y, z)
            self.moving_points.Modified()
            self.update_highlight_points()
            self.update_lines()
            self.frame.GetRenderWindow().Render()
            self.current_step += 1
        else:
            self.timer.stop()

class QVTKRenderWindowInteractor(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(QVTKRenderWindowInteractor, self).__init__(parent)
        self._RenderWindow = vtk.vtkRenderWindow()
        self._RenderWindowInteractor = vtk.vtkRenderWindowInteractor()
        self._RenderWindowInteractor.SetRenderWindow(self._RenderWindow)

        layout = QtWidgets.QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._RenderWindowInteractor.GetRenderWindow().GetGenericWindowId())
        self.setLayout(layout)

    def GetRenderWindow(self):
        return self._RenderWindow

    def GetInteractor(self):
        return self._RenderWindowInteractor

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
