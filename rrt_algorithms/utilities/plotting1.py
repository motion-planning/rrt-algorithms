from pathlib import Path
import plotly as py
from plotly import graph_objs as go
import numpy as np

colors = ['darkblue', 'teal']

class Plot(object):
    def __init__(self, filename):
        """
        Create a plot
        :param filename: filename
        """
        self.filename = Path(__file__).parent / "../../output/visualizations/" / f"{filename}.html"
        if not self.filename.parent.exists():
            self.filename.parent.mkdir(parents=True, exist_ok=True)
        self.filename = str(self.filename)
        self.data = []
        self.layout = {'title': 'Plot',
                       'showlegend': False}

        self.fig = {'data': self.data,
                    'layout': self.layout}

    def plot_tree(self, X, trees):
        """
        Plot tree
        :param X: Search Space
        :param trees: list of trees
        """
        if X.dimensions == 2:  # plot in 2D
            self.plot_tree_2d(trees)
        elif X.dimensions == 3:  # plot in 3D
            self.plot_tree_3d(trees)
        else:  # can't plot in higher dimensions
            print("Cannot plot in > 3 dimensions")

    def plot_tree_2d(self, trees):
        """
        Plot 2D trees
        :param trees: trees to plot
        """
        for i, tree in enumerate(trees):
            for start, end in tree.E.items():
                if end is not None:
                    trace = go.Scatter(
                        x=[start[0], end[0]],
                        y=[start[1], end[1]],
                        line=dict(
                            color=colors[i]
                        ),
                        mode="lines"
                    )
                    self.data.append(trace)

    def plot_tree_3d(self, trees):
        """
        Plot 3D trees
        :param trees: trees to plot
        """
        for i, tree in enumerate(trees):
            for start, end in tree.E.items():
                if end is not None:
                    trace = go.Scatter3d(
                        x=[start[0], end[0]],
                        y=[start[1], end[1]],
                        z=[start[2], end[2]],
                        line=dict(
                            color=colors[i]
                        ),
                        mode="lines"
                    )
                    self.data.append(trace)

    def plot_obstacles(self, X, obstacles):
        if X.dimensions == 3:  # Plot in 3D
            for obstacle in obstacles:
                x_center, y_center, z_min, z_max, radius = obstacle

                z = np.linspace(z_min, z_max, 50)
                theta = np.linspace(0, 2 * np.pi, 50)
                theta_grid, z_grid = np.meshgrid(theta, z)
                x_grid = x_center + radius * np.cos(theta_grid)
                y_grid = y_center + radius * np.sin(theta_grid)

                obs = go.Surface(
                    x=x_grid,
                    y=y_grid,
                    z=z_grid,
                    colorscale='Viridis',
                    opacity=0.7
                )
                self.data.append(obs)
    def draw(self, auto_open=True):
        """
        Render the plot to a file
        """
        py.offline.plot(self.fig, filename=self.filename, auto_open=auto_open)
    def plot_start(self, X, x_init):
        """
        Plot starting point
        :param X: Search Space
        :param x_init: starting location
        """
        if X.dimensions == 2:  # plot in 2D
            trace = go.Scatter(
                x=[x_init[0]],
                y=[x_init[1]],
                marker=dict(
                    color="orange",
                    size=10
                ),
                mode="markers"
            )
            self.data.append(trace)
        elif X.dimensions == 3:  # plot in 3D
            trace = go.Scatter3d(
                x=[x_init[0]],
                y=[x_init[1]],
                z=[x_init[2]],
                marker=dict(
                    color="orange",
                    size=10
                ),
                mode="markers"
            )
            self.data.append(trace)
        else:  # can't plot in higher dimensions
            print("Cannot plot in > 3 dimensions")
            
    def plot_goal(self, X, x_goal, color="green"):
        """
        Plot goal point
        :param X: Search Space
        :param x_goal: goal location
        :param color: color of the goal point
        """
        if X.dimensions == 2:  # plot in 2D
            trace = go.Scatter(
                x=[x_goal[0]],
                y=[x_goal[1]],
                marker=dict(
                    color=color,
                    size=10
                ),
                mode="markers"
            )
            self.data.append(trace)
        elif X.dimensions == 3:  # plot in 3D
            trace = go.Scatter3d(
                x=[x_goal[0]],
                y=[x_goal[1]],
                z=[x_goal[2]],
                marker=dict(
                    color=color,
                    size=10
                ),
                mode="markers"
            )
            self.data.append(trace)
        else:  # can't plot in higher dimensions
            print("Cannot plot in > 3 dimensions")

    def plot_path(self, X, path, color='red'):
        """
        Plot path through Search Space
        :param X: Search Space
        :param path: path through space given as a sequence of points
        :param color: color of the path to be plotted
        """
        if X.dimensions == 2:  # plot in 2D
            x, y = [], []
            for i in path:
                x.append(i[0])
                y.append(i[1])
            trace = go.Scatter(
                x=x,
                y=y,
                line=dict(
                    color=color,
                    width=4
                ),
                mode="lines"
            )

            self.data.append(trace)
        elif X.dimensions == 3:  # plot in 3D
            x, y, z = [], [], []
            for i in path:
                x.append(i[0])
                y.append(i[1])
                z.append(i[2])
            trace = go.Scatter3d(
                x=x,
                y=y,
                z=z,
                line=dict(
                    color=color,
                    width=4
                ),
                mode="lines"
            )

            self.data.append(trace)
        else:
            print("Cannot plot in > 3 dimensions")


    def plot_intermediate_goal(self, X, x_intermediate):
        """
        Plot intermediate goal point
        :param X: Search Space
        :param x_intermediate: intermediate goal location
        """
        if X.dimensions == 2:  # plot in 2D
            trace = go.Scatter(
                x=[x_intermediate[0]],
                y=[x_intermediate[1]],
                marker=dict(
                    color="pink",
                    size=10
                ),
                mode="markers"
            )
            self.data.append(trace)
        elif X.dimensions == 3:  # plot in 3D
            trace = go.Scatter3d(
                x=[x_intermediate[0]],
                y=[x_intermediate[1]],
                z=[x_intermediate[2]],
                marker=dict(
                    color="pink",
                    size=10
                ),
                mode="markers"
            )
            self.data.append(trace)
        else:  # can't plot in higher dimensions
            print("Cannot plot in > 3 dimensions")                    
