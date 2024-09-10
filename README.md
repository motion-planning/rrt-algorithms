## Requirements
pip install scikit-fuzzy 

pip install --upgrade numpy 

pip install tensorflow keras gym 

pip install tensorflow or pip install tensorflow==2.10.0 
# rrt
Collection of rrt-based algorithms in 3-dimensions:
- rrt


Utilizes [R-trees](https://en.wikipedia.org/wiki/R-tree) to improve performance by avoiding point-wise collision-checking and distance-checking.

This would not have been possible without Steven M. LaValle's excellent [Planning Algorithms](https://lavalle.pl/planning/) book, specifically [Chapter 5, Section 5: Sampling-Based Motion Planning, Rapidly Exploring Dense Trees](https://lavalle.pl/planning/node230.html).

## Requirements
- [Python 3+](https://www.python.org/downloads/)
- [NumPy](http://www.numpy.org/)
- [Rtree](https://pypi.python.org/pypi/Rtree/)
- [Plotly](https://plot.ly/python/getting-started/) (only needed for plotting)

## Usage
Define an n-dimensional Search Space, and n-dimensional obstacles within that space. Assign start and goal locations as well as the number of iterations to expand the tree before testing for connectivity with the goal, and the max number of overall iterations.

### Search Space
Assign bounds to Search Space in form: `[(x_lower, x_upper), (y_lower, y_upper), ...]`

### Start and Goal
Points represented by tuples of form: `(x, y, ...)`

### Obstacles
Axis-aligned (hyper)rectangles represented by a tuples of form `(x_lower, y_lower, ..., x_upper, y_upper, ...)`

Non-axis aligned (hyper)rectangles or other obstacle representations should also work, provided that `collision_free` and `obstacle_free` are updated to work with the new obstacles.

### Resolution
Assign resolution of edges:
- `q`: Distance away from existing vertices to probe.
- `r`: Discretization length to use for edges when sampling along them to check for collisions. Higher numbers run faster, but may lead to undetected collisions.

### Examples
Visualization examples can be found for rrt and rrt* in both 2 and 3 dimensions.

- [3D RRT](https://plot.ly/~szanlongo/81/plot/)

## Contributing

1. Fork it!
2. Create your feature branch: `git checkout -b my-new-feature`
3. Commit your changes: `git commit -am 'Add some feature'`
4. Push to the branch: `git push origin my-new-feature`
5. Submit a pull request :D

