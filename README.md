# rrt

Collection of rrt-based algorithms
- rrt
- rrt* (rrt-star)

## Requirements

Python 3+
Plotly (only needed for plotting)

## Usage

### Configuration Space
Assign bounds to configuration space in form: `[(x_lower, x_upper), (y_lower, y_upper), ...]`

### Start and Goal
Tuple of form: `(x, y, ...)`

### Obstacles
Tuples of form (x_lower, y_lower, ..., x_upper, y_upper, ...)

## Contributing

1. Fork it!
2. Create your feature branch: `git checkout -b my-new-feature`
3. Commit your changes: `git commit -am 'Add some feature'`
4. Push to the branch: `git push origin my-new-feature`
5. Submit a pull request :D

## TODO

- Use r-trees to reduce time needed for finding vertices and obstacles
