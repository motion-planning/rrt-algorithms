# rrt

Collection of rrt-based algorithms
- rrt
- rrt* (rrt-star)
Both implementations scale to n-dimensions.

## Requirements

Python 3+
Plotly (only needed for plotting)

## Usage

Examples can be found for rrt and rrt* in both 2 and 3 dimensions.

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

## Todo

- Use r-trees to reduce time needed for finding vertices and obstacles.

## License

[MIT License](https://github.com/motion-planning/rrt-algorithms/blob/master/LICENSE)