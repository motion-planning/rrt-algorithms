import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="rrt-algorithms",
    version="0.0.1",
    author="SZanlongo",
    author_email="",
    description="Collection of rrt-based algorithms that scale to n-dimensions",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/motion-planning/rrt-algorithms",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],
    include_package_data=True,
    python_requires='>=3.10',
    install_requires=['rtree==1.2.0',
                      'numpy>=1.25',
                      'plotly',
                      'scipy>=1.11'],
)
