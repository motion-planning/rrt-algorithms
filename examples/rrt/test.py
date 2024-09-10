import h5py
import numpy as np

# Sample data
data = np.random.random((100, 100))
def print_h5_structure(name, obj):
    """Function to print the structure of an HDF5 file."""
    if isinstance(obj, h5py.Group):
        print(f"{name} (Group)")
    elif isinstance(obj, h5py.Dataset):
        print(f"{name} (Dataset) - Shape: {obj.shape} - Dtype: {obj.dtype}")
# Writing data to an HDF5 file
file_path = "example.h5"

with h5py.File(file_path, "w") as h5file:
    # Creating a dataset in the file
    h5file.create_dataset("dataset_name", data=data)
    # Optionally flush data to disk
    h5file.flush()

# Verify that data was written correctly
with h5py.File(file_path, "r") as h5file:
    if "dataset_name" in h5file:
        dataset = h5file["dataset_name"]
        print("Data shape:", dataset.shape)
        print("Sample data:", dataset[:5, :5])
    else:
        print("Dataset not found in the file.")

