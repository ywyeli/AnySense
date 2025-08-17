# Remove the unused camera and LiDAR data.
# Aug 2025.


import json
import os


def check_sensor_data(root_path):

    # Path to your JSON file
    json_file_path = root_path + 'v1.0-trainval/sample_data.json'

    # Directory containing your segmentation files
    samples_dir_path = root_path + 'samples/'
    sweeps_dir_path = root_path + 'sweeps/'

    # Load the JSON data
    with open(json_file_path, 'r') as file:
        data = json.load(file)

    # Extract the filenames of the used segmentation files from the JSON data
    used_files = set()
    for entry in data:
        used_files.add(entry['filename'])

    all_files = set()

    for item in os.listdir(samples_dir_path):
        path = os.path.join(samples_dir_path, item)
        if os.path.isdir(path):
            for f in os.listdir(path):
                all_files.add(os.path.join('samples', item, f))

    # for item in os.listdir(sweeps_dir_path):
    #     path = os.path.join(sweeps_dir_path, item)
    #     if os.path.isdir(path):
    #         for f in os.listdir(path):
    #             all_files.add(os.path.join('sweeps', item, f))

    # Identify unused files
    unused_files = all_files - used_files

    unused_dst_path = root_path + 'unused_samples'
    os.makedirs(unused_dst_path, exist_ok=True)

    for file in unused_files:
        os.makedirs(os.path.dirname(os.path.join(unused_dst_path, file)), exist_ok=True)
        os.rename(os.path.join(root_path, file), os.path.join(unused_dst_path, file))

    print(f"Completed. Removed {len(unused_files)} unused files.")


if __name__ == '__main__':

    root_path = "../carla_nus/dataset/nus/LiDAR_p0_samples/"
    check_sensor_data(root_path)

    print('Done!')



