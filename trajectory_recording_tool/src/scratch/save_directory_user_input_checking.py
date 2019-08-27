#!/usr/bin/env python
# -*- coding: utf-8 -*

import os

from rospkg import RosPack, ResourceNotFound


def main():
    # Get target package path
    rospack = RosPack()
    target_package_found = False
    package_path = None
    while not target_package_found:
        target_package_name = raw_input("Please enter target package: ")
        try:
            package_path = rospack.get_path(target_package_name)
            target_package_found = True
            print("Target package path: {}".format(package_path))
        except ResourceNotFound:
            print("Package {} not found!!! Please try again.".format(target_package_name))
    # Create trajectories directory if it does not exist - https://stackoverflow.com/a/14364249
    save_dir_path = os.path.join(package_path, "trajectories")
    print("Save directory path: {}".format(save_dir_path))
    try:  # prevents a common race condition - duplicated attempt to create directory
        os.makedirs(save_dir_path)
    except OSError:
        if not os.path.isdir(save_dir_path):
            raise
    # Get save file name
    name_set = False
    target_save_name = None
    while not name_set:
        target_save_name = raw_input("Please enter the save file name: ")
        target_name_path_1 = os.path.join(save_dir_path, target_save_name, ".json")
        target_name_path_2 = os.path.join(save_dir_path, target_save_name, "_1.json")
        if os.path.isfile(target_name_path_1) or os.path.isfile(target_name_path_2):
            print("A file with name {} already exists!!! Please try again.".format(target_save_name))
        else:
            name_set = True


if __name__ == '__main__':
    main()
