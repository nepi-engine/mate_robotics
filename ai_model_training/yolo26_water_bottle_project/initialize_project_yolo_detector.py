#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_ai_frameworks) repo
# (see https://https://github.com/nepi-engine/nepi_ai_frameworks)
#
# License: nepi applications are licensed under the "Numurus Software License", 
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment bstab.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com

##########################################
# Update Files and Folders, then Label
##########################################


imports = True
try:
    import os
    import sys
    import copy
except Exception as e:
    print("Missing required python modules " + str(e))
    print("Connect to internet and run the following in this folder")
    print("sudo pip3 install -r requirements.txt")
    print("Then try rerunning this script agian")
    imports = False

if imports == True:
  import nepi_ai_train as ai_utils
  imports = ai_utils.imports

if imports == True:
  import yolo_detector_utils as yolo_utils
  imports = yolo_utils.imports


if imports == False:
    sys.exit(1) # Terminate the script with an exit code (e.g., 1 for error)


##########################################
# Variables
##########################################

##########################################
# Methods
##########################################

###############################################
# Main
###############################################

if __name__ == '__main__':
    sudo = ai_utils.check_for_sudo()
    print('Starting create label data process')
    project = yolo_utils.project_yolo_detector()
    project_dict = project.project_dict
    project_folder = project.project_folder
    model_name = project.model_name
    data_folder = project.data_folder
    label_folder = project.label_folder
    train_folder = project.train_folder
    deploy_folder = project.deploy_folder
    use_percent_data = project.use_percent_data
    classes = project.classes
    classes_dict = project.classes_dict
    classes_file = project.classes_file
    random_file_name = project.random_file_name
    random_data_size = project.random_data_size
    print("Use Percent Data: " + str(use_percent_data))


    success = ai_utils.fix_folder_permissions(data_folder,project.user,project.group)
    fixed_files = ai_utils.fix_data_files(label_folder)
    fixed_files = ai_utils.fix_data_files(data_folder)
    # Copy/Update files from raw data folder
    imgs_list = ai_utils.update_labling_data(data_folder, label_folder, use_percent_data)

    random_folder_path = os.path.join(label_folder,random_file_name)          
    rand_imgs_list = ai_utils.create_random_data_set(imgs_list,random_folder_path,random_data_size) 
    # Check/Fix xml labels and save txt label files
 
    folders = ai_utils.get_folder_list(label_folder)
    new_classes = copy.deepcopy(classes)
    new_classes_dict = copy.deepcopy(classes_dict)
    for folder in folders:
        [new_classes,new_classes_dict] = ai_utils.convert_xml_to_txt(folder,new_classes,new_classes_dict)
    print(new_classes_dict)
    if new_classes != classes or new_classes_dict != classes_dict:
        print('Updating classes in project settings')
        project.update_classes(new_classes,new_classes_dict)
    ai_utils.write_list_to_file(classes,classes_file)
    print('Updating folder stats')
    stats_dict = ai_utils.update_stats_file(data_folder)
    stats_dict = ai_utils.update_stats_file(label_folder)

    success = ai_utils.fix_folder_permissions(project_folder,project.user,project.group)


      
