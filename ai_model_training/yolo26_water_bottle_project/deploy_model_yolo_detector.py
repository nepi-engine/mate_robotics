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
# Copy best.pt and create yaml file
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
    print('Starting deploy model process')
    project = yolo_utils.project_yolo_detector()
    project_dict = project.project_dict
    project_folder = project.project_folder
    data_folder = project.data_folder
    label_folder = project.label_folder
    train_folder = project.train_folder
    deploy_folder = project.deploy_folder
    model_name = project.model_name
    framework = project.framework
    base_model = project.base_model
    best_model_path = project.best_model_path
    image_size = project.image_size


    print('Updating folder pbest_model_pathermissions')
    success = ai_utils.fix_folder_permissions(train_folder,project.user,project.group)
    success = ai_utils.fix_folder_permissions(deploy_folder,project.user,project.group)
    best_model_path = None
    deploy_name = model_name + '_' + framework + '_' + str(image_size)
    copy_file_path = os.path.join(deploy_folder,deploy_name+'.pt')
    best_model_path = yolo_utils.copy_best_model(train_folder,copy_file_path)
    if best_model_path is not None:
        if os.path.exists(copy_file_path):
            output_file_path = os.path.join(deploy_folder,deploy_name +'.yaml')
            success = yolo_utils.write_model_yaml_file(project_dict,output_file_path)
            if success == True:
                print('Deploy model updated from: ' + best_model_path)
                # Clean up default yaml file
                model_info_path = os.path.join(deploy_folder,'model_info.yaml')
                if os.path.exists(model_info_path) == True:
                    try:
                        # Rename the file
                        os.remove(model_info_path)
                    except:
                        pass
            else:
                print('Failed to update model yaml file')
        else:
            print('Failed to update model from best')
    success = ai_utils.fix_folder_permissions(deploy_folder,project.user,project.group)
