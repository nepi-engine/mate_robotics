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
    import subprocess
except Exception as e:
    print("Missing required python modules " + str(e))
    print("Connect to internet and run the following in this folder")
    print("sudo pip install -r requirements.txt")
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


home_directory = os.path.expanduser("~")
LABEL_IMAGE_CONFIG_FILE = os.path.join(home_directory,'.labelImgSettings.pkl')
##########################################
# Methods
##########################################


###############################################
# Main
###############################################

if __name__ == '__main__':
    sudo = ai_utils.check_for_sudo()
    print('Starting data labeling process')
    project = yolo_utils.project_yolo_detector()
    label_folder = project.label_folder
    classes = project.classes
    classes_dict = project.classes_dict
    classes_file = project.classes_file
    if os.path.exists(label_folder) == False:
        print('Failed to find required project folder: ' + label_folder + " " + str())
    else:

        print('Updating folder permissions')
        success = ai_utils.fix_folder_permissions(label_folder,project.user,project.group)

        if os.path.exists(LABEL_IMAGE_CONFIG_FILE):
            print("Reseting labelImg config for new session " + str(LABEL_IMAGE_CONFIG_FILE)) 
            os.remove(LABEL_IMAGE_CONFIG_FILE)
        else:
            print("Failed to reset labelImg config for new session " + str(LABEL_IMAGE_CONFIG_FILE))

        path, folders, files = next(os.walk(label_folder))
        print('')
        print('***********************')
        print('Select Folder to Label')
        print('')
        menu_options = folders
        ai_utils.display_menu(menu_options)
        sel_ind,sel_option = ai_utils.get_user_choice(menu_options)
        sel_path = os.path.join(label_folder,sel_option)
        print('')
        print("You selected: " + sel_option) 
        print('')
        print('***********************')
        if os.path.exists(sel_path) == False:
            print('Failed to find labeling folder: ' + sel_path + " " + str())
        else:
            success = ai_utils.fix_folder_permissions(sel_path,project.user,project.group)
            script = 'labelImg'
            classes_file = project.classes_file
            args = [sel_path,classes_file]
            command = [script] + args
            print("Launching script with command " + str(command)) 
            try:
                result = subprocess.run(command, capture_output=True, text=True, check=True)
                print("Script output:")
                print(result.stdout)
                if result.stderr:
                    print("Script errors:")
                    print(result.stderr)
            except subprocess.CalledProcessError as e:
                print("Error starting script: " + script + " " + str(e))

            print('Updating converting xml files to txt files')
            new_classes = copy.deepcopy(classes)
            new_classes_dict = copy.deepcopy(classes_dict)
            [new_classes,new_classes_dict] = ai_utils.convert_xml_to_txt(sel_path,classes,classes_dict)
            if new_classes != classes or new_classes_dict != classes_dict:
                project.update_classes(new_classes,new_classes_dict)
            print('Updating classes files')
            folder_classes_file = os.path.join(sel_path,yolo_utils.CLASSES_FILE_NAME)
            ai_utils.write_list_to_file(classes,folder_classes_file)
            ai_utils.write_list_to_file(classes,classes_file)
            print('Updating folder stats')
            stats_dict = ai_utils.update_stats_file(label_folder)
            #print('Ended Label Data session with label folder stats: ' + str(stats_dict))
            print('Updating folder permissions')
            success = ai_utils.fix_folder_permissions(sel_path,project.user,project.group)

    
      
