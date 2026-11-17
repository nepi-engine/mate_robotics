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
# Update Files and Folders, then Train
##########################################

imports = True
try:
    import os
    import sys
    import copy
    from ultralytics import YOLO
    import torch 
    import shutil

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



#YOLO.checks()

'''
cuda = None
try:
  cuda = torch.cuda.is_avialble()
  if cuda == False:
    print('No GPU found')
  else:
    print('Found GPU')
except:
  print('No GPU found')

ipex = None 
if cuda is None:   
  try:
    import intel_extension_for_pytorch as ipex
    print('Found XPU')
  except:
    print('No XPU found')    
'''
##########################################
# Variables
##########################################

##########################################
# Methods
##########################################
'''
def get_best_device():
	device = 'cpu'
	if device == 'cpu' and cuda is not None:
	    device = torch.device('cuda')
	if device == 'cpu' and ipex is not None:
	    device = torch.device('xpu')
	return device
'''


###############################################
# Main
###############################################


if __name__ == "__main__":
    sudo = ai_utils.check_for_sudo()
    print('Starting train model process')
    project = yolo_utils.project_yolo_detector()
    project_dict = project.project_dict
    project_folder = project.project_folder
    data_folder = project.data_folder
    label_folder = project.label_folder
    train_folder = project.train_folder


    model_name = project.model_name 
    classes = project.classes
    train_file = project.train_file

    best_model_path = project.best_model_path
    start_path = train_folder
    start_model = project.base_model
    
    img_size  = project.image_size
    num_epochs  = project.num_epochs
    batch_size  = project.batch_size
    num_workers = project.num_workers


    print('Updating folder permissions')
    success = ai_utils.fix_folder_permissions(label_folder,project.user,project.group)
    success = ai_utils.fix_folder_permissions(train_folder,project.user,project.group)
    print('Fixing any bad label files')
    fixed_files = ai_utils.fix_data_files(label_folder)
    print("Starting training for model name: " + model_name)
    try:
        print("Changing to training folder:", train_folder)
        os.chdir(train_folder)  
        cur_folder = os.getcwd()
    except Exception as e:
        print("Error: The specified training folder was not found: " + str(e))
 
    print("Updating training files in: " + str(train_folder))
    success = yolo_utils.update_train_files(project_dict,label_folder,train_folder)


    if success == False:
      print('Failed to udpate train file')
    else:
        # clear best files if base_model changed 
        last_dict = copy.deepcopy(project_dict)
        train_dict_file = os.path.join(train_folder,yolo_utils.TRAIN_DICT_FILE_NAME)
        if os.path.exists(train_dict_file):
            file_dict = ai_utils.read_dict_from_file(train_dict_file)
            if file_dict is not None:
                last_dict = file_dict
    
        if last_dict['BASE_MODEL'] != project_dict['BASE_MODEL']:
            print("Resetting training session for new base model: " + project_dict['BASE_MODEL'])
            for folder in ai_utils.get_folder_list(train_folder):
                try:
                    shutil.rmtree(folder)
                    print(f"Old Training Folder '{folder}' and its contents deleted successfully.")
                except OSError as e:
                    print(f"Error: {folder} : {e.strerror}")
        # last_best_file = os.path.join(train_folder,yolo_utils.BEST_FILE_NAME)
        # if os.path.exists(last_best_file):
        #     try:
        #         os.remove(last_best_file)
        #         print(f"Old Best Training file '{last_best_file}' removed.")
        #     except Exception as e:
        #         print("Failed to delete last_best_file: " + str(last_best_file))


        # Update start model

        if best_model_path is not None:
            if os.path.exists(best_model_path):
               start_path = os.path.dirname(best_model_path)
               start_model = os.path.basename(best_model_path)

        ai_utils.write_dict_to_file(project_dict,train_dict_file)
        success = ai_utils.fix_folder_permissions(train_folder,project.user,project.group)
        print("Starting training with base model: " + str(best_model_path))
        
        if os.path.exists(best_model_path):
            model = YOLO(best_model_path)
            #device = get_best_device()
            #print("Training with device: " + str(device))
            #model = model.to(device)
            print("Model Train Inputs: " + str([train_file, num_epochs, img_size, batch_size,model_name]))
            results = model.train(data=train_file, epochs=num_epochs, imgsz=img_size, batch=batch_size, workers=num_workers, name=model_name)
    success = ai_utils.fix_folder_permissions(train_folder,project.user,project.group)
