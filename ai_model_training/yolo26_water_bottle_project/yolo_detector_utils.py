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



############################
# Utility functions for yolo detector training
############################

imports = True
try:
    import os
    import sys
    import grp
    import pwd    
    import random
    from pathlib import Path

except Exception as e:
    print("Missing required python modules " + str(e))
    print("Connect to internet and run the following in this folder")
    print("sudo pip3 install -r requirements.txt")
    print("Then try rerunning this script agian")
    imports = False

if imports == True:
  import nepi_ai_train as ai_utils
  imports = ai_utils.imports


if imports == False:
    sys.exit(1) # Terminate the script with an exit code (e.g., 1 for error)



##########################################
# PORJECT SETTINGS - Edit as Necessary
##########################################

FRAMEWORK_NAME = 'yolo'
MODEL_TYPE = 'detection'

PROJECT_FILE = 'project_settings.yaml'

CURRENT_FOLDER = os.path.realpath(__file__)
DATA_RAW_FOLDER = 'data_raw'
DATA_LABEL_FOLDER = 'data_labeling'
MODEL_TRAIN_FOLDER = 'model_training'
MODEL_DEPLOY_FOLDER = 'model_deploy'

IMAGE_FILE_TYPES = ['jpg','JPG','jpeg','png','PNG']

RANDOM_FILE_NAME = 'random_set'
CLASSES_FILE_NAME = 'classes.txt'
STATS_FILE_NAME = 'stats.yaml'
TRAIN_DICT_FILE_NAME = 'train_info_dict.yaml'


CUSTOM_FILE_NAME = 'data_custom.yaml'
BEST_FILE_NAME = 'best.pt'

VAL_DATA_PERCENTAGE = 10
TEST_DATA_PERCENTAGE = 10
MAKE_TRAIN_TEST_UNIQUE = True
USE_BEST_MODEL_FOR_RETRAIN = True




##########################################
# Yolo Project Class
##########################################


class project_yolo_detector:
    
    model_name = ''
    framework = ''

    project_folder = ''
    projects_folder = ''
    mount_folder = ''

    data_folder = ''
    label_folder = ''
    train_folder = ''
    deploy_folder = ''

    use_percent_data = 100

    classes_file = ''
    train_file = ''
    img_types = IMAGE_FILE_TYPES

    base_model = ''
    best_model_path = ''
    image_size = 0
    num_epochs = 0
    batch_size = 0
    num_workers = 0

    # Read in project settings
    project_file = ''
    project_dict = dict()

    # Gather owner and group details for project mountpoint
    user = None
    group = None

    def __init__(self, project_folder = None):
        self.update_project(project_folder)
            
    def update_project(self,project_folder = None):
        file_path = os.path.realpath(__file__)
        self.project_folder = os.path.dirname(file_path)
        if project_folder is not None:
            if os.path.exists(project_folder) == True:
                    self.project_folder = project_folder

        self.projects_folder = os.path.dirname(self.project_folder)
        self.mount_folder = os.path.dirname(self.projects_folder)

        self.data_folder = os.path.join(self.project_folder,DATA_RAW_FOLDER)
        self.label_folder = os.path.join(self.project_folder,DATA_LABEL_FOLDER)
        self.train_folder = os.path.join(self.project_folder,MODEL_TRAIN_FOLDER)
        self.deploy_folder = os.path.join(self.project_folder,MODEL_DEPLOY_FOLDER)

        self.classes_file = os.path.join(self.label_folder,CLASSES_FILE_NAME)
        self.train_file = os.path.join(self.train_folder,CUSTOM_FILE_NAME)
        self.random_file_name = RANDOM_FILE_NAME

        self.img_types = IMAGE_FILE_TYPES

        # Read in project settings
        self.project_file = os.path.join(self.project_folder,PROJECT_FILE)
        if os.path.exists(self.project_file) == False:
            print("Failed to find project settings file: " + str(self.project_file))
            self.project_dict = dict()
        else:
            print("Importing project settings from file: " + str(self.project_file))
            self.project_dict = ai_utils.read_dict_from_file(self.project_file)
            #print("Imported project settings: " + str(self.project_dict))
            self.framework = self.project_dict['FRAMEWORK']
            self.model_name = ai_utils.get_clean_name(self.project_dict['NAME'])
            self.project_dict['NAME'] = self.model_name
            self.description = self.project_dict['DESCRIPTION']
            self.classes = self.project_dict['CLASSES']
            self.use_percent_data = self.project_dict['USE_PERCENT_DATA']
            self.random_data_size =  self.project_dict['RANDOM_DATA_SIZE']

            self.base_model = self.project_dict['BASE_MODEL']
            best_model_path = os.path.join(self.train_folder,self.base_model)
            if USE_BEST_MODEL_FOR_RETRAIN == True:
                found_best = self.get_best_model(self.train_folder)
                if found_best is not None:
                    best_model_path = found_best
            self.best_model_path = best_model_path  

            
            self.image_size = self.project_dict['IMAGE_SIZE']
            self.num_epochs = self.project_dict['NUM_EPOCHS']
            self.batch_size = self.project_dict['BATCH_SIZE']
            self.num_workers = self.project_dict['NUM_WORKERS']

            # Gather owner and group details for project mountpoint

            stat_info = os.stat(self.projects_folder)
            self.uid = stat_info.st_uid
            self.gid = stat_info.st_gid

            self.user = pwd.getpwuid(self.uid)[0]
            self.group = grp.getgrgid(self.gid)[0]
            #print([self.user, self.group])

            if 'CLASSES_DICT' not in self.project_dict.keys():
                self.project_dict['CLASSES_DICT'] = ai_utils.create_classes_dict(self.classes)
            self.classes_dict = self.project_dict['CLASSES_DICT']


    def get_best_model(self,directory='.'):
        best_model_path = None
        root_path = Path(directory)
        # Use rglob (recursive glob) with "**" pattern for the filename
        # The pattern should match any path leading to the exact filename
        # We join "**" and the target filename to create the full pattern
        pattern = f"**/{BEST_FILE_NAME}"
        file_list = [path.resolve() for path in root_path.rglob(pattern) if path.is_file()]
        if len(file_list) > 0:
           best_model_path = file_list[-1]
        return best_model_path




    def update_classes(self,classes, classes_dict):
        self.project_dict['CLASSES'] = classes
        self.classes = classes
        self.project_dict['CLASSES_DICT'] = classes_dict
        self.classes_dict = classes_dict
        print('Updating project settings file: ' + self.project_file + ' with dict: ' + str(self.project_dict) )
        ai_utils.write_dict_to_file(self.project_dict,self.project_file)



##########################################
# Methods
##########################################






def copy_best_model(source_folder,output_file_path):
    best_model_path = None
    found_model_path = None
    found_results_path = ''
    #print(source_folder)
    if os.path.exists(source_folder) == True:
        for path, dirs, files in os.walk(source_folder):
            #print(files)
            for file in files:
                if file == BEST_FILE_NAME:
                    found_model_path = os.path.join(path, file)
    if found_model_path is not None:
        print('Found best model file: ' + found_model_path)
        output_path = os.path.dirname(output_file_path)
        if os.path.exists(output_path):
            if os.path.exists(output_file_path):
                os.remove(output_file_path)
            success = ai_utils.copy_file(found_model_path,output_file_path)
            try:
                found_results_path = os.path.join(found_model_path.split('weights')[0],'results.csv')
                output_results_path = output_file_path.replace('.pt','.csv')
                print('Copying results file: ' + found_results_path + ' ' + output_results_path)
                if os.path.exists(output_results_path):
                    try:
                        os.remove(output_results_path)
                    except:
                        pass
                
                success = ai_utils.copy_file(found_results_path,output_results_path)
            except:
                pass
            if os.path.exists(output_file_path):
                best_model_path = found_model_path
            
    return best_model_path


def update_train_files(project_dict,label_folder,train_folder):
  print("Updating training files in: " + train_folder)
  exist_files = []
  train_files = []
  val_files = []
  test_files = []
  ulab_files = []

  ### Load existing files
  train_file_path = os.path.join(train_folder,'train_data.txt')
  if os.path.exists(train_file_path) == True:
    train_files = ai_utils.read_list_from_file(train_file_path)
    exist_files = train_files
  val_file_path = os.path.join(train_folder,'val_data.txt')
  if os.path.exists(val_file_path) == True:
    val_files = ai_utils.read_list_from_file(val_file_path)
    exist_files = exist_files + val_files
  test_file_path = os.path.join(train_folder,'test_data.txt')
  if os.path.exists(test_file_path) == True:
    test_files = ai_utils.read_list_from_file(test_file_path)
    exist_files = exist_files + test_files
  ### Walk through folder folders
  print("Processing folders in: " + label_folder)
  folders_to_process=ai_utils.get_folder_list(label_folder)
  print('')
  print('Found folders: ' + str(folders_to_process))
  for folder in folders_to_process:
    print('Processing folder: ' + folder)
    path, dirs, files = next(os.walk(folder))
    data_size = len(files)
    ind = 0
    data_val_size = int(float(1)/float(VAL_DATA_PERCENTAGE) * data_size)
    val_indexes = random.sample(range(data_size), k=data_val_size)
    #print("created val_data indexes: " + str(val_indexes))
    data_test_size = int(float(1)/float(TEST_DATA_PERCENTAGE) * data_size)
    test_indexes = random.sample(range(data_size), k=data_test_size)
    #print("created test_data indexes: " + str(test_indexes))
    files = os.listdir(folder)
    #print("Found " + str(len(files)) + " files in folder")
    #print('Found image files: ' + str(files))
    for f in files:
      f_ext = os.path.splitext(f)[1]
      f_ext = f_ext.replace(".","")
      try:
        if f_ext in ai_utils.IMAGE_FILE_TYPES:
          image_file = (folder + '/' + f)
          #print('Found image file: ' + image_file)
          #print(image_file)
          label_file = (folder + '/' + f.split(f_ext)[0]+'txt')
          #print('Looking for label file: ' + label_file)
          if os.path.exists(label_file):
            #print('Found label file' + label_file)
            ind += 1
            if ind in val_indexes and image_file not in exist_files: ##### FINISH THIS
              #print('Adding image to val file list')
              val_files.append(image_file)
            elif ind in test_indexes and image_file not in exist_files: 
              #print('Adding image to test file list')
              test_files.append(image_file)
            elif image_file not in exist_files: 
              #print('Adding image to train file list')
              train_files.append(image_file)
          else:
            # print("Warning: No label file for image: " + image_file)
            ulab_files.append(image_file)
      except Exception as e:
        print("Excepton on file write: " + str(e))


  #print(test_files)
  #print(test_label_files)
  #print(train_files)
  #print(val_files)

  print("Found " + str(len(ulab_files)) + " unlabeled files")

  ### Create train/test data set file
  ai_utils.write_list_to_file(train_files, train_file_path)
  ai_utils.write_list_to_file(val_files, val_file_path)
  ai_utils.write_list_to_file(test_files, test_file_path)

  ### Create dictionary

  classes_list = project_dict['CLASSES']
  number_of_classes = len(classes_list)


  #data : dict[str, any] = {
  data = {
    'path' : train_folder,
    'train' : os.path.basename(train_file_path),
    'val' : os.path.basename(val_file_path),
    'test' : os.path.basename(test_file_path),
    'nc' : number_of_classes,
    'names' : classes_list
  }
  print("Updating data_custom yaml values with: " + str(data))
  custom_file_path = os.path.join(train_folder,CUSTOM_FILE_NAME)
  print("Updating data_custom yaml file: " + str(custom_file_path))
  success = ai_utils.write_dict_to_file(data,custom_file_path)
  return success


def write_model_yaml_file(project_dict,output_file_path):
    success = False
    weight_file = os.path.basename(output_file_path).replace('.yaml','.pt')
    ### Create dictionary
    data = {
        'ai_model' : {
            'framework' : {
                'name' : project_dict['FRAMEWORK']
            },
            'type' : {
                'name' : MODEL_TYPE
            },
            'display_name' : {
               'name' : project_dict['DISPLAY_NAME']
            },
            'description' : {
                'name' : project_dict['DESCRIPTION']
            },
            'weight_file' : {
                'name' : weight_file
            },
            'image_size' : {
                'image_width' : {
                    'value' : project_dict['IMAGE_SIZE']
                },
                'image_height' : {
                    'value' : project_dict['IMAGE_SIZE']
                }
            },
            'classes' : {
                'names' : project_dict['CLASSES']
            }
        }
    }
    if os.path.exists(output_file_path) == True:
        try:
            os.remove(output_file_path)
        except Exception as e:
            print('Failed to delete existing file: ' + output_file_path)
    if os.path.exists(output_file_path) == False:
        success = ai_utils.write_dict_to_file(data, output_file_path)
        # print("Yaml created: " + success)
    return success