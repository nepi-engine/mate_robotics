# NEPI Docker Host Setup Instructions


This repository contains libraries, scripts and other utilities for creating custom AI detection models for Yolo based frameworks.
The process has been tested with both YoloV8 and YoloV11 models

The following instructions walk through
A) Setting up the model training environment
B) Setting up a custom model training project
C) Training a custom ai yolo image model
D) Testing the custom model
E) Retraining the custom model

For additional support, see the documentation, tuturials, videos, and community forum available at NEPI.com:
[NEPI Website](https://www.nepi.com)

########################################
## ONE-TIME: ENVIRONMENT SETUP #######
########################################

### Setup Your Yolo AI Model Training Evironment

Follow the instrucitons in this section to setup 
your ai_training environment on your training computer

REQUIREMENTS
A Linux computer (or NEPI enabled processor)  with internet access
Python3 with pip installed

**NOTE:** These instructions were tested on computer running
ubuntu 20.04 and python 3.8.10

**NOTE:** These instructions could be addapted for a Windows or Mac PC

#########
### Create an 'ai_training' folder for your AI training projects
in a folder on the Linux computer you want to train on:

**NOTE:** If you are training on a NEPI enabled system,
make sure your are training on the NEPI device's user storage SSD drive,
and not in the NEPI file system's ~/ai_training folder, which has limited space.
There is an existing 'ai_training' folder on the user storage drive at:
/mnt/nepi_storage/ai_training.
You can jump to this folder by typing 'train' from any terminal on your NEPI device

  mkdir ai_training
  cd ai_training

#########
### Clone the nepi_ai_training repo

Make sure your device is connected to the internet or download 
the repo on a PC and copy the 'ai_training' folder.

  git clone https://github.com/nepi-engine/nepi_ai_training
  cd nepi_ai_training


#########
### Install the ai training software requirements 

Make sure your device is connected to the internet

**NOTE:** If software needs to be installed, the process step "Building wheel for lxml (pyproject.toml) may take a while, so let it run to completion.

**NOTE:** If you are training on an Intel xpu, also run 'pip install intel-extension-for-pytorch'
  
  ./ai_train_env_setup.sh

**NOTE:** After this process both label-studio and labelImg software packages will be installed.


########################################
## ONE-TIME: PROJECT FOLDER SETUP #######
########################################



### Run the 'project_setup.sh' script to create a new training project with
the name of the project folder you want.

  PROJECT_NAME=my_project_name
  source ai_train_yolo_project_setup.sh $PROJECT_NAME

This will create and populate a new training project in the same
folder the nepi_ai_training repo is located in.

The new project folder includes a base training model 
(Yolov8 Tiny yolov8n.pt) in the model_folder.  

**OPTIONAL**
Install additional base model files that include 
small, medium, and large versions for
more accurate (but slower and higher resource) models

    cd model_training

    wget 'https://www.dropbox.com/scl/fi/wri9vqhr81jjh78lx13nr/yolo_detector_base_models.zip?rlkey=6tmqaqwb09wwy30g6k568f3zv&st=k4rza4b3&dl=0' -O yolo_detector_base_models.zip

    unzip yolo_detector_base_models.zip

    rm yolo_detector_base_models.zip
    ls
    cd ..

    **NOTE:** Find information on the different start model options at this link:
    https://docs.ultralytics.com/tasks/detect/#models


########################################
## PROJECT INITIALIZATION #######
########################################

### Edit the project settings file 

Navigate to the base project folder containing the project python scripts.
Open and edit the 'project_settings.yaml' file using the information below.

A) Change the 'MODEL_NAME' value to your desired deployed model name.

B) Enter a description for your model in the 'DESCRIPTION' field.

C) Enter the list of class labels to use for labeling/training under the 'CLASSES' line.
each label shoudl be on its own line proceeded by a ' - '

**NOTE:** You can change these values anytime, remove labels, or add labels
and rerun the 'initialize_project.py' script in the next section.
**NOTE:** The order of items in this list should not be changed after running
the 'initialize_project.py' script.

D) Set the 'USE_PERCENT_DATA' value to adjust the percentage of image files
transfered from the 'data_raw' folders to use for labeling/training data.
**NOTE:** You can increase this value at any time without loosing your exesting labeled data files.

E) Set the 'RANDOM_DATA_SIZE' value to the number of random test images you would like use in a test run folder.

F) Select a starting model (model_name.pt) from the 'model_training' folder to use for your first training session
**NOTE:** Larger model sizes require more GPU memory.  If your training session is crashing, try a smaller model.
**NOTE:** Additional training sessions will use the last best model in the model_training folder as the start model
Find information on the different start model options at this link:
https://docs.ultralytics.com/tasks/detect/#models

G) Set the 'USE_BEST_MODEL' to false to start training from the set 'BASE_MODEL',
rather than an existing 'best.pt' created during the previous training session.
**NOTE:** If you change the 'BASE_MODEL' after training with a different base_model,
then you will need to set this to false to reset the model source

H) Change the 'IMAGE_SIZE' value to change the models native image size
that input images will be resized to during training and live detection processing
**NOTE:** While increasing this value will provide better detections on smaller image targets
it comes at a significant increase in detection time/latency

I) Change the 'NUM_EPOCHS', 'BATCH_SIZE', and 'NUM_WORKERS' values to adjust the training session parameters

**EXAMPLE** 'project_settings.yaml' File

MODEL_NAME: light_bulb
DESCRIPTION: light bulb object detector
CLASSES:
  - Can
  - Lamp

USE_PERCENT_DATA: 100
RANDOM_DATA_SIZE: 100
USE_BEST_MODEL: true 

BASE_MODEL: yolov8n.pt
IMAGE_SIZE: 640
NUM_EPOCHS: 300 
BATCH_SIZE: 8
NUM_WORKERS: 2


Run the project initialization python script.
Open terminal in the same folder and run

  sudo python initialize_project_yolo_detector.py


#########
### Populate the raw data folder with image file folders.

Add folders that include the image files you want
to use for training to the project's 'data_raw' folder.
Supported image file types -> ['jpg','JPG','jpeg','png','PNG']

**NOTE:** Put images in subfolders in the 'data_raw' folder,
not directly in the data_raw folder

**NOTE:** If you have images with existing label files (xml or txt), 
add to the folders that include the corresponding image files.
these label files will be copied to the labeling data folder during
the project initiation step in the next section.



#########
### Run the project initialization script

Navigate to the base project folder containing the project python scripts.
Open a terminal and run the following command:

  sudo python initialize_project_yolo_detector.py

The script performs the following processes:
1) The script will populate the labeling data folder from data in the raw data folder and
check any existing label files against the classes in the 'project_settings.yaml' file.
2) Updates the 'stats.txt' files for data folders with folder data information
3) Fixes permissions of project files and folders 


**NOTE:** This script should be run when:
1) New data is added to the raw data folder
2) Any changes to the 'CLASSES' label list in the 'project_settings.yaml' file.


**NOTE:** It is recommended to run through the remaining label,train,deploy,test processes
using the random data set produced to test and familiarize the processes before
trying to process all the data. 


########################################
## LABEL DATA #######
########################################

#########
### Run the data labeling script

Navigate to the base project folder containing the project python scripts.
Open a terminal and run the following command:

  sudo python label_data_yolo_detector.py


Select the data labeling folder from the prompted list, 
which will open an labelImg session in the selected data labeling folder.
Set labelImg session configuration
1) Under the 'View' memu item, turn on the 'Auto Save Mode' and 'Single Class Mode' options
2) Under the 'File' memu item, select the 'Change Save Directory' option and select the session image folder in the project's 'data_labeling folder'
Click the 'Create RectBox' from the sidebar, drag mouse over object to label, then select the label class from the popup menu.
Run through all the data in the folder using the selected label.  
Repeat the process for each class label, by turning off the the 'Single Class Mode', labeling the a target with the next class label, then turning back on the 'Single Class Mode'
When complete, close the application.

**NOTE:** Using the following hot-keys can speed up the process significantly:
'W' for creating a new label box
'd' next image

**NOTE:** Before going to far, check that your labels are saving to the correct folderd

**NOTE:** You can check the stats of the imgage and label files in the 'stats.txt' file in the data labeling folder

**NOTE:** Any updates applied to data label files during the fix labels process
will first copy the original '.xml' label file to a 'xml.org' file

The script performs the following processes:
1) Fixes permissions of project files and folders
2) Prompts user to select the data labeling folder for the current session
2) Starts the labelImg application for the selected data labeling folder
3) Check any label files against the classes in the 'project_settings.yaml' file.
4) Creates 'txt' label files from the 'xml' label files created
4) Updates the 'stats.txt' file for data labeling folders with folder data information
5) Fixes permissions of project files and folders 


**NOTE:** This script should be run :
1) For each folder in the data labeling folder
2) For making updates to existing labels
3) After rerunning the 'initialize_project.py' script for new data


########################################
## TRAIN MODEL #######
########################################

### Run the model training script

Navigate to the base project folder containing the project python scripts.
Open a terminal and run the following command:

  sudo python train_model_yolo_detector.py

The script performs the following processes:
1) Fixes permissions of project files and folders
2) Creates (or updates) the train,val,test image lists used for training
3) Starts a model training session using values set in the 'project_settings.yaml' file
4) Fixes permissions of project files and folders 

**NOTE:** Training will run until:
1) The model reaches low enough loss score on the test data
2) The model runs through the set number of Epochs
3) You hit "Ctrl=C" to stop the training session

**NOTE:** You can rerun this script to retrain the last best model if additional data has been labeled to improve your mode.


########################################
## CREATE DEPLOY MODEL #######
########################################


### Run the deploy model script

Navigate to the base project folder containing the project python scripts.
Open a terminal and run the following command:

  sudo python deploy_model_yolo_detector.py

**NOTE:** If the script found a trained model in the training folder, you should now see three files in the 'model_deploy' folder:
1) '.py' weights file
2) '.yaml' model info file
3) '.txt' result file


The script performs the following processes:
1) Fixes permissions of project files and folders
2) Searches the model training folder for the latest 'best.pt' trained weights file and results file,
and copies them to the model deploy folder renamed to the model name + base_model + image size 
3) Creates a corresponding '.yaml' model info file with the same name
4) Fixes permissions of project files and folders 


**NOTE:** You should rerun this script after every training session to create the latest best model deployment package

########################################
## DEPLOY & TEST MODEL #######
########################################

In this section you will copy your custom model to the NEPI device's ai_models library,
enable your model one the NEPI AI Model Manager page,
connect your model to a live camera stream (or test imgs/videos stream using one of NEPI's built in File Pub applications),
enable and test your models performance
 

1) Copy the files from the projects's model deploy folder to the appropriate
framework folder on your NEPI device's 'ai_models' user folder.
(ie. yolov8 models should be copied to the /nepi_storage/ai_models/yolosv8 folder)

2) Restart your NEPI Device
3) Open the RUI System/AI Model Manager page and enable the framework and new custom model if not allready enabled
4) Open the RUI AI System/AI Detector Manager page and connect your image stream(s), then enable the detector

########################################
## Retrain MODEL #######
########################################
You can retrain your model whenever needed to improve desired performance.
 
1) Add new data to the projects raw data folder
2) Change, remove, or adjust class labels in the 'project_settings.yaml' file
3) Change the base model to a smaller or larger model network in the 'project_settings.yaml' file
**NOTE:** After making any changes above, rerun the project initalization script:

  sudo python initialize_project_yolo_detector.py

4) Add, remove, or adjust labeled boxes in your data labeling folder by reruning the label data script
**NOTE:** After making any changes above, rerun the data labeling script for any effected data folders:

  sudo python label_data_yolo_detector.py

5) Retrain the model 
**NOTE:** After making any changes above, rerun the model training script:

  sudo python train_model_yolo_detector.py

6) Update the deploy model files 
**NOTE:** After making any changes above, rerun the deploy model script:

  sudo python deploy_model_yolo_detector.py

7) Deploy and Test your updated model following the instructions in the previous section

