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
# Utility functions for ai training scripts
############################

imports = True
try:
    import os
    import re
    import sys
    sys.tracebacklimit = None
    import grp
    import pwd
    import subprocess
    import glob
    import fileinput
    import random
    import yaml
    import shutil
    import logging
    import declxml as xml
    from PIL import Image
    import shlex
    import getpass
    import xml.etree.ElementTree as ET
    

except Exception as e:
    print("Missing required python modules " + str(e))
    print("Connect to internet and run the following in this folder")
    print("sudo pip3 install -r requirements.txt")
    print("Then try rerunning this script agian")
    imports = False

##########################################
# PORJECT SETTINGS - Edit as Necessary
##########################################

PROJECT_FILE = 'project_settings.yaml'

DATA_RAW_FOLDER = 'data_raw'
DATA_LABEL_FOLDER = 'data_labeling'
MODEL_TRAIN_FOLDER = 'model_training'
MODEL_DEPLOY_FOLDER = 'model_deploy'

IMAGE_FILE_TYPES = ['jpg','JPG','jpeg','png','PNG']

RANDOM_FILE_NAME = 'random_set'
CLASSES_FILE_NAME = 'classes.txt'
STATS_FILE_NAME = 'stats.yaml'


##########################################
# System Variables
##########################################
CURRENT_FOLDER = os.path.realpath(__file__)


##########################################
# AI Training Utility Functions
##########################################


def check_for_sudo():
    # Check if the effective user ID is not 0 (root)
    if os.geteuid() != 0:
        print("Error: This script must be run with sudo or as root.")
        sys.exit(1) # Exit with a non-zero status code to indicate an error
    else:
        return True

def get_user_id(folder = CURRENT_FOLDER):
    stat_info = os.stat(folder)
    uid = stat_info.st_uid
    gid = stat_info.st_gid

    user = pwd.getpwuid(uid)[0]
    group = grp.getgrgid(gid)[0]
    #print([self.user, self.group])
    return user,group

def get_clean_name(name, invalid_chars = r'[<>:"/\\|?*\x00-\x1F]', replacement='_'):
    return re.sub(invalid_chars, replacement, name)

def make_folder(folder_path, user = None, group = None):
    success = False
    try:
        os.mkdir(folder_path)
        fix_folder_permissions(folder_path, user = user, group = user)
        success = True
    except Exception as e:
        print("Failed to make folder: " + folder_path + " " + str(e))
    return success

def fix_folder_permissions(folder_path, user = None, group = None):
    success = True
    [fuser,fgroup] = get_user_id(folder_path)
    if user is None:
        user = fuser
    if group is None:
        group = fgroup
    print("setting permissions for folder: " + folder_path + " to " + user + ":"  + group)
    if os.path.exists(folder_path) == True:
        try:
            os.system('chown -R ' + user + ':' + group + ' ' + folder_path) # Use os.system instead of os.chown to have a recursive option
            #os.chown(full_path_subdir, user, group)
            os.system('chmod -R 0775 ' + folder_path)
        except Exception as e:
            success = False
            print("Failed to update folder permissions: " + folder_path + " " + str(e))
    return success


def get_folder_list(folder_path):

  folder_list=[]
  if os.path.exists(folder_path):
    filelist=os.listdir(folder_path + '/')
    #print('')
    #print('Files and Folders in Path:')
    #print(folder_path)
    #print(filelist)
    for i, file in enumerate(filelist):
        #print(file)
        foldername = (folder_path + '/' + file)
        #print('Checking file: ')
        #print(foldername)
        if os.path.isdir(foldername): # file is a folder
            folder_list.append(foldername)
  return folder_list

def get_file_list(folder_path, ext_list = None):
  all_files = ext_list is None
  file_list = []
  if os.path.exists(folder_path):
    filelist=os.listdir(folder_path + '/')
    #print('')
    #print('Files and Folders in Path:')
    #print(folder_path)
    #print(filelist)
    for i, file in enumerate(filelist):
        #print(file)
        filename = (folder_path + '/' + file)
        #print('Checking file: ')
        #print(filename)
        if os.path.isdir(filename) == False: # file is a folder
            f_split = os.path.splitext(file)
            if len(f_split) > 1:
                f_ext = os.path.splitext(file)[1]
                f_ext_clean = f_split[1].replace(".","")
            if all_files == True or f_ext in ext_list or f_ext_clean in ext_list:
                file_list.append(filename)
  return file_list


def open_new_file(file_path):
  print('')
  if os.path.isfile(file_path):
    print('Deleting existing file:')
    print(file_path)
    os.remove(file_path)
  print('Creating new file: ' + file_path)
  fnew = open(file_path, 'w')
  return fnew

def read_list_from_file(file_path):
    lines = []
    with open(file_path) as f:
        lines = [line.rstrip() for line in f] 
    return lines

def write_list_to_file(data_list, file_path):
    success = True
    if os.path.exists(file_path) == True:
        os.remove(file_path)
    try:
        with open(file_path, 'w') as file:
            for data in data_list:
                file.write(data + '\n')
    except Exception as e:
        print("Failed to write list to file " + file_path + " " + str(e))
        success = False
    return success



def read_dict_from_file(file_path):
    dict_from_file = None
    if os.path.exists(file_path):
        try:
            with open(file_path) as f:
                dict_from_file = yaml.load(f, Loader=yaml.FullLoader)
        except Exception as e:
            print("Failed to get dict from file: " + file_path + " " + str(e))
    else:
        print("Failed to find dict file: " + file_path)
    return dict_from_file


def write_dict_to_file(dict_2_save,file_path,defaultFlowStyle=False,sortKeys=False):
    success = False
    try:
        with open(file_path, "w") as f:
            yaml.dump(dict_2_save, stream=f, default_flow_style=defaultFlowStyle, sort_keys=sortKeys)
        success = True
    except Exception as e:
        print("Failed to write dict: "  + " to file: " + file_path + " " + str(e))
    return success


def copy_file(file_path, destination_path):
    success = False
    output_path = destination_path.replace(" ","_")
    #print("Checking on file copy: " + file_path + " to: " + output_path)
    if os.path.exists(output_path) == False:
        try:
            shutil.copy(file_path, output_path)
            #print("File: " + file_path + " Copied to: " + output_path)
            success = True
        except FileNotFoundError:
            print("Error file " + file_path + "not found") 
        except Exception as e:
            print("Excepton: " + str(e))
    return success 


def get_folder_files(folder_path):
    files_dict = dict()
    if os.path.exists(folder_path) == False:
        print('Get stats folder not found: ' + folder_path)
    else:
        path, dirs, files = next(os.walk(folder_path))
        for file in files:
            f_ext = os.path.splitext(file)[1]
            f_ext = f_ext.replace(".","")
            if f_ext not in files_dict.keys():
                files_dict[f_ext] = [file]
            else:
                files_dict[f_ext].append(file)        
    return files_dict

def get_num_files(folder_path):
    img_files = []
    xml_files = []
    txt_files = []
    if os.path.exists(folder_path) == False:
        print('Get stats folder not found: ' + folder_path)
    else:
        path, dirs, files = next(os.walk(folder_path))
        for file in files:
            f_ext = os.path.splitext(file)[1]
            f_ext = f_ext.replace(".","")
            if f_ext in IMAGE_FILE_TYPES:
                img_files.append(file)
            if f_ext == 'xml':
                xml_files.append(file)
            if f_ext == 'txt':
                txt_files.append(file)
    return img_files,xml_files,txt_files


def update_stats_file(folder_path):
    stats_dict = dict()
    if os.path.exists(folder_path) == False:
        print('Stats update folder not found: ' + folder_path)
    else:
        print('Updating stats dict for folder: ' + folder_path)
        stats_file = os.path.join(folder_path,STATS_FILE_NAME)
        stats_dict['ALL_FOLDERS'] = {
            'num_img_files': 0,
            'num_xml_files': 0,
            'num_txt_files': 0
        }   
        folders_to_process=get_folder_list(folder_path)
        #print('Found folders: ' + str(folders_to_process))
        for folder in folders_to_process:
            #print('Gathering stats for folder: ' + folder)
            [img_files,xml_files,txt_files] = get_num_files(folder)
            folder_name = os.path.basename(folder)
            stats_dict[folder_name] = {
                'num_img_files': len(img_files),
                'num_xml_files': len(xml_files),
                'num_txt_files': len(txt_files)
            }

        # Update All Folders Stats
        num_img_files = 0
        num_xml_files = 0
        num_txt_files = 0
        for key in stats_dict.keys():
            if key != 'ALL_FOLDERS':
                num_img_files += stats_dict[key]['num_img_files']
                num_xml_files += stats_dict[key]['num_xml_files']
                num_txt_files += stats_dict[key]['num_txt_files']
        stats_dict['ALL_FOLDERS'] = {
            'num_img_files': num_img_files,
            'num_xml_files': num_xml_files,
            'num_txt_files': num_txt_files
        }
        success = write_dict_to_file(stats_dict,stats_file)
    return stats_dict


def create_classes_dict(classes):
    classes_dict = dict()
    for i, label in enumerate(classes):
        classes_dict[label] = i
    return classes_dict




def fix_brocken_labels(labels, classes, classes_dict):  
    for label in labels:      
        if label not in classes_dict.keys():
            options = classes + ['Add To List','Add New Label','Remove']
            print("")
            print("******")
            print("Unable to find label label: " + str(label))

            print("")
            print("Choose option from list to proceed")
            index = None
            while index is None:
                display_menu(options)
                sel_ind,sel_option = get_user_choice(options)
                if sel_ind < len(classes):
                    index = sel_ind
                elif sel_ind == len(classes):
                    classes.append(label)
                    index = classes.index(label)
                elif sel_ind == (len(classes) + 1):
                    label = get_new_label()
                    classes.append(label)
                    index = classes.index(label)
                elif sel_ind == (len(classes) + 2):
                    index = -1
                else:
                    print('')
                    print('Invalid option selected')
            print('Updating classes_dict: ' + label + " : " + str(index))
            classes_dict[label] = index
  
    return classes, classes_dict


def read_detect_dict_from_xml_file(file_path,classes = []):
    detect_dict_list = []
    f_ext = os.path.splitext(file_path)[1]
    if f_ext == '.xml':    
        tree = ET.parse(file_path)
        root = tree.getroot()
        labels = []
        bboxes = []
        size = root.find("size")
        image_width = 1.0 * int(size.find("width").text)
        image_height = 1.0 * int(size.find("height").text)
        for ind, o_entry in enumerate(root.findall("object")):
            try:
                label = o_entry.find("name").text
                labels.append(label)
                if label not in classes:
                    class_ind = -1
                else:
                    class_ind = classes.index(label)
                    
                box = o_entry.find("bndbox")
                xmax = int(box.find("xmax").text)
                xmin = int(box.find("xmin").text)
                ymax = int(box.find("ymax").text)
                ymin = int(box.find("ymin").text)

                absolute_x = xmin + 0.5 * (xmax - xmin)
                absolute_y = ymin + 0.5 * (ymax - ymin)

                absolute_width = xmax - xmin
                absolute_height = ymax - ymin

                x = absolute_x / image_width
                y = absolute_y / image_height
                width = absolute_width / image_width
                height = absolute_height / image_height

                det_area = (xmax - xmin) * (ymax - ymin)
                det_prob = 1
                detect_dict = {
                    'name': label, # Class String Name
                    'id': class_ind, # Class Index from Classes List
                    'uid': '', # Reserved for unique tracking by downstream applications
                    'prob': det_prob, # Probability of detection
                    'xmin': xmin,
                    'ymin': ymin,
                    'xmax': xmax,
                    'ymax': ymax,
                    'area_pixels': int(det_area),
                    'area_ratio': det_area / (image_width * image_height)
                }
                detect_dict_list.append(detect_dict)
            except Exception as e:
                print("Failed to convert xml data to detect data: " + str(e))

    return detect_dict_list, image_width, image_height




def update_xml_label_file(file_path,classes,classes_dict):
    success = False
    f_ext = os.path.splitext(file_path)[1]
    if f_ext == '.xml':    
        tree_orig = ET.parse(file_path)
        tree = ET.parse(file_path)
        root = tree.getroot()
        for ind, o_entry in enumerate(root.findall("object")):
            label = o_entry.find("name").text
            if label in classes_dict.keys():
                index = classes_dict[label]
                if index != -1:
                    o_entry.find("name").text = classes[index]
                    label = o_entry.find("name").text
                    #print(label)
                else:
                    root.remove(o_entry)
            else:
                print('No match for label: ' + label)
        try:
            orig_file = file_path+'.org'
            copy_file(file_path,orig_file)
            #print('Saved original annotation labels: ' + orig_file)
            success = True
        except Exception as e:
            print('Failed to copy labels to file: ' + orig_file)
        try:
            tree.write(file_path)
            #print('Updated annotation labels in file: ' + file_path)
            success = True
        except Exception as e:
            print('Failed to save annotation labels to file: ' + file_path)
    return success




def read_detect_dict_from_txt_file(file_path,image_width,image_height,classes_list = []):
    dets_list = []
    f_ext = os.path.splitext(file_path)[1]
    if f_ext == '.txt':    
        with open(file_path,'r') as f:
            reader = csv.reader(f, delimiter=' ') 
        for row in reader:
            if len(row) == 5:
                try:
                    dets_int = []
                    for i, entry in enumerate(row):
                        dets_int.append(float(row[i]))
                        dets_list.append(dets_int)
                except Exception as e:
                    print('Failed top convert det strings to floats: ' + str(e))
            
        detect_dict_list = []
        for det in dets_list: 

            try:
                class_id = int(det[0])       
                if len(classes_list) > (class_id):
                    class_name = classes_list[class_id]
                else:
                    class_name = 'class_' + str(id)
                    det_name = class_name
                    det_id = class_id
                    det_prob = 1
                    xwidth = det[3]/2.0
                    xmin = int(image_width * (det[1] - xwidth))
                    xmax = int(image_width * (det[1] + xwidth))
                    yheight = det[4]/2.0
                    ymin = int(image_height * (det[2] - yheight))
                    ymax = int(image_height * (det[2] + yheight))
                    det_area = (xmax - xmin) * (ymax - ymin)
                    detect_dict = {
                        'name': det_name, # Class String Name
                        'id': det_id, # Class Index from Classes List
                        'uid': '', # Reserved for unique tracking by downstream applications
                        'prob': det_prob, # Probability of detection
                        'xmin': xmin,
                        'ymin': ymin,
                        'xmax': xmax,
                        'ymax': ymax,
                        'area_pixels': int(det_area),
                        'area_ratio': det_area / (image_width * image_height)
                    }
                    detect_dict_list.append(detect_dict)
                    #self.msg_if.pub_info("Got detect dict entry: " + str(detect_dict))
            except Exception as e:
                print("Failed to convert txt data to detect data: " + str(e))
    return detect_dict_list


def save_txt_label_file(bounding_boxes,file_path):
    success = False
    lines = []
    for box in bounding_boxes:
        lines.append("%d %.6f %.6f %.6f %.6f" % (box[0],box[1],box[2],box[3],box[4]))
    success = write_list_to_file(lines,file_path)
    return success


def convert_dict_to_txt(detect_dict_list,image_width,image_height):
    labels = []
    bboxes = []
    for detect_dict in detect_dict_list:
        try:
            class_ind = detect_dict['id']
                
            xmax = detect_dict['xmax']
            xmin = detect_dict['xmin']
            ymax = detect_dict['ymax']
            ymin = detect_dict['ymin']

            absolute_x = xmin + 0.5 * (xmax - xmin)
            absolute_y = ymin + 0.5 * (ymax - ymin)

            absolute_width = xmax - xmin
            absolute_height = ymax - ymin

            x = absolute_x / image_width
            y = absolute_y / image_height
            width = absolute_width / image_width
            height = absolute_height / image_height

            bbox = [class_ind, x, y, width, height]                


            labels.append(detect_dict['name'])
            bboxes.append(bbox)
        except Exception as e:
            print("Failed to convert xml data to txt data: " + str(e))
    return labels, bboxes


def convert_xml_to_txt(folder_path, classes, classes_dict):
    files = get_file_list(folder_path, ext_list = ['xml'])
    for file in files:
        [detect_dict_list, image_width, image_height] = read_detect_dict_from_xml_file(file,classes)
        [labels, bboxes] = convert_dict_to_txt(detect_dict_list,image_width,image_height)
        broken_labels = [i for i, box in enumerate(bboxes) if box[0] == -1]
        if len(broken_labels) > 0:
            [new_classes, new_classes_dict] = fix_brocken_labels(labels,classes,classes_dict)
            classes = new_classes
            classes_dict = new_classes_dict
            new_bboxes = []
            for i, label in enumerate(labels):
                if label in classes_dict.keys():
                    ind = classes_dict[label]
                    if ind != -1:
                        bbox = bboxes[i]
                        bbox[0] = ind
                        new_bboxes.append(bbox)
            bboxes = new_bboxes
            success = update_xml_label_file(file,classes,classes_dict)
        txt_file = file.replace('.xml','.txt')
        success = save_txt_label_file(bboxes,txt_file)
    return classes,classes_dict



def update_label_files(classes, classes_file, label_folder):
    success = True
    labeled_images = 0
    unlabled_images = 0
    unlabled_label_files = []

    ## Update Data Labeling Folder
    new_classes = classes
    orig_classes = classes
    if os.path.exists(classes_file) == False:
        orig_classes = classes
        write_list_to_file(classes,classes_file)
    else:
        orig_classes = read_list_from_file(classes_file)
        if orig_classes != classes:
            print('Updated classes.txt labels from ' + str(orig_classes) + ' to ' + str(classes))
            write_list_to_file(classes,classes_file)
    
    folders_to_process=get_folder_list(label_folder)
    classes_dict = dict()
    for i, label in enumerate(classes):
        classes_dict[label] = i
    for folder in folders_to_process:       
        print('Preparing txt label files in: ' + str(folder))
        labels_changed = orig_classes != new_classes
        has_labels = False
        for f in os.listdir(folder):
            if f.endswith(".xml"):  
                if has_labels == False:
                    print('')
                    print('**************************')
                    print('Updating xml label files in folder : ' + str(folder))
                has_labels = True
            if has_labels == True:       
                [convert_classes,classes_dict] = convert_xml_to_txt(label_folder,new_classes,classes_dict)
                if convert_classes != new_classes:
                    print('Saving classes file to: ' + str(classes_file))
                    write_list_to_file(convert_classes, classes_file)
                    new_classes = convert_classes
        folder_classes_file = os.path.join(folder,CLASSES_FILE_NAME)
        print('Saving classes file to: ' + str(folder_classes_file))
        write_list_to_file(new_classes, folder_classes_file)
    return new_classes



def fix_data_files(source_path):
    imgs_list = []
    print('Looking for source folder: ' + source_path)
    if os.path.exists(source_path) == False:
        print('Source update folder not found: ' + source_path)
        return False
    success = False
    folders_to_process=get_folder_list(source_path)
    print('Fixing files in source folders: ' + str(folders_to_process))
    fixed_files = []
    for source_folder in folders_to_process:
        [img_files,xml_files,txt_files] = get_num_files(source_folder)
        files = img_files + xml_files + txt_files
        for file in files:
            cfile = file.replace(' ','_')
            if file != cfile:
                old_file_name = os.path.join(source_folder,file)
                new_file_name = os.path.join(source_folder,cfile)
                if os.path.exists(new_file_name):
                    os.remove(new_file_name)
                try:
                    # Rename the file
                    os.rename(old_file_name, new_file_name)
                    #print(f"File '{old_file_name}' renamed to '{new_file_name}' successfully.")
                    fixed_files.append(old_file_name)
                except FileNotFoundError:
                    print(f"Error: File '{old_file_name}' not found.")
                except FileExistsError:
                    print(f"Error: A file named '{new_file_name}' already exists.")
                except Exception as e:
                    print(f"An unexpected error occurred: {e}")
    return fixed_files    



def update_labling_data(source_path, output_path, use_percent_data = 100):
    imgs_list = []
    print('Looking for source folder: ' + source_path)
    if os.path.exists(source_path) == False:
        print('Source update folder not found: ' + source_path)
        return False

    print('Looking for label folder: ' + output_path)
    if os.path.exists(output_path) == False:
        try:
            print('Creating labeling folder: ' + output_path)
            make_folder(output_path)  
        except Exception as e:
            print('Failed to create labeling folder: ' + output_path + ' ' + str(e))
            return False

    success = False
    folders_to_process=get_folder_list(source_path)
    print('Updating from source folders: ' + str(folders_to_process))
    for source_folder in folders_to_process:
        [img_files,xml_files,txt_files] = get_num_files(source_folder)
        #print('Found Source files: ' + str([img_files,xml_files,txt_files]))
        source_name = os.path.basename(source_folder)
        output_folder = os.path.join(output_path,source_name)
        [limg_files,lxml_files,ltxt_files] = [[],[],[]]
        if os.path.exists(output_folder) == False:
            try:
                print('Creating label folder: ' + output_folder)
                make_folder(output_folder)  
            except Exception as e:
                print('Failed to create labeling folder: ' + output_folder + ' ' + str(e))
                return False
        else:
            print('Gathering data info for label folder: ' + output_folder)
            [limg_files,lxml_files,ltxt_files] = get_num_files(output_folder)
            #print('Found Dest files: ' + str([limg_files,lxml_files,ltxt_files]))

        # add random data form source as needed
        num_images = len(img_files)
        label_size = len(limg_files)
        if num_images <= 0:
            continue
        label_percent = (label_size / num_images) * float(100)
        print('Updating folder: ' + source_folder + ' with stats ' + str([num_images,label_size,label_percent]))
        #print('Starting folder: ' + output_folder + ' with files ' + str([limg_files,lxml_files,ltxt_files]))
        copy_files = []
        attempts = 0
        while label_percent < use_percent_data and attempts < (10  * num_images):

            if use_percent_data < 100:
                random_img = random.choice(img_files)
            elif num_images >= attempts:
                random_img = img_files[attempts]
            random_img_path = os.path.join(source_folder,random_img)
            attempts += 1
            #print('Checking image file: ' + str(random_img_path))
            if random_img not in limg_files:
                valid = check_image_file(random_img_path)
                if valid == False:
                    print('Skipping bad image file: ' + str(random_img_path))
                else:
                    #print('Adding image file: ' + str(random_img_path))
                    limg_files.append(random_img)
                    copy_files.append(random_img)
                    f_ext = os.path.splitext(random_img)[1]
                    xml_file = random_img.replace(f_ext,'.xml')
                    xml_file_path = os.path.join(source_folder,xml_file)
                    xml_exists = os.path.exists(xml_file_path)
                    
                    if xml_file not in lxml_files and xml_exists:
                        #print('Adding xml file: ' + str(xml_file))
                        copy_files.append(xml_file)
                    txt_file = random_img.replace(f_ext,'.txt')
                    txt_file_path = os.path.join(source_folder,txt_file)
                    txt_exists = os.path.exists(txt_file_path)
                    if txt_file not in ltxt_files and txt_exists:
                        #print('Adding txt file: ' + str(txt_file))
                        copy_files.append(txt_file)
        
            label_size = len(limg_files)
            label_percent = (label_size / num_images) * float(100)
            #print('Looping folder: ' + source_folder + ' with stats ' + str([label_size,label_percent]))
        print('Finished label data selection with attempts: ' + str(attempts))
        for img_file in limg_files:
            imgs_list.append(os.path.join(source_folder,img_file))
        
        print('Copying files from folder: ' + source_folder)
        print('Copying files to folder: ' + output_folder)
        #print('Copying files: ' + str(copy_files))

        
        for file in copy_files:
            file_path = os.path.join(source_folder,file)
            dest_path = os.path.join(output_folder,file)
            #print('Copying file: ' + str(file_path) + " to " +  str(dest_path))
            success = copy_file(file_path, dest_path)       

    return imgs_list     


def create_random_data_set(source_image_list,random_folder_path,random_data_size):

    num_images = len(source_image_list)
    print("Starting random data selection with num_images: " + str(num_images))

    if num_images < random_data_size:
        random_data_size = num_images
    if random_data_size == 0:
        return []

    ind = 0
    exists = True
    while exists == True:
        ind += 1
        random_folder = random_folder_path + '_' + str(ind)
        exists = os.path.exists(random_folder)
    try:
        make_folder(random_folder)
        fix_folder_permissions
    except Exception as e:
        print('Failed to create random data folder: ' + random_folder + ' ' + str(e))
        return []  
    img_files = []
    copy_files = []
    attempts = 0
    count = 0
    while count < random_data_size and attempts < (10  * num_images) :
        attempts += 1
        random_img = random.choice(source_image_list)
        #print('Checking image file: ' + str(random_img))

        valid = check_image_file(random_img)
        if valid == False:
            print('Skipping bad image file: ' + str(random_img))
        elif random_img not in copy_files:
            #print('Adding image file: ' + str(random_img))
            img_files.append(random_img)
            copy_files.append(random_img)
            count += 1
            f_ext = os.path.splitext(random_img)[1]
            xml_file = random_img.replace(f_ext,'.xml')
            xml_exists = os.path.exists(xml_file)
            if xml_exists:
                #print('Adding xml file: ' + str(xml_file))
                copy_files.append(xml_file)
            txt_file = random_img.replace(f_ext,'.txt')
            txt_exists = os.path.exists(txt_file)
            if txt_exists:
                #print('Adding txt file: ' + str(txt_file))
                copy_files.append(txt_file)
    print('Finished random data selection with attempts: ' + str(attempts))
    for file in copy_files:
        try:
            
            #print("File: " + image_file_path + " Copied to: " + random_folder)

            shutil.copy(file, random_folder)
            #print("File: " + image_file_path + " Copied to: " + random_folder)
        except FileNotFoundError:
            print("Error file " + copy_files + "not found") 
        except Exception as e:
            print("Excepton: " + str(e))
    
    return img_files



def display_menu(options):
    for i, option in enumerate(options, 1):
        print(f"{i}. {option}")

def get_user_choice(options):
    while True:
        try:
            choice = int(input("\nEnter your choice: "))
            if 1 <= choice <= len(options):
                ind = choice - 1
                return ind,options[ind]
            else:
                print("Invalid choice. Please enter a valid number.")
        except ValueError:
            print("Invalid input. Please enter a number.")

def get_new_label():
    while True:
        try:
            name = int(input("Enter a label name: "))
            if name == '':
                print("Invalid input. Please try again")
            else:
                return name
        except ValueError:
            print("Invalid input. Please try again")




def check_image_file(file_path):
    valid = False
    file = os.path.basename(file_path)
    f_ext = os.path.splitext(file)[1]
    f_ext = f_ext.replace(".","")
    if f_ext in IMAGE_FILE_TYPES and os.path.exists(file_path) == True:
      try:
        img = Image.open(file_path) # open the image file
        img.verify()
        valid = True
      except:
        #print('File not image: ' + str(file_path))
        pass
    else:
        print('Image file not found: ' + str(file_path))
    return valid

def remove_bad_label_files(folder_path):
  print("Checking for bad images in folder: " + folder_path)
  path, dirs, files = next(os.walk(folder_path))
  data_size = len(files)
  ind = 0
  for f in files:
    f_ext = os.path.splitext(f)[1]
    f_ext = f_ext.replace(".","")
    if f_ext in IMAGE_FILE_TYPES:
      #print('Found image file')
      image_file = (folder_path + '/' + f)
      valid = check_image_file(image_file)
      if valid == False:
        print('')
        print('Found bad image file:')
        print(image_file) # print out the names of corrupt files
        print('Deleting file')
        os.remove(image_file)
        print('Looking for label files')
        label_file = (folder_path + '/' + f.split(f_ext)[0]+'xml')
        #print(label_file)
        if os.path.exists(label_file):
          print('Found xml label file for bad image:')
          print(label_file) # print out the names of corrupt files
          print('Deleting file')
          os.remove(label_file)
        label_file = (folder_path + '/' + f.split(f_ext)[0]+'txt')
        #print(label_file)
        if os.path.exists(label_file):
          print('Found txt label file for bad image:')
          print(label_file) # print out the names of corrupt files
          print('Deleting file')
          os.remove(label_file)
        else:
          print('No label file found for bad image')



