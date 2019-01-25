#!/usr/bin/env python
import numpy as np
import os
import cv2
import csv
# from PIL import Image, ImageDraw
import xmltodict
import random
WIDTH = 640
HEIGHT = 480
class_dictionary = {"doublemint": 1, "kinder": 2, "kusan": 3}

def xml2mask(xml_path):
    file_exist = os.path.isfile(xml_path)    # True
    mask = np.zeros([HEIGHT, WIDTH], dtype = np.int8)
    if file_exist:
        print xml_path
        with open(xml_path) as fd:
            label_dict = xmltodict.parse(fd.read())
    else:
        print "This path doesn't exist"
        print xml_path
        return mask
    for object_ in label_dict['annotation']['object']:
        if object_['name'] in class_dictionary and object_['deleted'] == '0':
            class_id = class_dictionary[object_['name']]
            poly_vertice = []
            for pts_idx in object_['polygon']['pt']:
                poly_vertice.append([int(pts_idx['x']), int(pts_idx['y'])])
            poly_vertice = np.array(poly_vertice, np.int32)
            cv2.fillConvexPoly(mask, poly_vertice, class_id)

    return mask

def main():
    # imagefolder_path = "/home/michael/Downloads/mini_competition_dataset/image"

    xmlfolder_path = "label"
    maskfolder_path = "mask"
    trainlist_path = "train.csv"
    vallist_path = "val.csv"
    if not os.path.isdir(maskfolder_path):
        os.makedirs(maskfolder_path)
    image_label_list = []
    scenes = os.listdir(xmlfolder_path)
    for scene in sorted(scenes):
        scene_path = os.path.join(xmlfolder_path, scene)
        mask_scene_path = os.path.join(maskfolder_path, scene)
        print scene_path
        if not os.path.isdir(mask_scene_path):
            os.makedirs(mask_scene_path)
        xml_filenames = os.listdir(scene_path)
        for xml_filename in sorted(xml_filenames):
            file_path = os.path.join(scene_path, xml_filename)
            mask = xml2mask(file_path)
            print (np.unique(mask))
            mask_file_name = xml_filename.split("xml")[0] + "png"
            save_path = os.path.join(mask_scene_path, mask_file_name)
            cv2.imwrite(save_path, mask,[int(cv2.IMWRITE_JPEG_QUALITY), 100])
            image_file_path = save_path.replace("mask", "image")
            image_file_path = image_file_path.replace("png","jpg")
            image_label_list.append([image_file_path, save_path])
        random.shuffle (image_label_list)
    list_size = len(image_label_list)
    train_lis = image_label_list[0: int(list_size*0.9)][:]
    val_lis = image_label_list[int(list_size*0.9):list_size][:]
    with open(trainlist_path, "wb") as f:
        writer = csv.writer(f)
        writer.writerows(train_lis)
    with open(vallist_path, "wb") as f:
        writer = csv.writer(f)
        writer.writerows(val_lis)

if __name__ == "__main__":
    main()
