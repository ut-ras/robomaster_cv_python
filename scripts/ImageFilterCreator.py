import os
import cv2
import numpy as np
import shutil
from PIL import Image
import tkinter
from tkinter import filedialog

tkinter.Tk().withdraw()  # prevents an empty tkinter window from appearing

# specify the path to the unzipped Yolo File
src = filedialog.askdirectory()
if len(src) == 0:
    exit()

kelvin_table = {
    1000: (255, 56, 0),
    1500: (255, 109, 0),
    2000: (255, 137, 18),
    2500: (255, 161, 72),
    3000: (255, 180, 107),
    3500: (255, 196, 137),
    4000: (255, 209, 163),
    4500: (255, 219, 186),
    5000: (255, 228, 206),
    5500: (255, 236, 224),
    6000: (255, 243, 239),
    6500: (255, 249, 253),
    7000: (245, 243, 255),
    7500: (235, 238, 255),
    8000: (227, 233, 255),
    8500: (220, 229, 255),
    9000: (214, 225, 255),
    9500: (208, 222, 255),
    10000: (204, 219, 255),
    11000: (200, 213, 255),
    12000: (195, 209, 255),
    13000: (190, 206, 255),
    14000: (182, 206, 255),
    15000: (179, 204, 255),
}


def convert_temp(image, temp):
    r, g, b = kelvin_table[temp]
    matrix = (
        r / 255.0,
        0.0,
        0.0,
        0.0,
        0.0,
        g / 255.0,
        0.0,
        0.0,
        0.0,
        0.0,
        b / 255.0,
        0.0,
    )
    return image.convert("RGB", matrix)


def change_Bright(img, val):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv = np.array(hsv, dtype=np.float64)
    val += 100
    val = val / 100  # dividing by 100 to get in range 0-1.5
    # scale pixel values up or down for channel 2(Value)
    hsv[:, :, 2] = hsv[:, :, 2] * val
    hsv[:, :, 2][hsv[:, :, 2] > 255] = 255  # setting values > 255 to 255.
    hsv = np.array(hsv, dtype=np.uint8)
    res = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    return res


filterNames = ["Brighter", "Darker", "Warmer", "Cooler"]
new_folder_names = [src + filter for filter in filterNames]


def makeFolderCopies():
    # create the new folders
    for folder_name in new_folder_names:
        if not os.path.exists(folder_name):
            # make new folder with the name extension
            os.makedirs(folder_name)

            # make new sub folder with labels
            destLabel = os.path.join(folder_name, "labels")
            os.makedirs(destLabel)

            # make new sub directory with filtered images
            destImage = os.path.join(folder_name, "images")
            os.makedirs(destImage)

            # copy labels from original folder to new folder
            sourceLabels = os.path.join(src, "labels")
            for file in os.listdir(sourceLabels):
                newFile = os.path.join(sourceLabels, file)
                if os.path.isfile(newFile):
                    shutil.copy(newFile, destLabel)

            # copy classes and notes files
            shutil.copy(os.path.join(src, "classes.txt"), folder_name)
            shutil.copy(os.path.join(src, "notes.json"), folder_name)


def generatedFilteredImages():
    srcImages = os.path.join(src, "images")
    # loop through all the files in the original photos folder
    for file in os.listdir(os.path.join(src, "images")):
        print(os.path.join(srcImages, file))
        # load the image
        img = cv2.imread(os.path.join(srcImages, file))
        im = Image.open(os.path.join(srcImages, file))

        # return the output of each filter to their corresponding name. True if using cv2, False if PIL
        filters = {
            "Brighter": (True, change_Bright(img, 40), new_folder_names[0]),
            "Darker": (True, change_Bright(img, -40), new_folder_names[1]),
            "Warmer": (False, convert_temp(im, 4000), new_folder_names[2]),
            "Cooler": (False, convert_temp(im, 12000), new_folder_names[3]),
        }

        # save the filtered images to the output folders
        for name, filter in filters.items():
            # get the right output folder
            output_path = os.path.join(filter[2], "images")

            # save the filtered image to the output folder.
            if filter[0]:
                cv2.imwrite(os.path.join(output_path, file), filter[1])
            else:
                filter[1].save(os.path.join(output_path, file))


makeFolderCopies()
generatedFilteredImages()
