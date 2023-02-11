import os
import sys
from pathlib import Path

import numpy as np
import torch
import torchvision.transforms as T
import torchvision.transforms.functional as TF
from PIL import Image
import matplotlib.pyplot as plt
from torch.utils.data import DataLoader
import re
from AnnotatedDataset import AnnotatedDataset, CLASS_TO_ID, draw_bboxes_on_image
from ImageTransformations import *
from tqdm import tqdm

#################
# Visualization #
#################

plt.rcParams["savefig.bbox"] = 'tight'
# if you change the seed, make sure that the randomly-applied transforms
# properly show that the image can be both transformed and *not* transformed!
torch.manual_seed(0)


def display_samples(samples, orig_sample, with_orig=True, row_title=None, figsize=(20, 10), max_in_row=4, **imshow_kwargs):
    # Bake bboxes into image
    imgs = [draw_bboxes_on_image(s['image'], s['labels']) for s in samples]
    orig_img = draw_bboxes_on_image(orig_sample['image'], orig_sample['labels'])

    def divide_chunks(l, n):
        # looping till length l
        for i in range(0, len(l), n):
            yield l[i:i + n]

    imgs = list(divide_chunks(imgs, max_in_row))

    if not isinstance(imgs[0], list):
        # Make a 2d grid even if there's just 1 row
        imgs = [imgs]

    num_rows = len(imgs)
    num_cols = len(imgs[0]) + with_orig
    fig, axs = plt.subplots(nrows=num_rows, ncols=num_cols, squeeze=False, figsize=figsize)
    for row_idx, row in enumerate(imgs):
        row = [orig_img] + row if with_orig else row
        for col_idx, img in enumerate(row):
            ax = axs[row_idx, col_idx]
            ax.imshow(np.asarray(img), **imshow_kwargs)
            ax.set(xticklabels=[], yticklabels=[], xticks=[], yticks=[])
    if with_orig:
        axs[0, 0].set(title='Original image')
        axs[0, 0].title.set_size(8)
    if row_title is not None:
        for row_idx in range(num_rows):
            axs[row_idx, 0].set(ylabel=row_title[row_idx])
    plt.tight_layout()


if __name__ == '__main__':
    #  Define paths
    ORIGINAL_DATA_PATH = '../data/original'
    OUTPUT_PATH = '../data/output'
    EPOCHS = 10

    # Transforms for data augmentation
    transform_list = [ChangeColorTemperature(), ColorJitter(brightness=0.4, contrast=0.4, saturation=0.3, hue=0.07), RandomScale(scale=0.1), RandomRotate(angle=10), RandomShear(shear_factor=0.2), RandomTranslate(translate=0.1), RandomHorizontalFlip(p=0.5)]
    transforms = Sequence(transforms=transform_list, probs=0.5)

    ##############################
    # Visualizing the transforms #
    ##############################
    # original_ds = AnnotatedDataset(original_path=ORIGINAL_DATA_PATH, transform=None)
    #
    # for i, sample in enumerate(original_ds):
    #     if i > 10:
    #         break
    #
    #     transformed_samples = []
    #     for _ in range(20):
    #         transformed_sample = transforms([sample])
    #         transformed_samples.append(transformed_sample[0])
    #
    #     display_samples(samples=transformed_samples, orig_sample=sample, with_orig=True, figsize=(20, 10), max_in_row=4)
    #     plt.show()

    ###########################
    # Transforming the images #
    ###########################
    original_ds = AnnotatedDataset(original_path=ORIGINAL_DATA_PATH, transform=transforms)
    image_output_path = Path(f'{OUTPUT_PATH}/images/')
    label_output_path = Path(f'{OUTPUT_PATH}/labels/')
    image_output_path.mkdir(exist_ok=True, parents=True)
    label_output_path.mkdir(exist_ok=True, parents=True)

    # Print table to convert from new file idx to old file names
    print("\n".join([f"{idx} \t {path.stem}" for idx, path in enumerate(original_ds.annot_paths)]), file=open(f"{OUTPUT_PATH}/references.txt", "w"))

    for epoch in tqdm(range(EPOCHS), desc="Epoch No."):
        for i, sample in tqdm(enumerate(original_ds), desc="Sample No.", leave=False, total=len(original_ds)):
            file_stem = f'{epoch}_{i}'

            # Write image
            sample['image'].save(image_output_path / f'{file_stem}.png')
            # Write label
            label_strings = [bbox.convert_yolo() for bbox in sample['labels']]
            print("\n".join(label_strings), file=open(label_output_path / f"{file_stem}.txt", 'w'))

