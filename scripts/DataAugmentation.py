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
import pytest
import glob
from sklearn.model_selection import train_test_split
import shutil

os.environ["KMP_DUPLICATE_LIB_OK"] = "True"

#################
# Visualization #
#################

plt.rcParams["savefig.bbox"] = "tight"
# if you change the seed, make sure that the randomly-applied transforms
# properly show that the image can be both transformed and *not* transformed!
torch.manual_seed(0)


def display_samples(
    samples,
    orig_sample,
    with_orig=True,
    row_title=None,
    figsize=(20, 10),
    max_in_row=4,
    **imshow_kwargs,
):
    # Bake bboxes into image
    imgs = [draw_bboxes_on_image(s["image"], s["labels"]) for s in samples]
    orig_img = draw_bboxes_on_image(orig_sample["image"], orig_sample["labels"])

    def divide_chunks(l, n):
        # looping till length l
        for i in range(0, len(l), n):
            yield l[i : i + n]

    imgs = list(divide_chunks(imgs, max_in_row))

    if not isinstance(imgs[0], list):
        # Make a 2d grid even if there's just 1 row
        imgs = [imgs]

    num_rows = len(imgs)
    num_cols = len(imgs[0]) + with_orig
    fig, axs = plt.subplots(
        nrows=num_rows, ncols=num_cols, squeeze=False, figsize=figsize
    )
    for row_idx, row in enumerate(imgs):
        row = [orig_img] + row if with_orig else row
        for col_idx, img in enumerate(row):
            ax = axs[row_idx, col_idx]
            ax.imshow(np.asarray(img), **imshow_kwargs)
            ax.set(xticklabels=[], yticklabels=[], xticks=[], yticks=[])
    if with_orig:
        axs[0, 0].set(title="Original image")
        axs[0, 0].title.set_size(8)
    if row_title is not None:
        for row_idx in range(num_rows):
            axs[row_idx, 0].set(ylabel=row_title[row_idx])
    plt.tight_layout()


def copy_files_to_folder(list_of_files, destination_folder):
    for f in list_of_files:
        try:
            shutil.copy(f, destination_folder)
        except Exception as e:
            print(f"Error in moving {f} to {destination_folder} - {e}")


def create_train_val_test(original_path, output_path, overwrite):
    image_paths = [Path(x) for x in glob.glob(f"{original_path}/images/*.png")]
    annot_paths = [Path(x) for x in glob.glob(f"{original_path}/annotations/*.xml")]

    image_path_dict = {x.stem: x for x in image_paths}
    annot_path_dict = {x.stem: x for x in annot_paths}

    # Make sure there is a one to one mapping between images and annotations
    assert len(image_paths) == len(annot_paths)
    assert len(image_path_dict) == len(annot_path_dict)
    assert image_path_dict.keys() == annot_path_dict.keys()

    image_paths = sorted(image_paths, key=lambda x: x.stem)
    annot_paths = sorted(annot_paths, key=lambda x: x.stem)

    (
        train_image_paths,
        test_val_image_paths,
        train_annot_paths,
        test_val_annot_paths,
    ) = train_test_split(
        image_paths,
        annot_paths,
        test_size=TEST_SIZE + VALIDATION_SIZE,
        train_size=TRAIN_SIZE,
        random_state=42,
    )

    (
        val_image_paths,
        test_image_paths,
        val_annot_paths,
        test_annot_paths,
    ) = train_test_split(
        test_val_image_paths,
        test_val_annot_paths,
        test_size=TEST_SIZE / (TEST_SIZE + VALIDATION_SIZE),
        train_size=VALIDATION_SIZE / (TEST_SIZE + VALIDATION_SIZE),
        random_state=42,
    )

    train_dir = Path(f"{output_path}/train")
    val_dir = Path(f"{output_path}/val")
    test_dir = Path(f"{output_path}/test")

    if not overwrite:
        if (
            train_dir.is_dir()
            and not next(train_dir.iterdir(), None)
            and val_dir.is_dir()
            and not next(train_dir.iterdir(), None)
            and test_dir.is_dir()
            and not next(test_dir.iterdir(), None)
        ):
            raise OSError(
                f"At least one of {test_dir, val_dir, train_dir} already exists and is not empty.\
                    Overwrite setting are set to {overwrite}"
            )
    else:
        if train_dir.is_dir():
            shutil.rmtree(train_dir)
        if val_dir.is_dir():
            shutil.rmtree(val_dir)
        if test_dir.is_dir():
            shutil.rmtree(test_dir)

    train_img_dir = Path(f"{train_dir}/images")
    train_annots_dir = Path(f"{train_dir}/labels")
    val_img_dir = Path(f"{val_dir}/images")
    val_annots_dir = Path(f"{val_dir}/labels")
    test_img_dir = Path(f"{test_dir}/images")
    test_annots_dir = Path(f"{test_dir}/labels")

    out_dirs = [
        train_img_dir,
        train_annots_dir,
        val_img_dir,
        val_annots_dir,
        test_img_dir,
        test_annots_dir,
    ]

    for dir in out_dirs:
        dir.mkdir(exist_ok=True, parents=True)

    copy_files_to_folder(train_image_paths, train_img_dir)
    copy_files_to_folder(train_annot_paths, train_annots_dir)
    copy_files_to_folder(val_image_paths, val_img_dir)
    copy_files_to_folder(val_annot_paths, val_annots_dir)
    copy_files_to_folder(test_image_paths, test_img_dir)
    copy_files_to_folder(test_annot_paths, test_annots_dir)

    return (train_dir, val_dir, test_dir)


if __name__ == "__main__":
    #  Define paths
    ORIGINAL_DATA_PATH = "../data/original"
    OUTPUT_PATH = "../data/output"
    EPOCHS = 10
    TEST_SIZE = 0.1
    VALIDATION_SIZE = 0.1
    TRAIN_SIZE = 0.8
    OVERWRITE = True

    assert TEST_SIZE + VALIDATION_SIZE + TRAIN_SIZE == pytest.approx(1)

    train_dir, val_dir, test_dir = create_train_val_test(
        ORIGINAL_DATA_PATH, ORIGINAL_DATA_PATH, overwrite=OVERWRITE
    )

    # Transforms for data augmentation
    transform_list = [
        ChangeColorTemperature(),
        ColorJitter(brightness=0.4, contrast=0.4, saturation=0.3, hue=0.07),
        RandomScale(scale=0.1),
        RandomRotate(angle=10),
        RandomShear(shear_factor=0.2),
        RandomTranslate(translate=0.1),
        RandomHorizontalFlip(p=0.5),
    ]
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
    train_ds = AnnotatedDataset(
        original_path=train_dir,
        image_dir_name="images",
        annot_dir_name="labels",
        transform=transforms,
    )
    val_ds = AnnotatedDataset(
        original_path=val_dir,
        image_dir_name="images",
        annot_dir_name="labels",
        transform=None,
    )
    test_ds = AnnotatedDataset(
        original_path=test_dir,
        image_dir_name="images",
        annot_dir_name="labels",
        transform=None,
    )

    all_ds_dict = {"train": train_ds, "val": val_ds, "test": test_ds}

    # First iterate through all output_dirs to see if dirs are already existing and non-empty
    # In the case that OVERWRITE is set to False, this will ensure that we don't write to any existing directories
    # by having a check-loop first
    for ds_name in all_ds_dict:
        original_ds = all_ds_dict[ds_name]

        image_output_dir = Path(f"{OUTPUT_PATH}/{ds_name}/images/")
        label_output_dir = Path(f"{OUTPUT_PATH}/{ds_name}/labels/")
        if not OVERWRITE:
            if (
                image_output_dir.is_dir()
                and not next(image_output_dir.iterdir(), None)
                and label_output_dir.is_dir()
                and not next(label_output_dir.iterdir(), None)
            ):
                raise OSError(
                    f"At least one of {image_output_dir} and {label_output_dir} already exists and is not empty.\
                        Overwrite settings are set to {OVERWRITE}."
                )
        else:
            if image_output_dir.is_dir():
                shutil.rmtree(image_output_dir)
            if label_output_dir.is_dir():
                shutil.rmtree(label_output_dir)

    for ds_name in all_ds_dict:
        original_ds = all_ds_dict[ds_name]

        image_output_dir = Path(f"{OUTPUT_PATH}/{ds_name}/images/")
        label_output_dir = Path(f"{OUTPUT_PATH}/{ds_name}/labels/")

        image_output_dir.mkdir(exist_ok=True, parents=True)
        label_output_dir.mkdir(exist_ok=True, parents=True)
        # Print table to convert from new file idx to old file names
        print(
            "\n".join(
                [
                    f"{idx} \t {path.stem}"
                    for idx, path in enumerate(original_ds.annot_paths)
                ]
            ),
            file=open(f"{OUTPUT_PATH}/{ds_name}/references.txt", "w"),
        )

        for epoch in tqdm(range(EPOCHS), desc="Epoch No."):
            for i, sample in tqdm(
                enumerate(original_ds),
                desc="Sample No.",
                leave=False,
                total=len(original_ds),
            ):
                file_stem = f"{epoch}_{i}"

                # Write image
                sample["image"].save(image_output_dir / f"{file_stem}.png")
                # Write label
                label_strings = [bbox.convert_yolo() for bbox in sample["labels"]]
                print(
                    "\n".join(label_strings),
                    file=open(label_output_dir / f"{file_stem}.txt", "w"),
                )
