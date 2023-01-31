from __future__ import annotations

from pathlib import Path
import numpy as np
import torch
import glob
import re
from torch.utils.data import Dataset
from PIL import Image
import xml.etree.ElementTree as ET
import cv2
from typing import List, Tuple
import matplotlib.pyplot as plt

# Dictionary that maps class names to IDs
CLASS_TO_ID = {"Red Light": 0,
               "Blue Light": 1,
               "Blue Plate": 2,
               "Red Plate": 3}
ID_TO_CLASS = {v: k for k, v in CLASS_TO_ID.items()}


class BoundingBox:
    def __init__(self, class_name=None, class_id=None, xmin=None, ymin=None, xmax=None, ymax=None, center_x=None,
                 center_y=None, width=None,
                 height=None):
        if class_name is not None:
            self.class_name = class_name
            assert class_name in CLASS_TO_ID, "Invalid class name"
            self.class_id = CLASS_TO_ID[class_name]
        elif class_id is not None:
            self.class_id = class_id
            self.class_name = ID_TO_CLASS[class_id]

        self.is_corner_style = bool(xmin is not None and ymin is not None and xmax is not None and ymax is not None)
        self.is_yolo_style = bool(
            center_x is not None and center_y is not None and width is not None and height is not None)

        assert self.is_yolo_style or (xmax >= xmin and ymax >= ymin), "Invalid corner format"
        assert self.is_corner_style or (width >= 0 and height >= 0), "Invalid yolo style format"

        if self.is_corner_style:
            self.xmin = xmin
            self.ymin = ymin
            self.xmax = xmax
            self.ymax = ymax
            self.center_x = (xmin + xmax) / 2
            self.center_y = (ymin + ymax) / 2
            self.width = xmax - xmin
            self.height = ymax - ymin
        elif self.is_yolo_style:
            self.center_x = center_x
            self.center_y = center_y
            self.width = width
            self.height = height
            self.xmin = center_x - width / 2
            self.ymin = center_y - height / 2
            self.xmax = center_x + width / 2
            self.ymax = center_y + height / 2
        else:
            raise Exception("Neither corner format nor yolo style format was provided")

    def convert_yolo(self) -> Tuple[float, float, float, float]:
        """
        Returns (center_x, center_y, width, height)
        """
        return self.center_x, self.center_y, self.width, self.height

    def convert_corner(self) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        """
        Returns ((xmin, ymin), (xmax, ymax))
        """
        return (self.xmin, self.ymin), (self.xmax, self.ymax)

    def convert_corner_numpy(self) -> np.ndarray:
        """
        Returns np.array([xmin, ymin, xmax, ymax, class_id])
        """
        return np.array(list((self.xmin, self.ymin, self.xmax, self.ymax, self.class_id)))

    def swap_color(self) -> None:
        """
        Swap labels from red to blue or vice versa
        """
        if self.class_name == "Red Light":
            self.class_name = "Blue Light"
        elif self.class_name == "Red Plate":
            self.class_name = "Blue Plate"
        elif self.class_name == "Blue Light":
            self.class_name = "Red Light"
        elif self.class_name == "Blue Plate":
            self.class_name = "Red Plate"
        else:
            raise Exception(f"Unhandled class name: {self.class_name}")
        self.class_id = CLASS_TO_ID[self.class_name]

    @staticmethod
    def bboxes_to_numpy(bboxes: List[BoundingBox]):
        return np.stack([bbox.convert_corner_numpy() for bbox in bboxes], axis=0)

    @staticmethod
    def numpy_to_bboxes(bboxes_np: np.ndarray):
        bboxes = []
        for bbox_np in bboxes_np:
            bboxes.append(BoundingBox(class_id=bbox_np[4], xmin=bbox_np[0], ymin=bbox_np[1], xmax=bbox_np[2],
                                      ymax=bbox_np[3]))
        return bboxes


def get_elements_by_tag(xml_path: Path, parent: ET.Element, tag: str, bound_number: int, bound: str = None) -> List[
    ET.Element]:
    try:
        elements = parent.findall(tag)
        if bound_number:
            if bound == ">":
                assert len(elements) > bound_number
            elif bound == "==":
                assert len(elements) == bound_number
            elif bound == "lt":
                assert len(elements) < bound_number
            elif bound == ">=":
                assert len(elements) >= bound_number
            elif bound == "<=":
                assert len(elements) <= bound_number
            elif bound == "!=":
                assert len(elements) != bound_number
            elif bound != None:
                print(f"{bound} is not valid. Choose from one of the following ['>', '==', '>=', '<=', '!=']")
    except:
        raise Exception(f"{xml_path} has more or less than one <{tag}> tags from {parent}. Skipping....")
    return elements


def get_bbox(xml_filename: Path, bounding_boxes: ET.Element, class_name: str) -> BoundingBox:
    xmin = get_elements_by_tag(xml_filename, bounding_boxes, "xmin", 1, "==")
    ymin = get_elements_by_tag(xml_filename, bounding_boxes, "ymin", 1, "==")
    xmax = get_elements_by_tag(xml_filename, bounding_boxes, "xmax", 1, "==")
    ymax = get_elements_by_tag(xml_filename, bounding_boxes, "ymax", 1, "==")
    if xmin == None or ymin == None or xmax == None or ymax == None:
        raise Exception("Missing corner of bounding box")
    xmin = int(xmin[0].text)
    ymin = int(ymin[0].text)
    xmax = int(xmax[0].text)
    ymax = int(ymax[0].text)
    return BoundingBox(class_name=class_name, xmin=xmin, ymin=ymin, xmax=xmax, ymax=ymax)


def draw_bboxes_on_image(image: Image, labels: List[BoundingBox]):
    """
    Returns np.ndarray in format ready for plt.imshow() (Different format than cv2)
    """
    BGR_color_dict = {"Red Light": (0, 0, 255), "Blue Light": (255, 0, 0), "Blue Plate": (255, 238, 131),
                      "Red Plate": (42, 76, 234)}

    image = np.array(image)
    image = image[:, :, ::-1].copy()  # Convert RGB to BGR for cv2

    for bbox in labels:
        bbox_coords = bbox.convert_corner()
        cv2.rectangle(image, bbox_coords[0], bbox_coords[1], BGR_color_dict[bbox.class_name], 3)
    return image[:, :, ::-1]


class AnnotatedDataset(Dataset):
    def __init__(self, original_path, transform=None):
        self.original_path = original_path
        self.transform = transform
        image_paths_raw = [Path(x) for x in glob.glob(f'{original_path}/images/*.png')]  # List used for order of paths
        annot_paths_raw = [Path(x) for x in glob.glob(f'{original_path}/annotations/*.xml')]
        self.SAMPLE_FRAME_PATTERN = re.compile("[0-9]+_frame_[0-9]+")

        # Clean image paths
        self.image_paths = []
        self.annot_paths = []

        # Check if annotation in valid
        annot_paths_valid = []
        for annot_path in annot_paths_raw:
            try:
                tree = ET.parse(annot_path)
                root = tree.getroot()
                xml_filename = get_elements_by_tag(annot_path, root, "filename", 1, "==")[0]
                objects = get_elements_by_tag(annot_path, root, "object", 0, ">")
                for object in objects:
                    name = get_elements_by_tag(annot_path, object, "name", 1, "==")[0].text
                    bounding_box_xml = get_elements_by_tag(annot_path, object, "bndbox", 1, "==")[0]
                    bbox = get_bbox(annot_path, bounding_box_xml, name)
            except Exception as e:
                print(f"Error parsing XML for annotation {annot_path}")
                continue
            annot_paths_valid.append(annot_path)

        # Check if image has corresponding annotation
        valid_annots_set = set([x.stem for x in annot_paths_valid])
        for image_path in image_paths_raw:
            if image_path.stem in valid_annots_set:
                self.image_paths.append(image_path)
            else:
                print(f"Annotation not found for image {image_path}")

        # Check if annotation has corresponding image
        matched_images_set = set([x.stem for x in self.image_paths])
        for annot_path in annot_paths_raw:
            if annot_path.stem in matched_images_set:
                self.annot_paths.append(annot_path)
            else:
                print(f"Image not found for annotation {annot_path}")

        self.image_path_dict = {x.stem: x for x in self.image_paths}
        self.annot_path_dict = {x.stem: x for x in self.annot_paths}
        assert (len(self.image_paths) == len(self.annot_paths)), "Image and Annotation count mismatch after cleaning"

    def __len__(self):
        return len(self.image_paths)

    def __getitem__(self, idx):
        # Get annotations and image paths
        annot_path = self.annot_paths[idx]
        assert annot_path.stem in self.image_path_dict, f"{annot_path.stem}.xml's corresponding image could not be found"
        image_path = self.image_path_dict[annot_path.stem]

        # Process image
        image = Image.open(image_path).convert('RGB')  # Deal with RGBA

        # Process annotation
        labels = []
        tree = ET.parse(annot_path)
        root = tree.getroot()
        xml_filename = get_elements_by_tag(annot_path, root, "filename", 1, "==")[0]
        objects = get_elements_by_tag(annot_path, root, "object", 0, ">")
        for object in objects:
            name = get_elements_by_tag(annot_path, object, "name", 1, "==")[0].text
            bounding_box_xml = get_elements_by_tag(annot_path, object, "bndbox", 1, "==")[0]
            bbox = get_bbox(annot_path, bounding_box_xml, name)
            labels.append(bbox)
        labels = sorted(labels, key=lambda x: x.class_id)

        # Convert to sample
        sample = {'image': image, 'labels': labels}

        # Apply transforms
        if self.transform:
            sample = self.transform(sample)

        return sample


if __name__ == '__main__':

    ds = AnnotatedDataset(original_path='../data/original')
    for i, sample in enumerate(ds):
        if i > 5:
            break

        image = draw_bboxes_on_image(sample['image'], sample['labels'])

        fig, ax = plt.subplots()
        plt.imshow(image[:, :, ::-1])
        plt.show()
