import random
from typing import List

import cv2
import numpy as np
import torch
import torchvision.transforms as T
import torchvision.transforms.functional as TF
from matplotlib import pyplot as plt

from AnnotatedDataset import BoundingBox
from PIL import Image


def pil_to_cv2(image: Image):
    image = np.array(image)
    image = image[:, :, ::-1].copy()  # Convert RGB to BGR
    return image


def cv2_to_pil(image: np.ndarray):
    img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    im_pil = Image.fromarray(img)
    return im_pil


class Sequence:
    """Initialise Sequence object

    Apply a Sequence of transformations to the images/boxes.

    Parameters
    ----------
    augemnetations : list
        List containing Transformation Objects in Sequence they are to be
        applied

    probs : float or list
        If **int**, the probability with which each of the transformation will
        be applied. If **list**, the length must be equal to *augmentations*.
        Each element of this list is the probability with which each
        corresponding transformation is applied

    Returns
    -------

    Sequence
        Sequence Object

    """

    def __init__(self, transforms, probs=1):
        self.transforms = transforms
        self.probs = probs

    def __call__(self, samples):
        transformed_samples = []
        for sample in samples:
            for i, transform in enumerate(self.transforms):
                if type(self.probs) == list:
                    prob = self.probs[i]
                else:
                    prob = self.probs

                if random.random() < prob:
                    sample = transform(sample)
            transformed_samples.append(sample)
        return transformed_samples


class ChangeColorTemperature:
    def __init__(self):
        self.kelvin_table = {
            # 1000: (255, 56, 0),
            # 1500: (255, 109, 0),
            # 2000: (255, 137, 18),
            # 2500: (255, 161, 72),
            # 3000: (255, 180, 107),
            # 3500: (255, 196, 137),
            4000: (255, 209, 163),
            4500: (255, 219, 186),
            5000: (255, 228, 206),
            5500: (255, 236, 224),
            # 6000: (255, 243, 239),
            # 6500: (255, 249, 253),
            7000: (245, 243, 255),
            # 7500: (235, 238, 255),
            # 8000: (227, 233, 255),
            # 8500: (220, 229, 255),
            # 9000: (214, 225, 255),
            # 9500: (208, 222, 255),
            # 10000: (204, 219, 255),
            11000: (200, 213, 255),
            # 12000: (195, 209, 255),
            # 13000: (190, 206, 255),
            # 14000: (182, 206, 255),
            # 15000: (179, 204, 255),
        }
        self.kelvin_table_keys = list(self.kelvin_table.keys())

    def __call__(self, sample):
        kelvin = self.kelvin_table_keys[torch.randint(0, len(self.kelvin_table_keys), size=(1,))[0]]

        r, g, b = self.kelvin_table[kelvin]
        matrix = (r / 255.0, 0.0, 0.0, 0.0,
                  0.0, g / 255.0, 0.0, 0.0,
                  0.0, 0.0, b / 255.0, 0.0)
        image = sample['image'].convert('RGB', matrix)
        return {'image': image, 'labels': sample['labels'].copy()}


class ColorJitter:
    """
    See torchvision.transforms.ColorJitter
    """

    def __init__(self, brightness, contrast, saturation, hue):
        self._transform = T.ColorJitter(brightness=brightness, contrast=contrast, saturation=saturation,
                                        hue=hue)

    def __call__(self, sample):
        image = self._transform(sample['image'])
        return {'image': image, 'labels': sample['labels'].copy()}


class RandomColorSwap():
    """
    See torchvision.transforms.RandomHorizontalFlip
    p = probability of flip
    """

    def __init__(self, p=0.5):
        self.p = p

    def __call__(self, sample):
        if torch.rand(1) < self.p:
            image = sample['image']
            labels = sample['labels']

            # Convert bbox
            image = pil_to_cv2(image)
            image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
            blue_part = np.array(90 < image[:, :, 0]) & np.array(image[:, :, 0] < 140)
            red_part = (np.array(image[:, :, 0] < 15) | np.array(image[:, :, 0] > 165))
            neither_part = ~blue_part & ~red_part

            reconstructed_image_H = ((image[:, :, 0] + 60) % 180) * blue_part + (((image[:, :, 0] + 120) % 180)) * red_part + image[:, :, 0] * neither_part
            reconstructed_image = np.copy(image)
            reconstructed_image[:, :, 0] = reconstructed_image_H

            image = cv2.cvtColor(reconstructed_image, cv2.COLOR_HSV2BGR)
            image = cv2_to_pil(image)

            for label in labels:
                label.swap_color()

        else:
            image = sample['image']
            labels = sample['labels']

        return {'image': image, 'labels': labels}

#  Bounding box calculations below are taken from
#  https://github.com/Paperspace/DataAugmentationForObjectDetection/blob/master/data_aug/data_aug.py

class RandomHorizontalFlip:
    """
    See torchvision.transforms.RandomHorizontalFlip
    p = probability of flip
    """

    def __init__(self, p=0.5):
        self.p = p

    def __call__(self, sample):
        if torch.rand(1) < self.p:
            image = sample['image']

            # Convert bbox
            image = pil_to_cv2(image)
            image = image[:, ::-1, :]
            img_center = np.array(image.shape[:2])[::-1] / 2
            img_center = np.hstack((img_center, img_center))

            bboxes = BoundingBox.bboxes_to_numpy(sample['labels'])
            bboxes[:, [0, 2]] = (bboxes[:, [0, 2]].astype(np.float) + 2 * (img_center[[0, 2]] - bboxes[:, [0, 2]])).astype(np.int)
            box_w = abs(bboxes[:, 0] - bboxes[:, 2])
            bboxes[:, 0] -= box_w
            bboxes[:, 2] += box_w
            labels = BoundingBox.numpy_to_bboxes(bboxes)
            image = cv2_to_pil(image)
        else:
            image = sample['image']
            labels = sample['labels']

        return {'image': image, 'labels': labels}


class RandomScale(object):
    """Randomly scales an image


    Bounding boxes which have an area of less than 25% in the remaining in the
    transformed image is dropped. The resolution is maintained, and the remaining
    area if any is filled by black color.

    Parameters
    ----------
    scale:
        if **float**, the image is scaled by a factor drawn
        randomly from a range (1 - `scale` , 1 + `scale`).

    Returns
    -------

    numpy.ndaaray
        Scaled image in the numpy format of shape `HxWxC`

    numpy.ndarray
        Tranformed bounding box co-ordinates of the format `n x 4` where n is
        number of bounding boxes and 4 represents `x1,y1,x2,y2` of the box

    """

    def __init__(self, scale=0.2):
        assert scale > 0, "Please input a positive float"
        self.scale = (max(-1.0, -scale), scale)

    def __call__(self, sample):
        img = pil_to_cv2(sample['image'])
        bboxes = BoundingBox.bboxes_to_numpy(sample['labels'])

        img_shape = img.shape
        actual_scale = random.uniform(*self.scale)
        resize_scale = 1 + actual_scale

        img = cv2.resize(img, None, fx=resize_scale, fy=resize_scale)
        bboxes[:, :4] = (bboxes[:, :4].astype(np.float) * [resize_scale, resize_scale, resize_scale, resize_scale]).astype(np.int)

        canvas = np.zeros(img_shape, dtype=np.uint8)

        y_lim = int(min(resize_scale, 1) * img_shape[0])
        x_lim = int(min(resize_scale, 1) * img_shape[1])

        canvas[:y_lim, :x_lim, :] = img[:y_lim, :x_lim, :]

        img = canvas
        bboxes = clip_box(bboxes, [0, 0, 1 + img_shape[1], img_shape[0]], 0.25)

        return {'image': cv2_to_pil(img), 'labels': BoundingBox.numpy_to_bboxes(bboxes)}

class RandomTranslate(object):
    """Randomly Translates the image


    Bounding boxes which have an area of less than 25% in the remaining in the
    transformed image is dropped. The resolution is maintained, and the remaining
    area if any is filled by black color.

    Parameters
    ----------
    translate: float or tuple(float)
        if **float**, the image is translated by a factor drawn
        randomly from a range (1 - `translate` , 1 + `translate`). If **tuple**,
        `translate` is drawn randomly from values specified by the
        tuple

    Returns
    -------

    numpy.ndaaray
        Translated image in the numpy format of shape `HxWxC`

    numpy.ndarray
        Tranformed bounding box co-ordinates of the format `n x 4` where n is
        number of bounding boxes and 4 represents `x1,y1,x2,y2` of the box

    """

    def __init__(self, translate=0.2, diff=False):
        self.translate = translate

        if type(self.translate) == tuple:
            assert len(self.translate) == 2, "Invalid range"
            assert self.translate[0] > 0 & self.translate[0] < 1
            assert self.translate[1] > 0 & self.translate[1] < 1


        else:
            assert self.translate > 0 and self.translate < 1
            self.translate = (-self.translate, self.translate)

        self.diff = diff

    def __call__(self, sample):
        img = pil_to_cv2(sample['image'])
        bboxes = BoundingBox.bboxes_to_numpy(sample['labels'])

        # Chose a random digit to scale by
        img_shape = img.shape

        # percentage of the dimension of the image to translate
        translate_factor_x = random.uniform(*self.translate)
        translate_factor_y = random.uniform(*self.translate)

        if not self.diff:
            translate_factor_y = translate_factor_x

        canvas = np.zeros(img_shape).astype(np.uint8)

        corner_x = int(translate_factor_x * img.shape[1])
        corner_y = int(translate_factor_y * img.shape[0])

        # change the origin to the top-left corner of the translated box
        orig_box_cords = [max(0, corner_y), max(corner_x, 0), min(img_shape[0], corner_y + img.shape[0]),
                          min(img_shape[1], corner_x + img.shape[1])]

        mask = img[max(-corner_y, 0):min(img.shape[0], -corner_y + img_shape[0]),
               max(-corner_x, 0):min(img.shape[1], -corner_x + img_shape[1]), :]
        canvas[orig_box_cords[0]:orig_box_cords[2], orig_box_cords[1]:orig_box_cords[3], :] = mask
        img = canvas

        bboxes[:, :4] += [corner_x, corner_y, corner_x, corner_y]

        bboxes = clip_box(bboxes, [0, 0, img_shape[1], img_shape[0]], 0.25)

        return {'image': cv2_to_pil(img), 'labels': BoundingBox.numpy_to_bboxes(bboxes)}


class RandomRotate(object):
    """Randomly rotates an image


    Bounding boxes which have an area of less than 25% in the remaining in the
    transformed image is dropped. The resolution is maintained, and the remaining
    area if any is filled by black color.

    Parameters
    ----------
    angle: float or tuple(float)
        if **float**, the image is rotated by a factor drawn
        randomly from a range (-`angle`, `angle`). If **tuple**,
        the `angle` is drawn randomly from values specified by the
        tuple

    Returns
    -------

    numpy.ndaaray
        Rotated image in the numpy format of shape `HxWxC`

    numpy.ndarray
        Tranformed bounding box co-ordinates of the format `n x 4` where n is
        number of bounding boxes and 4 represents `x1,y1,x2,y2` of the box

    """

    def __init__(self, angle=10):
        self.angle = angle

        if type(self.angle) == tuple:
            assert len(self.angle) == 2, "Invalid range"

        else:
            self.angle = (-self.angle, self.angle)

    def __call__(self, sample):
        img = pil_to_cv2(sample['image'])
        bboxes = BoundingBox.bboxes_to_numpy(sample['labels'])

        angle = random.uniform(*self.angle)

        w, h = img.shape[1], img.shape[0]
        cx, cy = w // 2, h // 2

        img = rotate_im(img, angle)

        corners = get_corners(bboxes)

        corners = np.hstack((corners, bboxes[:, 4:]))

        corners[:, :8] = rotate_box(corners[:, :8], angle, cx, cy, h, w)

        new_bbox = get_enclosing_box(corners)

        scale_factor_x = img.shape[1] / w

        scale_factor_y = img.shape[0] / h

        img = cv2.resize(img, (w, h))

        new_bbox[:, :4] = (new_bbox[:, :4].astype(np.float) / [scale_factor_x, scale_factor_y, scale_factor_x, scale_factor_y]).astype(np.int)

        bboxes = new_bbox

        bboxes = clip_box(bboxes, [0, 0, w, h], 0.25)

        return {'image': cv2_to_pil(img), 'labels': BoundingBox.numpy_to_bboxes(bboxes)}


class RandomShear(object):
    """Randomly shears an image in horizontal direction


    Bounding boxes which have an area of less than 25% in the remaining in the
    transformed image is dropped. The resolution is maintained, and the remaining
    area if any is filled by black color.

    Parameters
    ----------
    shear_factor: float or tuple(float)
        if **float**, the image is sheared horizontally by a factor drawn
        randomly from a range (-`shear_factor`, `shear_factor`). If **tuple**,
        the `shear_factor` is drawn randomly from values specified by the
        tuple

    Returns
    -------

    numpy.ndaaray
        Sheared image in the numpy format of shape `HxWxC`

    numpy.ndarray
        Tranformed bounding box co-ordinates of the format `n x 4` where n is
        number of bounding boxes and 4 represents `x1,y1,x2,y2` of the box

    """

    def __init__(self, shear_factor=0.2):
        self.shear_factor = shear_factor

        if type(self.shear_factor) == tuple:
            assert len(self.shear_factor) == 2, "Invalid range for scaling factor"
        else:
            self.shear_factor = (-self.shear_factor, self.shear_factor)

        shear_factor = random.uniform(*self.shear_factor)

    def __call__(self, sample):
        img = pil_to_cv2(sample['image'])
        bboxes = BoundingBox.bboxes_to_numpy(sample['labels'])

        shear_factor = random.uniform(*self.shear_factor)

        w, h = img.shape[1], img.shape[0]

        if shear_factor < 0:
            img, bboxes = _horizontal_flip(img, bboxes)

        M = np.array([[1, abs(shear_factor), 0], [0, 1, 0]])

        nW = img.shape[1] + abs(shear_factor * img.shape[0])

        bboxes[:, [0, 2]] += ((bboxes[:, [1, 3]]) * abs(shear_factor)).astype(int)

        img = cv2.warpAffine(img, M, (int(nW), img.shape[0]))

        if shear_factor < 0:
            img, bboxes = _horizontal_flip(img, bboxes)

        img = cv2.resize(img, (w, h))

        scale_factor_x = nW / w

        bboxes[:, :4] = (bboxes[:, :4].astype(np.float) / [scale_factor_x, 1, scale_factor_x, 1]).astype(np.int)

        return {'image': cv2_to_pil(img), 'labels': BoundingBox.numpy_to_bboxes(bboxes)}


def _horizontal_flip(img, bboxes):
    """
    For use internal to ImageTransforms.py only
    Takes in cv2 image and numpy bboxes
    """
    img_center = np.array(img.shape[:2])[::-1] / 2
    img_center = np.hstack((img_center, img_center))

    img = img[:, ::-1, :]
    bboxes[:, [0, 2]] = (bboxes[:, [0, 2]].astype(np.float) + 2 * (img_center[[0, 2]] - bboxes[:, [0, 2]])).astype(np.int)

    box_w = abs(bboxes[:, 0] - bboxes[:, 2])

    bboxes[:, 0] -= box_w
    bboxes[:, 2] += box_w

    return img, bboxes


###################
# bbox transforms #
###################
# From https://github.com/Paperspace/DataAugmentationForObjectDetection/blob/master/data_aug/bbox_util.py


def draw_rect(im, cords, color=None):
    """Draw the rectangle on the image

    Parameters
    ----------

    im : numpy.ndarray
        numpy image

    cords: numpy.ndarray
        Numpy array containing bounding boxes of shape `N X 4` where N is the
        number of bounding boxes and the bounding boxes are represented in the
        format `x1 y1 x2 y2`

    Returns
    -------

    numpy.ndarray
        numpy image with bounding boxes drawn on it

    """

    im = im.copy()

    cords = cords[:, :4]
    cords = cords.reshape(-1, 4)
    if not color:
        color = [255, 255, 255]
    for cord in cords:
        pt1, pt2 = (cord[0], cord[1]), (cord[2], cord[3])

        pt1 = int(pt1[0]), int(pt1[1])
        pt2 = int(pt2[0]), int(pt2[1])

        im = cv2.rectangle(im.copy(), pt1, pt2, color, int(max(im.shape[:2]) / 200))
    return im


def bbox_area(bbox):
    return (bbox[:, 2] - bbox[:, 0]) * (bbox[:, 3] - bbox[:, 1])


def clip_box(bbox, clip_box, alpha):
    """Clip the bounding boxes to the borders of an image

    Parameters
    ----------

    bbox: numpy.ndarray
        Numpy array containing bounding boxes of shape `N X 4` where N is the
        number of bounding boxes and the bounding boxes are represented in the
        format `x1 y1 x2 y2`

    clip_box: numpy.ndarray
        An array of shape (4,) specifying the diagonal co-ordinates of the image
        The coordinates are represented in the format `x1 y1 x2 y2`

    alpha: float
        If the fraction of a bounding box left in the image after being clipped is
        less than `alpha` the bounding box is dropped.

    Returns
    -------

    numpy.ndarray
        Numpy array containing **clipped** bounding boxes of shape `N X 4` where N is the
        number of bounding boxes left are being clipped and the bounding boxes are represented in the
        format `x1 y1 x2 y2`

    """
    ar_ = (bbox_area(bbox))
    x_min = np.maximum(bbox[:, 0], clip_box[0]).reshape(-1, 1)
    y_min = np.maximum(bbox[:, 1], clip_box[1]).reshape(-1, 1)
    x_max = np.minimum(bbox[:, 2], clip_box[2]).reshape(-1, 1)
    y_max = np.minimum(bbox[:, 3], clip_box[3]).reshape(-1, 1)

    bbox = np.hstack((x_min, y_min, x_max, y_max, bbox[:, 4:]))

    delta_area = ((ar_ - bbox_area(bbox)) / ar_)

    mask = (delta_area < (1 - alpha)).astype(int)

    bbox = bbox[mask == 1, :]

    return bbox


def rotate_im(image, angle):
    """Rotate the image.

    Rotate the image such that the rotated image is enclosed inside the tightest
    rectangle. The area not occupied by the pixels of the original image is colored
    black.

    Parameters
    ----------

    image : numpy.ndarray
        numpy image

    angle : float
        angle by which the image is to be rotated

    Returns
    -------

    numpy.ndarray
        Rotated Image

    """
    # grab the dimensions of the image and then determine the
    # centre
    (h, w) = image.shape[:2]
    (cX, cY) = (w // 2, h // 2)

    # grab the rotation matrix (applying the negative of the
    # angle to rotate clockwise), then grab the sine and cosine
    # (i.e., the rotation components of the matrix)
    M = cv2.getRotationMatrix2D((cX, cY), angle, 1.0)
    cos = np.abs(M[0, 0])
    sin = np.abs(M[0, 1])

    # compute the new bounding dimensions of the image
    nW = int((h * sin) + (w * cos))
    nH = int((h * cos) + (w * sin))

    # adjust the rotation matrix to take into account translation
    M[0, 2] += (nW / 2) - cX
    M[1, 2] += (nH / 2) - cY

    # perform the actual rotation and return the image
    image = cv2.warpAffine(image, M, (nW, nH))

    #    image = cv2.resize(image, (w,h))
    return image


def get_corners(bboxes):
    """Get corners of bounding boxes

    Parameters
    ----------

    bboxes: numpy.ndarray
        Numpy array containing bounding boxes of shape `N X 4` where N is the
        number of bounding boxes and the bounding boxes are represented in the
        format `x1 y1 x2 y2`

    returns
    -------

    numpy.ndarray
        Numpy array of shape `N x 8` containing N bounding boxes each described by their
        corner co-ordinates `x1 y1 x2 y2 x3 y3 x4 y4`

    """
    width = (bboxes[:, 2] - bboxes[:, 0]).reshape(-1, 1)
    height = (bboxes[:, 3] - bboxes[:, 1]).reshape(-1, 1)

    x1 = bboxes[:, 0].reshape(-1, 1)
    y1 = bboxes[:, 1].reshape(-1, 1)

    x2 = x1 + width
    y2 = y1

    x3 = x1
    y3 = y1 + height

    x4 = bboxes[:, 2].reshape(-1, 1)
    y4 = bboxes[:, 3].reshape(-1, 1)

    corners = np.hstack((x1, y1, x2, y2, x3, y3, x4, y4))

    return corners


def rotate_box(corners, angle, cx, cy, h, w):
    """Rotate the bounding box.


    Parameters
    ----------

    corners : numpy.ndarray
        Numpy array of shape `N x 8` containing N bounding boxes each described by their
        corner co-ordinates `x1 y1 x2 y2 x3 y3 x4 y4`

    angle : float
        angle by which the image is to be rotated

    cx : int
        x coordinate of the center of image (about which the box will be rotated)

    cy : int
        y coordinate of the center of image (about which the box will be rotated)

    h : int
        height of the image

    w : int
        width of the image

    Returns
    -------

    numpy.ndarray
        Numpy array of shape `N x 8` containing N rotated bounding boxes each described by their
        corner co-ordinates `x1 y1 x2 y2 x3 y3 x4 y4`
    """

    corners = corners.reshape(-1, 2)
    corners = np.hstack((corners, np.ones((corners.shape[0], 1), dtype=type(corners[0][0]))))

    M = cv2.getRotationMatrix2D((cx, cy), angle, 1.0)

    cos = np.abs(M[0, 0])
    sin = np.abs(M[0, 1])

    nW = int((h * sin) + (w * cos))
    nH = int((h * cos) + (w * sin))
    # adjust the rotation matrix to take into account translation
    M[0, 2] += (nW / 2) - cx
    M[1, 2] += (nH / 2) - cy
    # Prepare the vector to be transformed
    calculated = np.dot(M, corners.T).T

    calculated = calculated.reshape(-1, 8)

    return calculated


def get_enclosing_box(corners):
    """Get an enclosing box for ratated corners of a bounding box

    Parameters
    ----------

    corners : numpy.ndarray
        Numpy array of shape `N x 8` containing N bounding boxes each described by their
        corner co-ordinates `x1 y1 x2 y2 x3 y3 x4 y4`

    Returns
    -------

    numpy.ndarray
        Numpy array containing enclosing bounding boxes of shape `N X 4` where N is the
        number of bounding boxes and the bounding boxes are represented in the
        format `x1 y1 x2 y2`

    """
    x_ = corners[:, [0, 2, 4, 6]]
    y_ = corners[:, [1, 3, 5, 7]]

    xmin = np.min(x_, 1).reshape(-1, 1)
    ymin = np.min(y_, 1).reshape(-1, 1)
    xmax = np.max(x_, 1).reshape(-1, 1)
    ymax = np.max(y_, 1).reshape(-1, 1)

    final = np.hstack((xmin, ymin, xmax, ymax, corners[:, 8:]))

    return final


def letterbox_image(img, inp_dim):
    '''resize image with unchanged aspect ratio using padding

    Parameters
    ----------

    img : numpy.ndarray
        Image

    inp_dim: tuple(int)
        shape of the reszied image

    Returns
    -------

    numpy.ndarray:
        Resized image

    '''

    inp_dim = (inp_dim, inp_dim)
    img_w, img_h = img.shape[1], img.shape[0]
    w, h = inp_dim
    new_w = int(img_w * min(w / img_w, h / img_h))
    new_h = int(img_h * min(w / img_w, h / img_h))
    resized_image = cv2.resize(img, (new_w, new_h))

    canvas = np.full((inp_dim[1], inp_dim[0], 3), 0)

    canvas[(h - new_h) // 2:(h - new_h) // 2 + new_h, (w - new_w) // 2:(w - new_w) // 2 + new_w, :] = resized_image

    return canvas