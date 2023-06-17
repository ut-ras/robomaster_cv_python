import json

import cv2
import numpy

from config.config import ConfigStore


class ConfigSource:
    def update(self, config_store: ConfigStore) -> None:
        raise NotImplementedError


class FileConfigSource(ConfigSource):
    CONFIG_FILENAME = "config.json"
    CALIBRATION_FILENAME = "calibration.json"

    def __init__(self) -> None:
        pass

    def update(self, config_store: ConfigStore) -> None:
        # Get config
        with open(self.CONFIG_FILENAME, "r") as config_file:
            config_data = json.loads(config_file.read())
            config_store.local_config.device_id = config_data["device_id"]
            config_store.local_config.server_ip = config_data["server_ip"]
            config_store.local_config.stream_port = config_data["stream_port"]

        # Get calibration
        calibration_store = cv2.FileStorage(self.CALIBRATION_FILENAME, cv2.FILE_STORAGE_READ)
        camera_matrix = calibration_store.getNode("camera_matrix").mat()
        distortion_coefficients = calibration_store.getNode("distortion_coefficients").mat()
        calibration_store.release()
        if type(camera_matrix) == numpy.ndarray and type(distortion_coefficients) == numpy.ndarray:
            config_store.local_config.camera_matrix = camera_matrix
            config_store.local_config.distortion_coefficients = distortion_coefficients
            config_store.local_config.has_calibration = True