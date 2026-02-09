import airsim
import math
import numpy as np
import cv2

objects_dict = {
    "turbine1": "BP_Wind_Turbines_C_1",
    "turbine2": "StaticMeshActor_2",
    "solarpanels": "StaticMeshActor_146",
    "crowd": "StaticMeshActor_6",
    "car": "StaticMeshActor_10",
    "tower1": "SM_Electric_trellis_179",
    "tower2": "SM_Electric_trellis_7",
    "tower3": "SM_Electric_trellis_8",
}


class AirSimWrapper:
    def __init__(self):
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)

    def takeoff(self):
        self.client.takeoffAsync().join()

    def land(self):
        self.client.landAsync().join()

    def get_drone_position(self):
        pose = self.client.simGetVehiclePose()
        return [pose.position.x_val, pose.position.y_val, pose.position.z_val]

    def fly_to(self, point):
        if point[2] > 0:
            self.client.moveToPositionAsync(point[0], point[1], -point[2], 5).join()
        else:
            self.client.moveToPositionAsync(point[0], point[1], point[2], 5).join()

    def fly_path(self, points):
        airsim_points = []
        for point in points:
            if point[2] > 0:
                airsim_points.append(airsim.Vector3r(point[0], point[1], -point[2]))
            else:
                airsim_points.append(airsim.Vector3r(point[0], point[1], point[2]))
        self.client.moveOnPathAsync(airsim_points, 5, 120, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False, 0), 20, 1).join()

    def set_yaw(self, yaw):
        self.client.rotateToYawAsync(yaw, 5).join()

    def get_yaw(self):
        orientation_quat = self.client.simGetVehiclePose().orientation
        yaw = airsim.to_eularian_angles(orientation_quat)[2]
        return yaw

    def get_position(self, object_name):
        query_string = objects_dict[object_name] + ".*"
        object_names_ue = []
        while len(object_names_ue) == 0:
            object_names_ue = self.client.simListSceneObjects(query_string)
        pose = self.client.simGetObjectPose(object_names_ue[0])
        return [pose.position.x_val, pose.position.y_val, pose.position.z_val]

    def get_image(self, camera_name="front_center", image_type="scene"):
        """Get camera image, returns numpy array (BGR format)"""
        image_type_map = {
            "scene": airsim.ImageType.Scene,
            "depth": airsim.ImageType.DepthPerspective,
            "segmentation": airsim.ImageType.Segmentation,
            "infrared": airsim.ImageType.Infrared,
        }
        img_type = image_type_map.get(image_type, airsim.ImageType.Scene)

        responses = self.client.simGetImages([
            airsim.ImageRequest(camera_name, img_type, False, False)
        ])

        if responses and len(responses) > 0:
            response = responses[0]
            img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
            img_bgr = img1d.reshape(response.height, response.width, 3)
            return img_bgr
        return None

    def get_depth_image(self, camera_name="front_center"):
        """Get depth image, returns numpy array with depth values"""
        responses = self.client.simGetImages([
            airsim.ImageRequest(camera_name, airsim.ImageType.DepthPerspective, True, False)
        ])

        if responses and len(responses) > 0:
            response = responses[0]
            depth = airsim.list_to_2d_float_array(response.image_data_float, response.width, response.height)
            return depth
        return None

    def set_camera_orientation(self, camera_name, pitch, roll, yaw):
        """Set camera orientation (in degrees)"""
        pitch_rad = math.radians(pitch)
        roll_rad = math.radians(roll)
        yaw_rad = math.radians(yaw)

        # Get current camera pose to preserve position
        camera_info = self.client.simGetCameraInfo(camera_name)
        current_pose = camera_info.pose

        # Create new orientation
        orientation = airsim.to_quaternion(pitch_rad, roll_rad, yaw_rad)

        # Create new pose with same position but new orientation
        new_pose = airsim.Pose(current_pose.position, orientation)
        self.client.simSetCameraPose(camera_name, new_pose)

    def get_camera_orientation(self, camera_name="front_center"):
        """Get current camera orientation, returns [pitch, roll, yaw] in degrees"""
        camera_info = self.client.simGetCameraInfo(camera_name)
        orientation_quat = camera_info.pose.orientation
        pitch, roll, yaw = airsim.to_eularian_angles(orientation_quat)
        return [math.degrees(pitch), math.degrees(roll), math.degrees(yaw)]

    def save_image(self, filename, camera_name="front_center", image_type="scene"):
        """Save image to file"""
        img = self.get_image(camera_name, image_type)
        if img is not None:
            cv2.imwrite(filename, img)
            return True
        return False

