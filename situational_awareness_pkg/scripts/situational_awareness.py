#!/usr/bin/env python

import json
import rospy
import light_tracking
import jetson.inference
from std_msgs.msg import String
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose
from mission_mgmt_pkg.msg import MissionManagement as MissionManagementMsg
from situational_awareness_pkg.msg import SituationalAwareness as SituationalAwarenessMsg
from autonomy_bus_pkg.msg import AutonomyBus as AutonomyBusMsg

class SituationalAwareness:
    """
        This class represents the situational awareness module specified in UMAA.

        'SituationalAwareness' maintains the world model for decision-making by 'MissionMgmt'.
    """

    _CAMERA_TOPIC_NAME = 'simple_robot/camera1/image_raw'
    _SITUATIONAL_AWARENESS_MSG = SituationalAwarenessMsg()

    def __init__(self):
        """
            Constructs an 'SituationalAwareness' object. In constructing a 'SituationalAwareness'
            object you are creating a ROS node which maintains the world model for decision-making 
            by 'MissionMgmt'.

            Fields
            +--------------------+
            pub_data: A ROS publisher which retrieves conglomerated data from 'autonomy_bus', parses that data
                    then publishing parsed data back to 'autonomy_bus'
            pub_logger: A ROS publisher which broadcasts logging info about 'SituationalAwareness' object
            data: A dictionary data structure that stores parsed info from conglomerated data 'autonomy_bus' sends out
                    (key: module_name, value: module_name_message_type)
            data_logger: A dictionary data structure that stores incoming logging info from 'autonomy_bus'
                    (key: module_name, value: module_published_logging_data)
            _CAMERA_TOPIC_NAME: The topic name for the images the camera is publishing too
            _SITUATIONAL_AWARENESS_MSG: An 'SituationalAwarenessMsg' that belongs to 'SituationalAwareness' class 
                    for updating and publishing

            How Things Work:
            +--------------------+
            1. 'SituationalAwareness' listens to only 'AutonomyBus'
            2. The data from 'AutonomyBus' is a big blob of data comprised of all other module's/package's
                    information
            3. The blob of data is parsed for the data needed for 'SituationalAwareness'
            4. 'data' field is a dictionary of (key: module_name, value: module_name_message_type)
            5. 'data' should at least store an 'autonomy_bus', 'situational_awareness', and other module names
                    pertinent to 'situational_awareness' as keys
                - e.g., 'data = {'autonomy_bus': AutonomyBusMsg, 'situational_awareness': SituationalAwarenessMsg,
                'support_ops': SupportOperationsMsg}', in this case 'SituationalAwareness' 
                relies on data from 'support_ops_pkg'
        """
        self._init_models()
        # Create a ROS node named, "situational_awareness"
        rospy.init_node('situational_awareness', anonymous=True)
        # Setup publisher of data
        self.pub_data = rospy.Publisher('situational_awareness_out', SituationalAwarenessMsg, queue_size=10)
        # Setup publisher of logging info
        self.pub_logger = rospy.Publisher('situational_awareness_logger_out', String, queue_size=10)
        # Setup data structure for actual data
        self.data = {}
        # Setup data structure for logging
        self.data_logger = {}
        # Setup modules/packages to listen to (for data info)
        self._listen_data()
        # Setup modules/packages to listen to (for logging info)
        #self._listen_logger()
        # Start publishing parsed data from 'autonomy_bus'
        self._publish()

    def _init_models(self):
        """
            Initializes AI/ML models used in 'situational_awareness_pkg' such as light tracking. This is less modular
            as we are loading the models on start of ROS node but is faster than having 'light_tracking.py' have to
            load a model everytime an image comes in (a Jetson has a hard time with this as well as other packages
            running simultaneously)
        """
        # Import exported custom object detection model
        args = [
            '--model=/home/alex/krill_ws/src/situational_awareness_pkg/models/light_tracking_model/ssd-mobilenet.onnx',
            '--labels=/home/alex/krill_ws/src/situational_awareness_pkg/models/light_tracking_model/labels.txt',
            '--input-blob=input_0',
            '--output-cvg=scores',
            '--output-bbox=boxes'
        ]
        self.light_tracking_model = jetson.inference.detectNet(argv=args, threshold=0.5)

    def _publish(self):
        """
            Publishes incoming messages from 'autonomy_bus_pkg' in a uniform format.
        """
        # Initialize the rate at which messages
        rate = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            # Store 'SituationalAwarenessMsg' in 'data'
            self.data['situational_awareness'] = self._SITUATIONAL_AWARENESS_MSG
            # Update timestamp of 'SituationalAwarenessMsg' before every publication
            self._SITUATIONAL_AWARENESS_MSG.header.stamp = rospy.Time.now()
            # Create logger message
            logger_msg = '[{0}] "situational_awareness" node is okay!'.format(rospy.Time.now())
            # Publish "SituationalAwarenessMsg" to "situational_awareness_out" topic
            self.pub_data.publish(self._SITUATIONAL_AWARENESS_MSG)
            # Publish "logger_msg" to "support_ops_logger_out" topic
            #self.pub_logger.publish(logger_msg)
            # Sleep for 2 second
            rate.sleep()
        print('"situational_awareness" node has gone offline.')
            
    def _listen_data(self):
        """
            Setup 'SituationalAwareness' to listen other module's/package's actual data info
        """
        # Setup this node's subscribers in regards to actual data
        rospy.Subscriber('autonomy_bus_out', AutonomyBusMsg, self._autonomy_bus_callback)
        rospy.Subscriber(self._CAMERA_TOPIC_NAME, Image, self._image_callback)

    def _listen_logger(self):
        """
            Setup the listener to listen 'autonomy_bus_pkg' module's/package's logging info
        """
        # Setup this node's subscribers in regards to logging info
        rospy.Subscriber('autonomy_bus_logger_out', String, self._autonomy_bus_logger_callback)

    def _autonomy_bus_callback(self, data):
        """
            Callback method for the 'autonomy_bus_out' topic. It takes the data from the specified
            topic and parses it for data relevant to this module

            Parameters
            +--------------------+
            data: 'AutonomyBus' message type which is a message type that holds conglomerated data from
                    all modules involved in UUV
        """
        # Check if there is an 'autonomy_bus' key within 'data' field
        if ('autonomy_bus' in self.data):
            # Check if data sent from 'autonomy_bus_pkg' is different from previously sent data
            if (data.header.stamp != self.data['autonomy_bus'].header.stamp):
                # Update 'data' dict's 'autonomy_bus' key with new 'AutonomyBusMsg'
                self.data['autonomy_bus'] = data
        else:
            # Update 'data' dict's 'autonomy_bus' key with new 'AutonomyBusMsg'
            self.data['autonomy_bus'] = data
        # Parse "AutonomyBusMsg" for "MissionManagementMsg".
        self._parse_mission_mgmt()

    def _image_callback(self, data):
        """
            Callback method for the camera mounted on the Gazebo robot. This class/node subscribes to images
            being broadcasted and processing is done here. This method is called everytime a image is published
            from Gazebo robot.

            Parameters
            +--------------------+
            data: 'Image' message type containing image info taken from the robot within Gazebo
        """
        # Populate and update 'image' data in 'SituationalAwarenessMsg'
        self._SITUATIONAL_AWARENESS_MSG.image = data

        # Try grabbing 'mission_mgmt' (key) value from 'data'
        if ('mission_mgmt' in self.data):
            mission_mgmt_msg = self.data['mission_mgmt']
        else:
            return

        # Check if we need to execute a mission
        if (mission_mgmt_msg.execute_mission):
            # Check what mission to execute
            if (mission_mgmt_msg.mission == 'light tracking'):
                # Get results from 'light_tracking' module
                results = light_tracking.get_result(data, self.light_tracking_model)
                print('"results": {0}'.format(results))
                # Parse results update/populate 'SituationalAwarenessMsg' for 'maneuver_ops'.
                # 'maneuver_ops' will use the results to determine UUV movements
                if (results != None):
                    # Create detection message
                    det_msg = Detection2D()
                    det_msg.header.stamp = rospy.Time.now()
                    det_msg.bbox.size_x = results['width']
                    det_msg.bbox.size_y = results['height']
                    det_msg.bbox.center.x = results['center_x']
                    det_msg.bbox.center.y = results['center_y']
                    # Create classification hypothesis message
                    hyp = ObjectHypothesisWithPose()
                    hyp.id = results['class_id']
                    hyp.score = results['confidence']
                    det_msg.results.append(hyp)
                    # Store in our custom message
                    self._SITUATIONAL_AWARENESS_MSG.detection_msg = det_msg

    def _parse_mission_mgmt(self):
        """
            Parses the 'AutonomyBusMsg' message data to retrieve the data pertaining to 'mission_mgmt'
            ROS node. 
            
            May be blank data if 'situational_awareness' and 'autonomy_bus' node are online 
            before the 'mission_mgmt' node. Otherwise 'mission_mgmt' node should be publishing data to
            the 'autonomy_bus' node and then the 'situational_awareness' node is capturing 'mission_mgmt'
            data through the 'autonomy_bus'
        """
        # Grab 'MissionManagementMsg' from 'AutonomyBusMsg'
        mission_mgmt_msg = self.data['autonomy_bus'].mission_mgmt
        # Store the above 'MissionManagementMsg' into the 'data' field
        self.data['mission_mgmt'] = mission_mgmt_msg
        # For debugging
        rospy.loginfo('Mission: {0} || Execute? {1}'.format(mission_mgmt_msg.mission, mission_mgmt_msg.execute_mission))

    def _autonomy_bus_logger_callback(self, data):
        """
            Callback method for the 'autonomy_bus_logger_out' topic

            Parameters
            +--------------------+
            data: 'String' message type containing logging info from 'autonomy_bus_pkg'
        """
        # For debugging
        rospy.loginfo('Situational awareness receiving: [{0}] from /autonomy_bus_logger_out'.format(data.data))
        # Populate logging data pertaining to the autonomy bus
        self.data_logger['autonomy_bus_log'] = data.data

def main():
    """
        Driver method for 'situational_awareness.py'
    """
    # Start 'SituationalAwareness'
    situational_awareness = SituationalAwareness()

if __name__ == '__main__':
    main()