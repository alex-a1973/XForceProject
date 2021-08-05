#!/usr/bin/env python

import json
import rospy
from std_msgs.msg import String
from autonomy_bus_pkg.msg import AutonomyBus as AutonomyBusMsg
from communication_ops_pkg.msg import CommunicationOperations as CommunicationOperationsMsg
from engineering_ops_pkg.msg import EngineeringOperations as EngineeringOperationsMsg
from maneuver_ops_pkg.msg import ManeuverOperations as ManeuverOperationsMsg
from mission_mgmt_pkg.msg import MissionManagement as MissionManagementMsg
from processing_ops_pkg.msg import ProcessingOperations as ProcessingOperationsMsg
from sensor_effector_mgmt_pkg.msg import SensorEffectorManagement as SensorEffectorManagementMsg
from situational_awareness_pkg.msg import SituationalAwareness as SituationalAwarenessMsg
from support_ops_pkg.msg import SupportOperations as SupportOperationsMsg

class AutonomyBus:
    """
        This class represents the autonomy bus specified in UMAA.

        Acts as a data bus between all modules (or packages in this case). Distributing
        data to all necessary modules and receiving data from all modules.
    """

    _AUTONOMY_BUS_MSG = AutonomyBusMsg()

    def __init__(self):
        """
            Constructs an 'AutonomyBus' object. In constructing an 'AutonomyBus' object you
            are creating a ROS node which resembles a central data bus

            Fields
            +--------------------+
            pub_data: A ROS publisher which broadcasts retrieved data to all listeners
            pub_logger: A ROS publisher which broadcasts logging info about 'AutonomyBus' object
            data: A dictionary data structure that stores all incoming data from all modules
                    (key: module_name, value: module_name_message_type)
            data_logger: A dictionary data structure that stores all incoming logging data from all modules
                    (key: module_name, value: module_published_logging_data)
            _AUTONOMY_BUS_MSG: An 'AutonomyBusMsg' that belongs to 'AutonomyBus' class for updating and
                    publishing

            How Things Work:
            +--------------------+
            1. 'AutonomyBus' listens to all other modules/packages
            2. Each module/package has a specific message type which contains the information and datatypes
                    that package is publishing or expecting to receive
            3. 'AutonomyBus.msg' is a custom message type specifically for the 'AutonomyBus' object which
                    is comprised of some metadata (std_msgs/Header) and all other module/package custom message
                    types
                - This allows 'AutonomyBus' to receive individual module's/package's message, store them in
                blob of data ('AutonomyBus.msg'), and publish that blob of data to individual modules/packages
                for them to parse for specific information they need
            4. 'data' field is a dictionary of (key: module_name, value: module_name_message_type) where we
                    update 'module_name's value when that specific module publishes its data and we've received
                    it
                - e.g. 'data = {'communication_ops': CommunicationOperations, 'engineering_ops': EngineeringOperations}',
                where 'CommunicationOperations' and 'EngineeringOperations' are both communication_ops_pkg &
                engineering_ops_pkg's custom message type
        """
        # Create a ROS node named, "autonomy_bus" (isn't used if you launch from .launch file)
        rospy.init_node('autonomy_bus', anonymous=True)
        # Setup publisher of data
        self.pub_data = rospy.Publisher('autonomy_bus_out', AutonomyBusMsg, queue_size=10)
        # Setup publisher of logging info
        self.pub_logger = rospy.Publisher('autonomy_bus_logger_out', String, queue_size=10)
        # Setup data structure for actual data
        self.data = {}
        # Setup data structure for logging
        self.data_logger = {}
        # Setup modules/packages to listen to (for data info)
        self._listen_data()
        # Setup modules/packages to listen to (for logging info)
        #self._listen_logger()
        # Start publishing incomimg data
        self._publish()

    def _publish(self):
        """
            Publishes incoming messages from all sources (modules that it listens too) in a
            uniform format.
        """
        # Initialize the rate at which messages
        rate = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            # Store 'AutonomyBusMsg' in 'data'
            self.data['autonomy_bus'] = self._AUTONOMY_BUS_MSG
            # Update timestamp of 'AutonomyBusMsg' before every publication
            self._AUTONOMY_BUS_MSG.header.stamp = rospy.Time.now()
            # Encode Python logging dictionary into JSON string
            encoded_log_data = json.dumps(self.data_logger)
            # Create logger message
            logger_msg = '[{0}] "autonomy_bus" node is okay! Logging Data: {1}'.format(rospy.Time.now(), encoded_log_data)
            # Publish "AutonomyBusMsg" to "autonomy_bus_out" topic
            self.pub_data.publish(self._AUTONOMY_BUS_MSG)
            # Publish "logger_msg" to "autonomy_bus_out_logger" topic
            #self.pub_logger.publish(logger_msg)
            # Sleep for 2 second
            rate.sleep()
            
    def _listen_data(self):
        """
            Setup 'AutonomyBus' to listen all other module's/package's actual data info
        """
        # Setup this node's subscribers in regards to actual data
        rospy.Subscriber('communication_ops_out', CommunicationOperationsMsg, self._communication_ops_callback)
        #rospy.Subscriber('engineering_ops_out', _, self._)
        #rospy.Subscriber('maneuver_ops_out', _, self._)
        rospy.Subscriber('mission_mgmt_out', MissionManagementMsg, self._mission_mgmt_callback)
        #rospy.Subscriber('processing_ops_out', _, self._)
        #rospy.Subscriber('sensor_effector_mgmt_out', _, self._)
        #rospy.Subscriber('situational_awareness_out', _, self._)
        #rospy.Subscriber('support_ops_out', _, self._)

    def _communication_ops_callback(self, data):
        """
            Callback method for the 'communication_ops_out' topic. This method takes data 
            published from 'communication_ops_pkg' ('CommunicationOperationsMsg'),
            overrides the already existing 'CommunicationOperationsMsg' in 'AutonomyBusMsg'
            for publishing the new 'AutonomyBusMsg' with updated 'communication_ops' data,
            and updates 'communication_ops' key of 'data' dictionary with new incoming data

            Parameters
            +--------------------+
            data: 'CommunicationOperations' message type containing data from 'communication_ops_pkg'
        """
        # For debugging purposes
        # data_str = 'Wired: {0} || Optical: {1} || Encoding: {2} || Decoding: {3} || User Interface: {4}'.format(data.wired, 
        #                                                                        data.optical, 
        #                                                                        data.encoding,
        #                                                                        data.decoding,
        #                                                                        data.user_interface)
        # rospy.loginfo(data_str)
        # Check if there is an 'communication_ops' key within 'data' field
        if ('communication_ops' in self.data):
            # Check if data sent from 'communication_ops_pkg' is different from previously sent data
            if (data.header.stamp != self.data['communication_ops'].header.stamp):
                # Update 'data' dict's 'communication_ops' key with new 'CommunicationOperationsMsg'
                self.data['communication_ops'] = data
                # Update 'AutonomyBusMsg' 'communication_ops' data field with new data
                self._AUTONOMY_BUS_MSG.communication_ops = data
        else:
            # Update 'data' dict's 'communication_ops' key with new 'CommunicationOperationsMsg'
            self.data['communication_ops'] = data
            # Update 'AutonomyBusMsg' 'communication_ops' data field with new data
            self._AUTONOMY_BUS_MSG.communication_ops = data

    def _mission_mgmt_callback(self, data):
        """
            Callback method for the 'mission_mgmt_out' topic. This method takes data 
            published from 'mission_mgmt_pkg' ('MissionManagementMsg'), overrides the already existing 
            'MissionManagementMsg' in 'AutonomyBusMsg' for publishing the new 'AutonomyBusMsg' 
            with updated 'mission_mgmt' data, and updates 'mission_mgmt' key of 'data'
            dictionary with new incoming data

            Parameters
            +--------------------+
            data: 'MissionManagement' message type containing data from 'mission_mgmt_pkg'
        """
        # For debugging purposes
        # data_str = 'Mission: {0} || Execute? {1}'.format(data.mission, data.execute_mission)
        # rospy.loginfo(data_str)
        # Check if there is an 'mission_mgmt' key within 'data' field
        if ('mission_mgmt' in self.data):
            # Check if data sent from 'mission_mgmt_pkg' is different from previously sent data
            if (data.header.stamp != self.data['mission_mgmt'].header.stamp):
                # Update 'data' dict's 'mission_mgmt' key with new 'MissionManagementMsg'
                self.data['mission_mgmt'] = data
                # Update 'AutonomyBusMsg' 'mission_mgmt' data field with new data
                self._AUTONOMY_BUS_MSG.mission_mgmt = data
        else:
            # Update 'data' dict's 'mission_mgmt' key with new 'MissionManagementMsg'
            self.data['mission_mgmt'] = data
            # Update 'AutonomyBusMsg' 'mission_mgmt' data field with new data
            self._AUTONOMY_BUS_MSG.mission_mgmt = data

    def _listen_logger(self):
        """
            Setup the listener to listen all other module's/package's logging info
        """
        # Setup this node's subscribers in regards to logging info
        rospy.Subscriber('communication_ops_logger_out', String, self._communication_ops_logger_callback)
        rospy.Subscriber('engineering_ops_logger_out', String, self._engineering_ops_logger_callback)
        rospy.Subscriber('maneuver_ops_logger_out', String, self._maneuver_ops_logger_callback)
        rospy.Subscriber('mission_mgmt_logger_out', String, self._mission_mgmt_logger_callback)
        rospy.Subscriber('processing_ops_logger_out', String, self._processing_ops_logger_callback)
        rospy.Subscriber('sensor_effector_mgmt_logger_out', String, self._sensor_effector_mgmt_logger_callback)
        rospy.Subscriber('situational_awareness_logger_out', String, self._situational_awareness_logger_callback)
        rospy.Subscriber('support_ops_logger_out', String, self._support_ops_logger_callback)

    def _communication_ops_logger_callback(self, data):
        """
            Callback method for the 'communication_ops_logger_out' topic

            Parameters
            +--------------------+
            data: 'String' message type containing logging info from 'communication_ops_pkg'
        """
        # For debugging
        rospy.loginfo('Autonomy bus receiving: [{0}] from /communication_ops_logger_out'.format(data.data))
        # Populate logging data pertaining to communications operations
        self.data_logger['communication_ops_log'] = data.data
    
    def _engineering_ops_logger_callback(self, data):
        """
            Callback method for the 'engineering_ops_logger_out' topic

            Parameters
            +--------------------+
            data: 'String' message type containing logging info from 'engineering_ops_pkg'
        """
        # For debugging
        rospy.loginfo('Autonomy Bus receiving: [{0}] from /engineering_ops_logger_out'.format(data.data))
        # Populate logging data pertaining to engineering operations
        self.data_logger['engineering_ops_log'] = data.data
    
    def _maneuver_ops_logger_callback(self, data):
        """
            Callback method for the 'maneuver_ops_logger_out' topic

            Parameters
            +--------------------+
            data: 'String' message type containing logging info from 'maneuver_ops_pkg'
        """
        # For debugging
        rospy.loginfo('Autonomy Bus receiving: [{0}] from /maneuver_ops_logger_out'.format(data.data))
        # Populate logging data pertaining to maneuver operations
        self.data_logger['maneuver_ops_log'] = data.data

    def _mission_mgmt_logger_callback(self, data):
        """
            Callback method for the 'mission_mgmt_logger_out' topic

            Parameters
            +--------------------+
            data: 'String' message type containing logging info from 'mission_mgmt_pkg'
        """
        # For debugging
        rospy.loginfo('Autonomy Bus receiving: [{0}] from /mission_mgmt_logger_out'.format(data.data))
        # Populate logging data pertaining to mission management
        self.data_logger['mission_mgmt_log'] = data.data

    def _processing_ops_logger_callback(self, data):
        """
            Callback method for the 'processing_ops_logger_out' topic

            Parameters
            +--------------------+
            data: 'String' message type containing logging info from 'processing_ops_pkg'
        """
        # For debugging
        rospy.loginfo('Autonomy Bus receiving: [{0}] from /processing_ops_logger_out'.format(data.data))
        # Populate logging data pertaining to procesing operations
        self.data_logger['processing_ops_log'] = data.data

    def _sensor_effector_mgmt_logger_callback(self, data):
        """
            Callback method for the 'sensor_effector_mgmt_logger_out' topic

            Parameters
            +--------------------+
            data: 'String' message type containing logging info from 'sensor_effector_mgmt_pkg'
        """
        # For debugging
        rospy.loginfo('Autonomy Bus receiving: [{0}] from /sensor_effector_mgmt_logger_out'.format(data.data))
        # Populate logging data pertaining to sensor effector management
        self.data_logger['sensor_effector_mgmt_log'] = data.data

    def _situational_awareness_logger_callback(self, data):
        """
            Callback method for the 'situational_awareness_logger_out' topic

            Parameters
            +--------------------+
            data: 'String' message type containing logging info from 'situational_awareness_pkg'
        """
        # For debugging
        rospy.loginfo('Autonomy Bus receiving: [{0}] from /situational_awareness_logger_out'.format(data.data))
        # Populate logging data pertaining to situational awareness
        self.data_logger['situational_awareness_log'] = data.data

    def _support_ops_logger_callback(self, data):
        """
            Callback method for the 'support_ops_logger_out' topic

            Parameters
            +--------------------+
            data: 'String' message type containing logging info from 'support_ops_pkg'
        """
        # For debugging
        rospy.loginfo('Autonomy Bus receiving: [{0}] from /support_ops_logger_out'.format(data.data))
        # Populate logging data pertaining to support operations
        self.data_logger['support_ops_log'] = data.data

def main():
    """
        Driver method for 'autonomy_bus.py'
    """
    # Start 'AutonomyBus'
    autonomy_bus = AutonomyBus()

if __name__ == '__main__':
    main()