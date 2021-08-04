#!/usr/bin/env python

import json
import rospy
from std_msgs.msg import String
from mission_mgmt_pkg.msg import MissionManagement as MissionManagementMsg
from autonomy_bus_pkg.msg import AutonomyBus as AutonomyBusMsg

class MissionMgmt:
    """
        This class represents the mission management module specified in UMAA.

        'MissionMgmt' provides the management and execution of the overall mission and 
        governs the overall operation of the system.
    """

    _MISSION_MGMT_MSG = MissionManagementMsg()

    def __init__(self):
        """
            Constructs an 'MissionMgmt' object. In constructing a 'MissionMgmt'
            object you are creating a ROS node which management and execution of the overall mission and
            governs the overall operation of the system.

            Fields
            +--------------------+
            pub_data: A ROS publisher which retrieves conglomerated data from 'autonomy_bus', parses that data
                    then publishing parsed data back to 'autonomy_bus'
            pub_logger: A ROS publisher which broadcasts logging info about 'MissionMgmt' object
            data: A dictionary data structure that stores parsed info from conglomerated data 'autonomy_bus' sends out
                    (key: module_name, value: module_name_message_type)
            data_logger: A dictionary data structure that stores incoming logging info from 'autonomy_bus'
                    (key: module_name, value: module_published_logging_data)
            _MISSION_MGMT_MSG: An 'MissionManagementMsg' that belongs to 'MissionMgmt' class for updating 
                    and publishing

            How Things Work:
            +--------------------+
            1. 'MissionMgmt' listens to only 'AutonomyBus'
            2. The data from 'AutonomyBus' is a big blob of data comprised of all other module's/package's
                    information
            3. The blob of data is parsed for the data needed for 'MissionMgmt'
            4. 'data' field is a dictionary of (key: module_name, value: module_name_message_type)
            5. 'data' should at least store an 'autonomy_bus', 'mission_mgmt', and other module names
                    pertinent to 'maneuver_ops' as keys
                - e.g., 'data = {'autonomy_bus': AutonomyBusMsg, 'mission_mgmt': MissionManagementMsg,
                'support_ops': SupportOperationsMsg}', in this case 'MissionMgmt' 
                relies on data from 'support_ops_pkg'
        """
        # Give message dummy values
        self._MISSION_MGMT_MSG.mission = 'light tracking'
        self._MISSION_MGMT_MSG.execute_mission = True
        # Create a ROS node named, "maneuver_ops"
        rospy.init_node('mission_mgmt', anonymous=True)
        # Setup publisher of data
        self.pub_data = rospy.Publisher('mission_mgmt_out', MissionManagementMsg, queue_size=10)
        # Setup publisher of logging info
        self.pub_logger = rospy.Publisher('mission_mgmt_logger_out', String, queue_size=10)
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

    def _publish(self):
        """
            Publishes incoming messages from 'autonomy_bus_pkg' in a uniform format.
        """
        # Initialize the rate at which messages
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            # Store 'MissionManagementMsg' in 'data'
            self.data['mission_mgmt'] = self._MISSION_MGMT_MSG
            # Update timestamp of 'MissionManagementMsg' before every publication
            self._MISSION_MGMT_MSG.header.stamp = rospy.Time.now()
            # Create logger message
            logger_msg = '[{0}] "mission_mgmt" node is okay!'.format(rospy.Time.now())
            # Publish "MissionManagementMsg" to "mission_mgmt_out" topic
            self.pub_data.publish(self._MISSION_MGMT_MSG)
            # Publish "logger_msg" to "mission_mgmt_logger_out" topic
            #self.pub_logger.publish(logger_msg)
            # Sleep for 1 second
            rate.sleep()
        print('"mission_mgmt" node has gone offline.')
            
    def _listen_data(self):
        """
            Setup 'MissionMgmt' to listen other module's/package's actual data info
        """
        # Setup this node's subscribers in regards to actual data
        rospy.Subscriber('autonomy_bus_out', AutonomyBusMsg, self._autonomy_bus_callback)

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
        # TODO: Parse "data"
        pass

    def _autonomy_bus_logger_callback(self, data):
        """
            Callback method for the 'autonomy_bus_logger_out' topic

            Parameters
            +--------------------+
            data: 'String' message type containing logging info from 'autonomy_bus_pkg'
        """
        # For debugging
        rospy.loginfo('Mission managemant receiving: [{0}] from /autonomy_bus_logger_out'.format(data.data))
        # Populate logging data pertaining to the autonomy bus
        self.data_logger['autonomy_bus_log'] = data.data

def main():
    """
        Driver method for 'mission_mgmt.py'
    """
    # Start 'MissionMgmt'
    mission_mgmt = MissionMgmt()

if __name__ == '__main__':
    main()