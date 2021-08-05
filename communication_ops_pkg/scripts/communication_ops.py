#!/usr/bin/env python

import json
import rospy
from std_msgs.msg import String
from communication_ops_pkg.msg import CommunicationOperations as CommunicationOperationsMsg
from autonomy_bus_pkg.msg import AutonomyBus as AutonomyBusMsg

class CommunicationOperations:
    """
        This class represents the communication operations module specified in UMAA.

        'CommunicationOperations' handles the communications on board the UUV. Which
        type of communication, user interfacing, and encoding/decoding. Manages bandwidth 
        and packet routing to optimize the use of multiple communication links based on 
        factors including priority, compression, network availability, and QoS
    """

    _COMMUNICATION_OPS_MSG = CommunicationOperationsMsg()
    
    def __init__(self):
        """
            Constructs a 'CommunicationOperations' object. In constructing a 'CommunicationOperations'
            object you are creating a ROS node which handles communication operations on board a UUV.

            Fields
            +--------------------+
            pub_data: A ROS publisher which retrieves conglomerated data from 'autonomy_bus', parses that data
                    then publishing parsed data back to 'autonomy_bus'
            pub_logger: A ROS publisher which broadcasts logging info about 'CommunicationOperations' object
            data: A dictionary data structure that stores parsed info from conglomerated data 'autonomy_bus' sends out
                    (key: module_name, value: module_name_message_type)
            data_logger: A dictionary data structure that stores incoming logging info from 'autonomy_bus'
                    (key: module_name, value: module_published_logging_data)
            _COMMUNICATION_OPS_MSG: An 'CommunicationOperationsMsg' that belongs to 'CommunicationOperations' class 
                    for updating and publishing

            How Things Work:
            +--------------------+
            1. 'CommunicationOperations' listens to only 'AutonomyBus'
            2. The data from 'AutonomyBus' is a big blob of data comprised of all other module's/package's
                    information
            3. The blob of data is parsed for the data needed for 'CommunicationOperations'
            4. 'data' field is a dictionary of (key: module_name, value: module_name_message_type)
            5. 'data' should at least store an 'autonomy_bus', 'communication_ops', and other module names
                    pertinent to 'communication_ops' as keys
                - e.g., 'data = {'autonomy_bus': AutonomyBusMsg, 'communication_ops': CommunicationOperationsMsg,
                'engineering_ops': EngineeringOperationsMsg}', in this case 'CommunicationOperations' 
                relies on data from 'engineering_ops_pkg'
        """
        # Give message dummy values
        self._COMMUNICATION_OPS_MSG.wired = True
        self._COMMUNICATION_OPS_MSG.optical = False
        self._COMMUNICATION_OPS_MSG.encoding = 'N/A'
        self._COMMUNICATION_OPS_MSG.decoding = 'N/A'
        # Create a ROS node named, "communication_ops"
        rospy.init_node('communication_ops', anonymous=True)
        # Setup publisher of data
        self.pub_data = rospy.Publisher('communication_ops_out', CommunicationOperationsMsg, queue_size=10)
        # Setup publisher of logging info
        self.pub_logger = rospy.Publisher('communication_ops_logger_out', String, queue_size=10)
        # Setup data structure for actual data
        self.data = {}
        # Setup data structure for logging
        self.data_logger = {}
        # Setup modules/packages to listen to (for data info)
        self._listen_data()
        # Setup modules/packages to listen to (for logging info)
        self._listen_logger()
        # Start publishing parsed data from 'autonomy_bus'
        self._publish()

    def _publish(self):
        """
            Publishes incoming messages from 'autonomy_bus_pkg' in a uniform format.
        """
        # Initialize the rate at which messages
        rate = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            # Store 'CommunicationOperationsMsg' in 'data'
            self.data['communication_ops'] = self._COMMUNICATION_OPS_MSG
            # Update timestamp of 'CommunicationOperationsMsg' before every publication
            self._COMMUNICATION_OPS_MSG.header.stamp = rospy.Time.now()
            # Create logger message
            logger_msg = '[{0}] "communication_ops" node is okay!'.format(rospy.Time.now())
            # Publish "CommunicationOperationsMsg" to "communication_ops_out" topic
            self.pub_data.publish(self._COMMUNICATION_OPS_MSG)
            # Publish "logger_msg" to "communication_ops_logger_out" topic
            self.pub_logger.publish(logger_msg)
            # Sleep for 2 second
            rate.sleep()
        print('"communication_ops" node has gone offline.')
            
    def _listen_data(self):
        """
            Setup 'CommunicationOperations' to listen other module's/package's actual data info
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
            topic and parses it for data relevant to this module. 

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

    def _autonomy_bus_logger_callback(self, data):
        """
            Callback method for the 'autonomy_bus_logger_out' topic

            Parameters
            +--------------------+
            data: 'String' message type containing logging info from 'autonomy_bus_pkg'
        """
        # For debugging
        rospy.loginfo('Communication operations receiving: [{0}] from /autonomy_bus_logger_out'.format(data.data))
        # Populate logging data pertaining to the autonomy bus
        self.data_logger['autonomy_bus_log'] = data.data

def main():
    """
        Driver method for 'communication_ops.py'
    """
    # Start 'CommunicationOperations'
    communication_ops = CommunicationOperations()

if __name__ == '__main__':
    main()