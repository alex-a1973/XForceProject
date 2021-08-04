#!/usr/bin/env python

import json
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from processing_ops_pkg.msg import ProcessingOperations as ProcessingOperationsMsg
from autonomy_bus_pkg.msg import AutonomyBus as AutonomyBusMsg

class ProcessingOperations:
    """
        This class represents the processing operations module specified in UMAA.

        'ProcessingOperations' manages software services that refine data received 
        from sensing and other systems into higher-level info constructs.
    """

    _PROCESSING_OPS_MSG = ProcessingOperationsMsg()

    def __init__(self):
        """
            Constructs an 'ProcessingOperations' object. In constructing a 'ProcessingOperations'
            object you are creating a ROS node which handles refinement of data received 
            from sensing.

            Fields
            +--------------------+
            pub_data: A ROS publisher which retrieves conglomerated data from 'autonomy_bus', parses that data
                    then publishing parsed data back to 'autonomy_bus'
            pub_logger: A ROS publisher which broadcasts logging info about 'ProcessingOperations' object
            data: A dictionary data structure that stores parsed info from conglomerated data 'autonomy_bus' sends out
                    (key: module_name, value: module_name_message_type)
            data_logger: A dictionary data structure that stores incoming logging info from 'autonomy_bus'
                    (key: module_name, value: module_published_logging_data)
            _PROCESSING_OPS_MSG: An 'ProcessingOperationsMsg' that belongs to 'ProcessingOperations' class for updating 
                    and publishing

            How Things Work:
            +--------------------+
            1. 'ProcessingOperations' listens to only 'AutonomyBus'
            2. The data from 'AutonomyBus' is a big blob of data comprised of all other module's/package's
                    information
            3. The blob of data is parsed for the data needed for 'ProcessingOperations'
            4. 'data' field is a dictionary of (key: module_name, value: module_name_message_type)
            5. 'data' should at least store an 'autonomy_bus', 'processing_ops', and other module names
                    pertinent to 'processing_ops' as keys
                - e.g., 'data = {'autonomy_bus': AutonomyBusMsg, 'processing_ops': ProcessingOperationsMsg,
                'support_ops': SupportOperationsMsg}', in this case 'ProcessingOperations' 
                relies on data from 'support_ops_pkg'
        """
        # Create a ROS node named, "processing_ops"
        rospy.init_node('processing_ops', anonymous=True)
        # TODO: Setup publisher of data
        #self.pub_data = rospy.Publisher('processing_ops_out', _, queue_size=10)
        # Setup publisher of logging info
        self.pub_logger = rospy.Publisher('processing_ops_logger_out', String, queue_size=10)
        # Setup data structure for actual data
        self.data = {}
        # Setup data structure for logging
        self.data_logger = {}
        # TODO: Setup modules/packages to listen to (for data info)
        #self._listen_data()
        # Setup modules/packages to listen to (for logging info)
        self._listen_logger()
        # Start publishing parsed data from 'autonomy_bus'
        self._publish()

    def _publish(self):
        """
            Publishes incoming messages from 'autonomy_bus_pkg' in a uniform format.
        """
        # Initialize the rate at which messages
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            # Store 'ProcessingOperationsMsg' in 'data'
            self.data['processing_ops'] = self._PROCESSING_OPS_MSG
            # Update timestamp of 'ProcessingOperationsMsg' before every publication
            self._PROCESSING_OPS_MSG.header.stamp = rospy.Time.now()
            # Create logger message
            logger_msg = '[{0}] "processing_ops" node is okay!'.format(rospy.Time.now())
            # TODO: Publish "ProcessingOperationsMsg" to "processing_ops_out" topic
            #self.pub_data.publish(_)
            # Publish "logger_msg" to "processing_ops_logger_out" topic
            self.pub_logger.publish(logger_msg)
            # Sleep for 1 second
            rate.sleep()
        print('"processing_ops" node has gone offline.')
            
    def _listen_data(self):
        """
            Setup 'ProcessingOperations' to listen other module's/package's actual data info
        """
        # TODO: Setup this node's subscribers in regards to actual data
        #rospy.Subscriber('autonomy_bus_out', _, self._)

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
        rospy.loginfo('Processing operations receiving: [{0}] from /autonomy_bus_logger_out'.format(data.data))
        # Populate logging data pertaining to the autonomy bus
        self.data_logger['autonomy_bus_log'] = data.data

def main():
    """
        Driver method for 'processing_ops.py'
    """
    # Start 'ProcessingOperations'
    processing_ops = ProcessingOperations()

if __name__ == '__main__':
    main()