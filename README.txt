Krill UUV ROS Packages

'simulation' Branch
+----------------------------------------------------------+
    System Ran On (Jetson Nano Development Kit):
        - OS: Ubuntu 18.04.5 LTS
        - Kernel: Linux 4.9.201-tegra
        - Architecture: arm64
        - ROS Version: ROS Melodic (Desktop-Full Install)
            - Installation Link: http://wiki.ros.org/melodic/Installation/Ubuntu

    +----------------------------------------------------------+
    Components/Packages (File Structure w/ Descriptions)

        - ~/<ros_workspace_folder>/src/<repo_name>/simulation_pkg/
            |__ include/
            |__ launch/ --> Folder which contain '.launch' files that may run one or more ROS nodes. Ultimately running the package
            |       |__ simple_robot.launch --> File launches a Gazebo simulation including the world and simple robot created
            |
            |__ models/
            |       |__ light_source/ --> Folder that contains configuration files for creating a sphere for our simulation
            |       |       |__ model.config --> File specifies metadata for the 'model.sdf' file such as author, version, etc.
            |       |       |__ model.sdf --> File actually creates the sphere or so-called "light source" to track
            |       |
            |       |__ simple_robot/ --> Folder that contains configuration files for creating a simple robot for our simulation
            |               |__ simple_robot.gazebo --> File specifies customization for our simple robot as well as plugins
            |               |__ simple_robot.urdf.xacro --> File builds/creates the simple robot
            |
            |__ worlds/ --> Folder which contains all the worlds needed for simulation
            |       |__ simple_robot.world --> '.world' file which specifies the environment/world for simulation software
            |
            |__ CMakeLists.txt --> Used for compilation of whole workspace when calling "catkin_make"
            |__ package.xml --> Where you specify external packages required for this package    

        - ~/<ros_workspace_folder>/src/<repo_name>/autonomy_bus_pkg/
            |__ include/
            |__ launch/ --> Folder which contain '.launch' files that may run one or more ROS nodes. Ultimately running the package
            |       |__ autonomy_bus.launch --> File launches the 'autonomy_bus.py' file which initializes a ROS node
            |
            |__ msg/ --> Where custom message types reside for 'autonomy_bus_pkg'
            |       |__ AutonomyBus.msg --> This message is comprised of metadata and other ROS packages custom message types
            |
            |__ scripts/
            |       |__ autonomy_bus.py --> This file creates a ROS node which simulates an autonomy data bus
            |
            |__ src/ --> Folder which contains source code
            |
            |__ CMakeLists.txt --> Used for compilation of whole workspace when calling "catkin_make"
            |__ package.xml --> Where you specify external packages required for this package

        - ~/<ros_workspace_folder>/src/<repo_name>/communication_ops_pkg/
            |__ include/
            |__ launch/ --> Folder which contain '.launch' files that may run one or more ROS nodes. Ultimately running the package
            |       |__ communication_ops.launch --> File launches the 'communication_ops.py' file which initializes a ROS node
            |
            |__ msg/ --> Where custom message types reside for 'communication_ops_pkg'
            |       |__ CommunicationOperations.msg --> This message is comprised of the data that this package should broadcast
            |
            |__ scripts/ --> Where Python scripts reside
            |       |__ communication_ops.py --> This file creates a ROS node which simulates communication operations
            |
            |__ src/ --> Folder which contains source code
            |
            |__ CMakeLists.txt --> Used for compilation of whole workspace when calling "catkin_make"
            |__ package.xml --> Where you specify external packages required for this package

        - ~/<ros_workspace_folder>/src/<repo_name>/engineering_ops_pkg/
            |__ include/
            |__ launch/ --> Folder which contain '.launch' files that may run one or more ROS nodes. Ultimately running the package
            |       |__ engineering_ops.launch --> File launches the 'engineering_ops.py' file which initializes a ROS node
            |
            |__ msg/ --> Where custom message types reside for 'engineering_ops_pkg'
            |       |__ EngineeringOperations.msg --> This message is comprised of the data that this package should broadcast
            |
            |__ scripts/ --> Where Python scripts reside
            |       |__ engineering_ops.py --> This file creates a ROS node which simulates engineering operations
            |
            |__ src/ --> Folder which contains source code
            |
            |__ CMakeLists.txt --> Used for compilation of whole workspace when calling "catkin_make"
            |__ package.xml --> Where you specify external packages required for this package

        - ~/<ros_workspace_folder>/src/<repo_name>/maneuver_ops_pkg/
            |__ include/
            |__ launch/ --> Folder which contain '.launch' files that may run one or more ROS nodes. Ultimately running the package
            |       |__ maneuver_ops.launch --> File launches the 'maneuver_ops.py' file which initializes a ROS node
            |
            |__ msg/ --> Where custom message types reside for 'maneuver_ops_pkg'
            |       |__ ManeuverOperations.msg --> This message is comprised of the data that this package should broadcast
            |
            |__ scripts/ --> Where Python scripts reside
            |       |__ maneuver_ops.py --> This file creates a ROS node which simulates maneuver operations
            |
            |__ src/ --> Folder which contains source code
            |
            |__ CMakeLists.txt --> Used for compilation of whole workspace when calling "catkin_make"
            |__ package.xml --> Where you specify external packages required for this package

        - ~/<ros_workspace_folder>/src/<repo_name>/mission_mgmt_pkg/
            |__ include/
            |__ launch/ --> Folder which contain '.launch' files that may run one or more ROS nodes. Ultimately running the package
            |       |__ mission_mgmt.launch --> File launches the 'mission_mgmt.py' file which initializes a ROS node
            |
            |__ msg/ --> Where custom message types reside for 'mission_mgmt_pkg'
            |       |__ MissionManagement.msg --> This message is comprised of the data that this package should broadcast
            |
            |__ scripts/ --> Where Python scripts reside
            |       |__ mission_mgmt.py --> This file creates a ROS node which simulates mission management
            |
            |__ src/ --> Folder which contains source code
            |
            |__ CMakeLists.txt --> Used for compilation of whole workspace when calling "catkin_make"
            |__ package.xml --> Where you specify external packages required for this package

        - ~/<ros_workspace_folder>/src/<repo_name>/processing_ops_pkg/
            |__ include/
            |__ launch/ --> Folder which contain '.launch' files that may run one or more ROS nodes. Ultimately running the package
            |       |__ processing_ops.launch --> File launches the 'processing_ops.py' file which initializes a ROS node
            |
            |__ msg/ --> Where custom message types reside for 'processing_ops_pkg'
            |       |__ ProcessingOps.msg --> This message is comprised of the data that this package should broadcast
            |
            |__ scripts/ --> Where Python scripts reside
            |       |__ processing_ops.py --> This file creates a ROS node which simulates processing operations
            |
            |__ src/ --> Folder which contains source code
            |
            |__ CMakeLists.txt --> Used for compilation of whole workspace when calling "catkin_make"
            |__ package.xml --> Where you specify external packages required for this package

        - ~/<ros_workspace_folder>/src/<repo_name>/sensor_effector_mgmt_pkg/
            |__ include/
            |__ launch/ --> Folder which contain '.launch' files that may run one or more ROS nodes. Ultimately running the package
            |       |__ sensor_effector_mgmt.launch --> File launches the 'sensor_effector_mgmt.py' file which initializes a ROS node
            |
            |__ msg/ --> Where custom message types reside for 'sensor_effector_mgmt_pkg'
            |       |__ SensorEffectorManagement.msg --> This message is comprised of the data that this package should broadcast
            |
            |__ scripts/ --> Where Python scripts reside
            |       |__ sensor_effector_mgmt.py --> This file creates a ROS node which simulates sensor effector management
            |
            |__ src/ --> Folder which contains source code
            |
            |__ CMakeLists.txt --> Used for compilation of whole workspace when calling "catkin_make"
            |__ package.xml --> Where you specify external packages required for this package

        - ~/<ros_workspace_folder>/src/<repo_name>/situational_awareness_pkg/
            |__ include/
            |__ launch/ --> Folder which contain '.launch' files that may run one or more ROS nodes. Ultimately running the package
            |       |__ situational_awareness.launch --> File launches the 'situational_awareness.py' file which initializes a ROS node
            |    
            |__ models/ --> Where custom models reside for 'situational_awareness_pkg'
            |       |__  --> This model is trained on a specific object to track within Gazebo
            |
            |__ msg/ --> Where custom message types reside for 'situational_awareness_pkg'
            |       |__ SituationalAwareness.msg --> This message is comprised of the data that this package should broadcast
            |
            |__ scripts/ --> Where Python scripts reside
            |       |__ situational_awareness.py --> This file creates a ROS node which simulates situational awareness
            |       |__ light_trackin.py --> This file uses custom exported model to track an object in Gazebo
            |
            |__ src/ --> Folder which contains source code
            |
            |__ CMakeLists.txt --> Used for compilation of whole workspace when calling "catkin_make"
            |__ package.xml --> Where you specify external packages required for this package

        - ~/<ros_workspace_folder>/src/<repo_name>/support_ops_pkg/
            |__ include/
            |__ launch/ --> Folder which contain '.launch' files that may run one or more ROS nodes. Ultimately running the package
            |       |__ support_ops.launch --> File launches the 'support_ops.py' file which initializes a ROS node
            |
            |__ msg/ --> Where custom message types reside for 'support_ops_pkg'
            |       |__ support_ops.msg --> This message is comprised of the data that this package should broadcast
            |
            |__ scripts/ --> Where Python scripts reside
            |       |__ support_ops.py --> This file creates a ROS node which simulates support operations
            |
            |__ src/ --> Folder which contains source code
            |
            |__ CMakeLists.txt --> Used for compilation of whole workspace when calling "catkin_make"
            |__ package.xml --> Where you specify external packages required for this package

    +----------------------------------------------------------+
    How To Run:
        1. Create a ROS workspace on your machine using 'catkin_make'
            - This assumes you have installed catkin and sourced your environment. Look at installation link above for tutorial
            - Create a folder (this will be your '<ros_workspace_folder>') with a 'src/' folder within it
            - 'cd' into the newly created folder
            - Call 'catkin_make' then you've just created a ROS workspace
        2. 'git clone' repository in your 'src/' folder after you've created your ROS workspace
            - A 'src/' folder should be created after you 'catkin_make'
            - 'cd <ros_workspace_folder>/src/'
            - 'git clone https://github.com/alex-a1973/UUVLightTrackingSrc.git'
        3. Now you have '~/<ros_workspace_folder>/src/UUVLightTrackingSrc/' which contains all the custom packages.
        4. You MUST make sure in '~/<ros_workspace_folder>/src/UUVLightTrackingSrc/autonomy_bus_pkg/CMakeLists.txt', under
                'generate_messages' all but 'std_msgs' and 'sensor_msgs' are commented out (i.e., all custom packages)
        5. You MUST make sure in '~/<ros_workspace_folder>/src/UUVLightTrackingSrc/autonomy_bus_pkg/msg/AutonomyBus.msg' all
                custom packages are commented out
        6. Once the appropriate stuff is commented out, 'cd ~/<ros_workspace_folder>' and 'catkin_make' (compile ROS workspace)
        7. After successful compilation, uncomment all the commented out sections from steps 4 and 5
        8. Open terminals (or terminal tabs) equal to the number of packages w/in this workspace (10)
        9. 'cd ~/krill_ws', Change directory into the 'krill_ws/' folder for all terminals
        10. 'source devel/setup.bash', Source the project for all terminals
        11. There should be a 'launch/' folder in every single one of these packages. You'll need to run each '.launch' file
                in each package in a separate terminal (hence the reason opening 10 terminals)
            - 'roslaunch <x>_pkg <y>.launch'
                - Replace '<x>' with the name of a specific package
                    - Note: All packages should be given a specific suffix, '_pkg'
                - Replace '<y>' with the name of the launch file under the already specified package replacing '<x>'

||=======================================================================================================||

Hardware Branch
---------------

Serial

- This folder contains the necessary contents for establishing a serial connection between the Jetson and Arduino
- Must be kept in the same directory as ADetection_xy.py

ADetection_xy.py

- This is the python script that runs DetectNet on the Jetson for Object Detection and tracking
- To run, you must set up DetectNet on your Jetson from source using the tutorial here: https://github.com/dusty-nv/jetson-inference/blob/master/docs/building-repo-2.md
- The code intakes Object Detection information and outputs instructions to the serial port, which are read by 


