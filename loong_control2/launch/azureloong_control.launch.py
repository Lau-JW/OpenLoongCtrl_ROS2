from launch import LaunchDescription
from launch_ros.actions import Node

#from launch.actions import ExecuteProcess
#from launch.substitutions import FindExecutable

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration,Command
from launch_ros.parameter_descriptions import ParameterValue

#from launch.actions import IncludeLaunchDescription
#from launch.launch_description_sources import PythonLaunchDescriptionSource

#from launch_ros.actions import PushRosNamespace
#from launch.actions import GroupAction

#from launch.event_handlers import OnProcessStart,OnProcessExit
#from launch.actions import ExecuteProcess,RegisterEventHandler,LogInfo

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
 
def generate_launch_description():

    model_urdf_dir = get_package_share_directory("azureloong_description")
    
    default_model_path =os.path.join(
        model_urdf_dir,"urdf/","AzureLoong.urdf"
    )
    model=DeclareLaunchArgument(
        'model', default_value=default_model_path,
        description='Absolute path to robot urdf file'
    )
    robot_description=ParameterValue(Command(["xacro ",LaunchConfiguration("model")]))

    default_rviz_path=os.path.join(model_urdf_dir,"rviz/","default.rviz")

    robot_state_publisher=Node(
      package="robot_state_publisher",
      executable="robot_state_publisher",
      parameters=[{"robot_description":robot_description}]
    )

    rviz2=Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d',default_rviz_path]
        )
    joint_state_publisher=Node(
      package="joint_state_publisher",
      executable="joint_state_publisher"
    )

    Azureloong_control=Node(
            package='loong_control',
            executable='loong_control2',
            name='loong_control'
        )

    return LaunchDescription([
        model,
        Azureloong_control,
        robot_state_publisher,
        rviz2,
        
        joint_state_publisher
    ])
