import launch
import launch_ros.actions 

def generate_launch_description():
    return launch.LaunchDescription([
     launch_ros.actions.Node(
         package='first_project',
         executable='sender',
         name='sender'
     ),
      launch_ros.actions.Node(
         package='first_project',
         executable='reciever',
         name='reciever'
     ),

    ])