from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command, FindExecutable
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('dogbot_description')
    sdf_file = os.path.join(pkg_path, 'sdf', 'dogbot.sdf')

    return LaunchDescription([

        ExecuteProcess(
            cmd=[
                'gz', 'sim',
                '--render-engine', 'ogre',
                os.path.join('/usr/share/gz/gz-sim8/worlds/empty.sdf')
            ],
            output='screen'
        ),

        ExecuteProcess(
            cmd=[
                'gz', 'service', '-s', '/world/empty/create',
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--req', f"sdf_filename: '{sdf_file}', name: 'dogbot', pose: {{ position: {{ z: 0.25 }} }}",
                '--timeout', '5000'
            ],
            output='screen'
        )
    ])
