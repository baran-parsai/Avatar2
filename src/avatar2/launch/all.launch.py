import os
import sys
import json
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    root_dir = '/home/baranparsai/Documents/Avatar2/scenarios/hearing_clinic/faces'
    debug = False
    ui_imagery = '/home/baranparsai/Documents/Avatar2/ros_avatar'   # default imagery location
    ros_ui = False
    config_file = 'config.json'

    for arg in sys.argv[4:]:
        if arg.startswith('config_file:='):
            config_file = arg.split('config_file:=', 1)[1]
            print(f"Launching with config_file as {config_file}")
        elif arg.startswith('ros_ui'):
            print(f"Launching with debug is {arg.split('ros_ui:=', 1)[1]}")
            ros_ui = bool(arg.split('ros_ui:=', 1)[1])
            print(f'Launching with ros_ui as {ros_ui}')
        elif arg.startswith('ui_imagery:='):
           print(arg.split('ui_imagery:=', 1)[1])
           ui_imagery = arg.split('ui_imagery:=', 1)[1]
           print(f"Launching with imagery at {ui_imagery}")
        else:
            print("Usage: launch avatar2 all.launch.py [config_file:=<config_file>] [ui_imagery:=<ui_imagery_root>] [ros_ui:= False|True]")
            sys.exit()

    with open(config_file) as f:
        config = json.load(f)
    print(f)
    try:
        root = config['root']
    except:
        pass
    
    try:
        debug = boot(config['debug'])
    except:
        pass
    
    
    nodes = []
    microphone_node = Node(
             package='avatar2',
             executable='sound_capture',
             name='sound_capture',
             output='screen',
             namespace="/avatar2",
             parameters=[{'non_speaking_duration': 1.0, 'pause_threshold': 1.0}])
    nodes.append(microphone_node)

    sound_to_text_node = Node(
             package='avatar2',
             executable='sound_to_text',
             name='sound_to_text',
             output='screen',
             namespace="/avatar2")
    nodes.append(sound_to_text_node)

    text_to_sound = Node(
             package='avatar2',
             executable='text_to_sound',
             name='text_to_sound',
             output='screen',
             namespace="/avatar2")
    nodes.append(text_to_sound)

    camera_node = Node(
            package='avatar2',
            executable='avatar_camera',
            name='avatar_camera',
            output='screen',
            namespace="/avatar2")
    nodes.append(camera_node)

    face_recognizer_node = Node(
            package='avatar2',
            executable='head_detect',
            name='head_detect',
            output='screen',
            namespace="/avatar2",
            parameters=[{'root_dir' : root_dir, 'debug': debug}]) 
    nodes.append(face_recognizer_node)
        
    llm_wizard_clinic_node = Node(
            package='avatar2',
            executable='llm_engine',
            name='llm_engine',
            output='screen',
            namespace="/avatar2",
            parameters=[{'config_file': config_file}])
    nodes.append(llm_wizard_clinic_node)

    if ros_ui:
        ros_node = Node(
             package='avatar2',
             executable='ros_avatar',
             name='ros_avatar',
             output='screen',
             parameters=[{'imagery': ui_imagery, 'debug': debug}])
        nodes.append(ros_node)

    return LaunchDescription(nodes)
