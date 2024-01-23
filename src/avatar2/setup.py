from setuptools import setup

package_name = 'avatar2'

setup(
    name=package_name,
    version='0.0.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='walleed',
    maintainer_email='walleed@todo.todo',
    description='TODO',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sound_capture = avatar2.audio_input:main',
            'sound_play = avatar2.audio_input_wav:main',
            'sound_dump = avatar2.audio_dump:main',
            'sound_to_text = avatar2.audio_to_text:main',
            'avatar_camera = avatar2.opencv_camera:main',
            'head_info = avatar2.yolo_head:main',
            'sentiment_analysis = avatar2.sentiment_analysis:main',
        ],
    },
)
