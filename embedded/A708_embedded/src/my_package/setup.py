from setuptools import setup

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SSAFY',
    maintainer_email='wjdcksdud96@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_package.my_node:main',
            'path_plan = my_package.path_plan:main',
            'local_path = my_package.global_to_local_path:main',
            'drive = my_package.follow_the_carrot:main',
            'appliance_ctrl = my_package.iot_udp:main',
            'load_map = my_package.load_map:main',
            'perception = my_package.create_logListId:main',
            'schedule = my_package.schedule_TTS:main',
            'weather_pub = my_package.weather_pub:main',
            'cctv = my_package.cctv_socket:main',
            'cctv_test = my_package.cctv_test:main',
            'follow = my_package.auto_follow:main'
        ],
    },
)
