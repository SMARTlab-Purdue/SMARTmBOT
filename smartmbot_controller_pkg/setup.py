import os
from glob import glob
from setuptools import setup


package_name = 'smartmbot_controller_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),   
        (os.path.join('share', package_name), glob('launch/*.launch.py')), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='heav101-pc1',
    maintainer_email='wonsu0513@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		    'go_to_goal_controller = smartmbot_controller_pkg.smartmbot_go_to_goal:main',
            'pure_pursuit_controller = smartmbot_controller_pkg.smartmbot_pure_pursuit:main',
			'line_tracing_controller = smartmbot_controller_pkg.smartmbot_line_tracer:main',

        ],
    },
)
