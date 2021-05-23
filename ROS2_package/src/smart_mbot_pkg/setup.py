import os
from glob import glob
from setuptools import setup

package_name = 'smart_mbot_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wonse Jo and Jaeeun Kim',
    maintainer_email='jow@purdue.edu',
    description='This package is a fundamental pakage for SMARTmBOT. If you need more information, please visit our site; https://github.itap.purdue.edu/ByungcheolMinGroup/smart_mbot_ws',
    license='GNU General Public License v3.0: gpl-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Reading_SPI_ADC = smart_mbot_pkg.ros2_spi_adc_reading:main',
            'Writing_DC_Motor = smart_mbot_pkg.ros2_dc_motor_control:main',
            'Reading_I2C_ToF = smart_mbot_pkg.ros2_i2c_tof_reading:main',
            'Writing_WB2813b_RGB_STRIP = smart_mbot_pkg.ros2_rgb_strip_ws2812b:main',
            'Writing_GPIO_SMD5050_LED = smart_mbot_pkg.ros2_gpio_rgb_led:main',
        ],
    },
)
