from setuptools import setup

package_name = 'ros2_yolov5'

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
    maintainer='philambda',
    maintainer_email='phlicht64@gmail.com',
    description='Object detection with custom yolov5 model in ros2 foxy',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inferator = ros2_yolov5.inference:main',
        ],
    },
)
