from setuptools import setup

package_name = 'proj_1'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name), glob("launch/*_launch.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='greatgnu',
    maintainer_email='email@email.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener1 = proj_1.joy:main'
            'listener2 = proj_1.launch_point_at_carrot:main'
            'listener3 = proj_1.PoseEstimator:main'
            'listener4 = proj_1.vehicle_controller:main'
        ],
    },
)
