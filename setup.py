from setuptools import setup

package_name = 'turtle_autonomy'

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
    maintainer='satwik',
    maintainer_email='satwik@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'turtle_odometry = turtle_autonomy.turtle_odometry:main',
        'turtle_autonomy = turtle_autonomy.turtle_autonomy_node:main',
    ],
},

)
