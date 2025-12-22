from setuptools import find_packages, setup

package_name = 'my_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kelenna-udo',
    maintainer_email='kelenna-udo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_news_station = my_py_pkg.topics.robot_news_station:main',
            'my_oop_node = my_py_pkg.nodes.my_oop_node:main',
            'my_first_node = my_py_pkg.nodes.my_first_node:main',
            'smart_phone = my_py_pkg.topics.smart_phone:main',
            'add_two_ints_server = my_py_pkg.services.add_two_ints_server:main',
            'add_two_ints_client = my_py_pkg.services.add_two_ints_client:main',
            'add_two_ints_client_no_oop = my_py_pkg.services.add_two_ints_client_no_oop:main',
            'hardware_status_publisher = my_py_pkg.custom_interfaces.hardware_status_publisher:main',
            'number_publisher = my_py_pkg.parameters.number_publisher:main',
            'number_counter = my_py_pkg.parameters.number_counter:main'
        ],
    },
)
