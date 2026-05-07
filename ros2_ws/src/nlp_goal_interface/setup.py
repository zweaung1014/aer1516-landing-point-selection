from setuptools import find_packages, setup

package_name = 'nlp_goal_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zwe Min Htet Aung',
    maintainer_email='zweminhtetaung@todo.todo',
    description='Natural language goal specification interface for robot navigation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'nlp_goal_node = nlp_goal_interface.nlp_goal_node:main',
        ],
    },
)
