from setuptools import find_packages, setup

package_name = 'test_tutorial_interfaces'

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
    maintainer='zweminhtetaung',
    maintainer_email='zweminhtetaung@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'MinimalPublisher = test_tutorial_interfaces.MinimalPublisher:main',
            'MinimalSubscriber = test_tutorial_interfaces.MinimalSubscriber:main',
            'MinimalService = test_tutorial_interfaces.MinimalService:main',
            'MinimalClientAsync = test_tutorial_interfaces.MinimalClientAsync:main'
        ],
    },
)
