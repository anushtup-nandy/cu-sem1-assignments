from setuptools import find_packages, setup

package_name = 'assignment1'

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
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'solution = assignment1.solution:main',
            'dxy_solution = assignment1.solution_dxy:main',
            'marker_publisher = assignment1.marker_publisher:main',
            'autograde = assignment1.autograde:main',
        ],
    },
)
