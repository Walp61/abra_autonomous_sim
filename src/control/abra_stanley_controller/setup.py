from setuptools import find_packages, setup

package_name = 'abra_stanley_controller'

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
    maintainer='hezarfen',
    maintainer_email='hezarfen@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'abra_stanley_controller_node = abra_stanley_controller.abra_stanley_controller_node:main'
        ],
    },
)
