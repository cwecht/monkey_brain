from glob import glob

from setuptools import find_packages, setup

package_name = 'monkey_brain_demos'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
         ('share/' + package_name + '/launch', glob('launch/*launch.[pxy][yma]*')),
         ('share/' + package_name + '/config/atomic',
          glob('config/atomic/*')),
         ('share/' + package_name + '/config/service_client_demo',
          glob('config/service_client_demo/*')),
         ('share/' + package_name + '/config/action_client_demo',
          glob('config/action_client_demo/*')),
         ('share/ament_index/resource_index/packages',
          ['resource/' + package_name]),
         ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
