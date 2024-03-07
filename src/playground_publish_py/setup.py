from setuptools import find_packages, setup

package_name = 'playground_publish_py'

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
    maintainer='ros',
    maintainer_email='tobias@aescape.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'heavy_pub_node = playground_publish_py.heavy_pub_node:main',
            'qos_pub_node = playground_publish_py.qos_pub_node:main',
        ],
    },
)
