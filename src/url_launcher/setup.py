from setuptools import setup
import os
from glob import glob

package_name = 'url_launcher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # 🚀 launch 폴더 추가!
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jinshim',
    maintainer_email='tlathwls0518@gmail.com',
    description='URL launcher for QR scanning',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'url_launcher_node = url_launcher.url_launcher_node:main',
        ],
    },
)

