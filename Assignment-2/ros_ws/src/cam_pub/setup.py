from setuptools import setup

package_name = 'cam_pub'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Webcam/video publisher',
    entry_points={
        'console_scripts': [
            'cam_pub_node = cam_pub.cam_pub_node:main',
        ],
    },
)
