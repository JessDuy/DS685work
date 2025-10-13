from setuptools import setup

package_name = 'objdet'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Object detection node with pgvector storage',
    entry_points={
        'console_scripts': [
            'objdet_node = objdet.objdet_node:main',
        ],
    },
)
