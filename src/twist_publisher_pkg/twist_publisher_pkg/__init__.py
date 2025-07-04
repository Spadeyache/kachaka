from setuptools import setup

package_name = 'twist_publisher_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Publisher node that sends Twist messages',
    license='MIT',
    entry_points={
        'console_scripts': [
            'twist_publisher = twist_publisher_pkg.twist_publisher:main',
        ],
    },
)
