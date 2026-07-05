from setuptools import find_packages, setup

package_name = 'abra_stanley_controller_new'

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
    maintainer='mirac',
    maintainer_email='miracgur061@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'abra_stanley_controller_new_node = abra_stanley_controller_new.abra_stanley_controller_new_node:main'
        ],
    },
)
