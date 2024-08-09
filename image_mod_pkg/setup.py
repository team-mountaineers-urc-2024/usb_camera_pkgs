from setuptools import find_packages, setup

package_name = 'image_mod_pkg'

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
    maintainer='jdb3',
    maintainer_email='jalen.beeman@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_flip = image_mod_pkg.image_flip:main',
            'image_convert= image_mod_pkg.image_convert:main'
        ],
    },
)
