from setuptools import find_packages, setup

package_name = 'hitachi_gui'

setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='kwd',
    maintainer_email='kwd@example.com',

    description='GUI for Hitachi AMR transformer inspection system',

    license='Apache-2.0',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'gui_node = hitachi_gui.gui_node:main',
        ],
    },
)
