from setuptools import setup

package_name = 'smart_nav_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vaseegaran',
    maintainer_email='vasee_vasanth@outlook.com',
    description='Smart navigation bot package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_controller = smart_nav_bot.voice_controller:main',
            'room_saver = smart_nav_bot.room_saver:main',
            'result_subscriber = smart_nav_bot.result_subscriber:main',
        ],
    },
)
