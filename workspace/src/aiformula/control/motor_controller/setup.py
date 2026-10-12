from setuptools import setup
from common_python.setup_util import get_data_files

package_name = 'motor_controller'

setup(
    name=package_name,
    version='0.3.0',
    packages=[package_name],
    data_files=get_data_files(package_name, ("config", "launch",)) + [("share/" + package_name, ["README.md", "UPSTREAM.json", "LICENSE"])],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='masayaokada',
    maintainer_email='masayaokada@jp.honda',
    description='Vehicle sigmoid steering compensation with fixed wheel RPM compensation.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller=motor_controller.motor_controller:main',
        ],
    },
)
