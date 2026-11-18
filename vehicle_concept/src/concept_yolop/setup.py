from glob import glob
from setuptools import find_packages, setup

setup(
    name='concept_yolop',
    version='6.27.4',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/concept_yolop']),
        ('share/concept_yolop', ['package.xml', 'README.md', 'SOURCES.md']),
        ('share/concept_yolop/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    python_requires='>=3.8',
    maintainer='Sophia AI Formula',
    maintainer_email='maintainers@example.invalid',
    description='Frozen B1.4 camera perception for concept driving.',
    license='AGPL-3.0-only',
    zip_safe=False,
    entry_points={'console_scripts': [
        'concept_yolop_node = concept_yolop.node:main',
        'concept_yolop_weights = concept_yolop.weights:main',
        'concept_yolop_check = concept_yolop.check:main',
        'concept_yolop_calibrate = concept_yolop.calibrate:main',
    ]},
)
