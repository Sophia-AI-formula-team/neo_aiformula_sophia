import os

from Cython.Distutils import build_ext
from setuptools import Extension, setup

name = 'concept_road'
setup(
    name=name,
    version='6.27.6',
    packages=[name],
    cmdclass={'build_ext': build_ext},
    options={'build_ext': {'cython_c_in_temp': True, 'cython_directives': {'language_level': 3}}},
    ext_modules=[Extension(name + '.' + module, [name + '/' + module + '.pyx'],
                   extra_compile_args=['/O2', '/fp:strict'] if os.name == 'nt'
                   else ['-O3', '-ffp-contract=off']) for module in ('_markings', '_memory')],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml']),
    ],
    package_data={name: ['v3_evidence/plant_fit.json']},
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Sophia AI Formula Team',
    maintainer_email='maintainers@example.invalid',
    description='V6.26 road, marking and local odometry memory.',
    license='AGPL-3.0-only',
)
