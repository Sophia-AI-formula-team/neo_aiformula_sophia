import os

from Cython.Distutils import build_ext
from setuptools import Extension, setup

name = 'concept_road'
native_sources = {'_markings': '_markings.pyx', '_memory': '_memory.pyx', 'field_scan': 'field_scan.py'}
compiler_flags = ['/O2', '/fp:strict'] if os.name == 'nt' else ['-O3', '-ffp-contract=off']
setup(
    name=name,
    version='6.27.6',
    packages=[name],
    cmdclass={'build_ext': build_ext},
    options={'build_ext': {'cython_c_in_temp': True, 'cython_directives': {'language_level': 3}}},
    ext_modules=[Extension(name + '.' + module, [name + '/' + source], extra_compile_args=compiler_flags)
                 for module, source in native_sources.items()],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml']),
    ],
    package_data={name: ['v3_evidence/plant_fit.json', 'field_scan.pxd']},
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Sophia AI Formula Team',
    maintainer_email='maintainers@example.invalid',
    description='V6.26 road, marking and local odometry memory.',
    license='AGPL-3.0-only',
)
