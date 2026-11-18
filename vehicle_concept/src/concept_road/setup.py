import os

from Cython.Build import cythonize
from setuptools import Extension, setup

name = 'concept_road'
setup(
    name=name,
    version='6.26.0',
    packages=[name],
    ext_modules=cythonize(
        [Extension(name + '._markings', [name + '/_markings.pyx'],
                   extra_compile_args=['/O2', '/fp:strict'] if os.name == 'nt'
                   else ['-O3', '-ffp-contract=off'])],
        compiler_directives={'language_level': 3}, quiet=True, build_dir='build/cython'),
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
