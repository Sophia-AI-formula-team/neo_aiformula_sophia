from setuptools import Extension, setup
from Cython.Build import cythonize
import os

name = 'concept_candidates'
setup(
    name=name,
    version='6.26.0',
    packages=[name],
    ext_modules=cythonize(
        [Extension(name + '.' + module, [name + '/' + module + '.pyx'],
                   extra_compile_args=['/O2', '/fp:strict'] if os.name == 'nt' else ['-O3', '-ffp-contract=off'])
         for module in ['_progress']],
        compiler_directives={'language_level': 3}, quiet=True, build_dir='build/cython'),

    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml']),
    ],
    package_data={name: []},
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Sophia AI Formula Team',
    maintainer_email='maintainers@example.invalid',
    description='V6.26 forward trajectories, contingencies and reverse recovery.',
    license='AGPL-3.0-only',
)
