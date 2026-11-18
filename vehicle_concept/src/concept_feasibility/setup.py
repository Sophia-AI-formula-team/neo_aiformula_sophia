from setuptools import Extension, setup
from Cython.Distutils import build_ext
import os

name = 'concept_feasibility'
setup(
    name=name,
    version='6.26.0',
    packages=[name],
    cmdclass={'build_ext': build_ext},
    options={'build_ext': {'cython_c_in_temp': True, 'cython_directives': {'language_level': 3}}},
    ext_modules=[Extension(name + '.' + module, [name + '/' + module + '.pyx'],
                   extra_compile_args=['/O2', '/fp:strict', '/openmp'] if os.name == 'nt' else ['-O3', '-ffp-contract=off', '-fopenmp'],
                   extra_link_args=[] if os.name == 'nt' else ['-fopenmp'])
        for module in ['_motion', '_tread', '_risk']],

    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml']),
    ],
    package_data={name: ['response61515.json', 'reverse_model617.json', 'reverse_response.json']},
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Sophia AI Formula Team',
    maintainer_email='maintainers@example.invalid',
    description='V6.26 tyre, response and obstacle clearance checks.',
    license='AGPL-3.0-only',
)
