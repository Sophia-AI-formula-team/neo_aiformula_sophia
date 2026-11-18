from setuptools import setup

name = 'concept_ranker'
setup(
    name=name,
    version='6.26.0',
    packages=[name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml']),
    ],
    package_data={name: ['model-h12.json']},
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Sophia AI Formula Team',
    maintainer_email='maintainers@example.invalid',
    description='Frozen V6.26 H12 trajectory ranker and trained parameters.',
    license='AGPL-3.0-only',
)
