from setuptools import setup

name = 'concept_feasibility'
setup(
    name=name,
    version='6.26.0',
    packages=[name],
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
