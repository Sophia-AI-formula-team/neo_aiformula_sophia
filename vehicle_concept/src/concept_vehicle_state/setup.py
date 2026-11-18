from setuptools import setup

PACKAGE = 'concept_vehicle_state'
setup(name=PACKAGE, version='0.1.0', packages=[PACKAGE],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
                  ('share/' + PACKAGE, ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Sophia AI Formula', maintainer_email='maintainers@example.invalid',
      description='Stamped VectorNav and audited CAN feedback for Concept.', license='MIT',
      entry_points={'console_scripts': [PACKAGE + ' = ' + PACKAGE + '.node:main']})
