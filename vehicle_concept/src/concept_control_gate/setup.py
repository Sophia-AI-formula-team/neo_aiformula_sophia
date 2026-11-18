from setuptools import setup

PACKAGE = 'concept_control_gate'
setup(name=PACKAGE, version='6.27.4', packages=[PACKAGE],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
                  ('share/' + PACKAGE, ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Sophia AI Formula', maintainer_email='maintainers@example.invalid',
      description='Manual and autonomous command arbitration and final CAN interlock.', license='MIT',
      entry_points={'console_scripts': [PACKAGE + ' = ' + PACKAGE + '.node:main']})
