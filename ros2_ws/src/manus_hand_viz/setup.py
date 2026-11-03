from glob import glob
from setuptools import setup
package_name = 'manus_hand_viz'
setup(name=package_name, version='0.1.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml']),
                  ('share/' + package_name + '/launch', glob('launch/*.py')),
                  ('share/' + package_name + '/config', glob('config/*.rviz'))],
      install_requires=['setuptools'], zip_safe=True, maintainer='Local developer',
      maintainer_email='local@example.invalid', description='MANUS skeleton visualization', license='Apache-2.0',
      entry_points={'console_scripts': ['hand_viz = manus_hand_viz.visualizer:main',
                                       'mock_gloves = manus_hand_viz.mock_gloves:main']})
