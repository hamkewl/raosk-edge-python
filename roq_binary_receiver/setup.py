from setuptools import find_packages
from setuptools import setup
import os
from glob import glob

package_name = 'roq_binary_receiver'

setup(
	name = package_name,
	version = '0.7.1',
	packages = find_packages(exclude = ['test']),
	data_files = [
		('share/ament_index/resource_index/packages',
			['resource/' + package_name]),
		('share/' + package_name, ['package.xml']),
		(os.path.join('share', package_name, 'launch'), glob('*.launch.py'))
	],
	install_requires = ['setuptools'],
	zip_safe = True,
	author = 'Koki Nagahama',
	author_email = 'hamstick.devel@gmail.com',
	maintainer = 'dolyLab',
	maintainer_email = 'ubuntu@todo.todo',
	keywords = ['ROS'],
	classifiers = [
		'Intended Audience :: Developers',
		'License :: MIT License',
		'Programming Language :: Python',
		'Topic :: Software Development',
	],
	description = (
		'ROQ Offloader System Modeler package'
	),
	license = 'MIT License',
	tests_require = ['pytest'],
	entry_points = {
		'console_scripts': [
			'bin_start = src.BinaryReceiver:main',
			'bin_tester = src.tester:main',
		],
	},
)
