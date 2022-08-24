from setuptools import setup
from glob import glob

package_name = 'process_sarif'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/config", glob('config/*')),
        ('share/' + package_name + "/resources", glob('resources/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eli Benevedes',
    maintainer_email='ebenevedes@blueorigin.com',
    description='Python libraries and scripts to parse and process SARIF from static analyzers.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sarif_load_test = process_sarif.sarif_load_test:main',
            'conformance = process_sarif.conformance:main',
            'duplicates = process_sarif.duplicates:main',
            'visualize = process_sarif.visualize:main'
        ],
    },
)
