from setuptools import setup, find_packages
import sys

name_package='trajectory'
setup(
    name=name_package,  # Remplacez par le nom réel de votre package
    version='0.0.1',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    install_requires=[],
    include_package_data=True,
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
    data_files=[('include/trajectory', ['include/trajectory/excel_reader_class.py'])],
)

