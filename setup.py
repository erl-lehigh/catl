from setuptools import setup, find_namespace_packages
import sys

setup(
    name='catl',
    version='1.0.0',
    url='https://github.com/wasserfeder/catl',
    license='MIT',
    maintainer='Cristian-Ioan Vasile',
    maintainer_email='crv519@lehigh.edu',
    description='A library for manipulating Capability Temporal Logic Formulae',
    packages=find_namespace_packages(include=['catl', 'catl.*']),
    install_requires=[
        "python-stl"
    ]
)
