# -*- coding: utf-8 -*-

from setuptools import setup, find_packages


with open('README.rst') as f:
    readme = f.read()

with open('LICENSE') as f:
    lic = f.read()

setup(
    name='rosagent',
    version='0.0.1',
    description='Module for creating one ROS agent in a multi agent scenario',
    long_description=readme,
    author='Sriram Krishnaswamy',
    author_email='krishnaswamy.14@osu.edu',
    license=lic,
    packages=find_packages(exclude=('data', 'plots', 'tests', 'docs'))
)
