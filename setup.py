from setuptools import setup

setup(
    name='I2cKeypad',
    version='1.0.0',
    description='Raspberry Pi software library for reading I²C keypad using MCP23008 chip',
    long_description=open('README.md').read(),
    url='https://github.com/dcityorg/i2c-keypad-raspberrypi',
    author='Gary Muhonen',
    author_email='gary@dcity.org',
    license='MIT',
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'Topic :: System :: Hardware',
        'Topic :: Software Development :: Libraries :: Python Modules',
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.2',
        'Programming Language :: Python :: 3.3',
        'Programming Language :: Python :: 3.4',
        'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 3.6',
    ],
    keywords='RPi MCP23008 I2C Keypad',
    py_modules=['I2cKeypad'],
    install_requires=['smbus'],    
)