#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
    config.py script
    =====================

    The purpose of this simple script is only to define some configuration
    variables. The script is imported into readGateway.py script.
    Rename this file to config.py after customizing it.
"""


import logging

# Web server address and password for posting data
cfgPostPwd = 'password'
cfgPostUrl = 'http://www.example.com'

# Serial port config
cfgSerialName = "/dev/ttyAMA0" # default raspbian uart name
cfgSerialSpeed = 115200 # default speed

# Files
cfgBufferFile = '/path/to/buffer_file'
cfgLoggerFile = '/path/to/log_file'

# Logger config
cfgLogLevel = logging.DEBUG

__author__ = "Lucas Glénat"
__copyright__ = "Copyright 2017, whmnet project"
__credits__ = ["Lucas Glénat"]
__license__ = "GPLv3"
__version__ = "1.0.0"
__maintainer__ = "Lucas Glénat"
__email__ = "lucasglenat@hotmail.com"
__status__ = "Production"

