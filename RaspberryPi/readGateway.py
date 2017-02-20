#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
    readGateway.py script
    =====================

    This script is used in the whmnet project to receive data from the
    wireless network gateway and send it to a custom server on the web.
    This script is run on a Raspberry Pi, connected to the Gateway 
    through the UART serial port on the Pi GPIO header.
"""
# built-in modules
import serial
import binascii
import struct
import datetime
import logging
import logging.handlers

# third-party modules
import crcmod
import requests

# Import configuration variables
from config import *

# Constants
FILE_TYPE_GW_LOG        = 0
FILE_TYPE_SENSOR_LOG    = 1
FILE_TYPE_LEGACY_DATA   = 2
FILE_TYPE_DATA          = 3

GW_TYPE_REMOTE_DATA   = 0
GW_TYPE_LOCAL_DATA    = 1
GW_TYPE_LOG           = 2

SENS_TYPE_DATA        = 0
SENS_TYPE_LOG         = 1
SENS_TYPE_DATA_LEGACY = 2

logger = logging.getLogger()

def main():

    # Configure logger
    wfh = logging.handlers.WatchedFileHandler(cfgLoggerFile) # log file
    formatter = logging.Formatter('%(asctime)s - %(levelname)s: %(message)s') # log header
    wfh.setFormatter(formatter)
    logger.addHandler(wfh)
    logger.setLevel(cfgLogLevel) # set level according to your needs

    # Configure CRC with polynome, reversing etc.
    crc32_func = crcmod.mkCrcFun(0x104C11DB7, initCrc=0x0, rev=True, xorOut=0xFFFFFFFF)

    # Open serial port to communicate with gateway
    try:
        logger.info('Opening serial port.')
        port = serial.Serial(cfgSerialName, cfgSerialSpeed, timeout=None)
    except serial.SerialException:
        logger.critical('Serial port unavailable')
        raise
    else:
        logger.info('Serial port successfully opened.')

    # main loop
    while True:

        # search for sync byte 0xAA
        rcv = port.read(1)
        if rcv == b'\xAA':
            logger.debug('Sync word received.')

            # Get timestamp
            timedata = datetime.datetime.now()

            # Proceed with message
            # First byte is length of UART frame
            # use ord() because data is written in binary format on UART by STM32 (not char)
            length = ord(port.read(1)) 
            logger.debug('Size of rcvd frame: ' + str(length))

            # We can have length = 0 if rx uart buffer is full (in case python script
            # is started after sensor gateway)
            if length > 0:

                # Then read the entire frame
                msg = port.read(length)
                logger.debug('Rx frame: ' + binascii.hexlify(msg))

                # Unpack the CRC from the 4 last bytes of frame
                try:
                    rxcrc = struct.unpack('<I', msg[length-4:length])[0]
                except struct.error:
                    logger.exception('CRC struct error.')
                else:
                    logger.debug('Rx CRC: ' + str(rxcrc) + ' - ' + hex(rxcrc))

                # Compute CRC on frame data (except sync and length bytes)
                compcrc = crc32_func(msg[0:length-4])
                logger.debug('Calculated CRC: ' + str(compcrc) + ' - ' + hex(int(compcrc)))

                # Compare rcvd CRC and calculated CRC
                if rxcrc != int(compcrc):
                    # A problem occured during UART transmission
                    logger.info('CRC ERROR.')
                else:
                    # Get message type from Gateway
                    gwMsgType = ord(msg[0]);

                    # Remote data is data coming from wireless sensors
                    if gwMsgType == GW_TYPE_REMOTE_DATA:
                        
                        # get sensor id and msg type
                        sensMsgType = ord(msg[2]) >> 4
                        sensorId = ord(msg[2]) & 0xf
                       
                        # get RSSI (can be negative)
                        rssi = ord(msg[length-6])
                        if rssi > 127:
                            rssi = (256 - rssi) * (-1)

                        # Print sensor ID
                        logger.info('Sensor ID: ' + str(sensorId) + ' - RSSI: ' + str(rssi))

                        # log/error message from sensor
                        if sensMsgType == SENS_TYPE_LOG:
                            
                            # print and process log message
                            log_msg = binascii.hexlify(msg[3:6])
                            logger.info('Log message: ' + log_msg)
                            
                            # Write msg to file
                            writeSensorLog(sensorId, timedata, log_msg, rssi)
                            # Post msg on server
                            postMeasFromFile()

                        # measurement message from V1 sensor (not used anymore)
                        elif sensMsgType == SENS_TYPE_DATA_LEGACY:

                            # Extract and print temperature #
                            temperature = computeTemp(msg[3:5])
                            logger.debug('Temperature: ' + str(temperature))
    
                            # Write measurement to file
                            writeLegacyData(sensorId, timedata, temperature, rssi)

                            # Post measurement on server
                            postMeasFromFile()
                        
                        # measurement message from V2 sensor
                        elif sensMsgType == SENS_TYPE_DATA:
                            
                            #Extract data from message
                            data = computeData(msg[3:8])
                            logger.info('Temp: ' + '{:.2f}'.format(data['temp']))
                            logger.info('Hum: ' + '{:.2f}'.format(data['hum']))
                            logger.info('Pres: ' + str(data['pres']))

                            # Write data to file
                            writeData(sensorId, timedata, data['temp'], data['hum'], data['pres'], rssi)

                            # Post on server
                            postMeasFromFile()

                        else:
                            logger.warning('UNKNOWN SENSOR MSG TYPE.') 
                    
		            # log message from gateway itself
                    elif gwMsgType == GW_TYPE_LOG:
                        # Print log message
                        logger.info('Gateway log: ' + str(ord(msg[1])))
                        # Write msg to file
                        writeGatewayLog(timedata, ord(msg[1]))
                        # Post msg on server
                        postMeasFromFile()
                   
                    else:
                        logger.warning('UNKNOWN GATEWAY MSG TYPE.') 

            else:
                logger.error('Gateway msg is of length 0.')



# The 4 functions below save posts to the CSV buffer file before they are sent to the server
def writeLegacyData(id, timedata, temp, rssi):
    with open(cfgBufferFile, 'a') as f:
        f.write(str(id) + ',' + str(FILE_TYPE_LEGACY_DATA) + ',' +
                timedata.strftime("%Y-%m-%d %H:%M:%S") + ',' +
                str(temp) + ',' + str(rssi))
        f.write('\n')

def writeData(id, timedata, temp, hum, pres, rssi):
    with open(cfgBufferFile, 'a') as f:
        f.write(str(id) + ',' + str(FILE_TYPE_DATA) + ',' + timedata.strftime("%Y-%m-%d %H:%M:%S") + ',' + 
                '{:.2f}'.format(temp) + ',' + '{:.2f}'.format(hum) + ',' + str(pres) + ',' + str(rssi))
        f.write('\n')

def writeSensorLog(id, timedata, log, rssi):
    with open(cfgBufferFile, 'a') as f:
        f.write(str(id) + ',' + str(FILE_TYPE_SENSOR_LOG) + ',' + timedata.strftime("%Y-%m-%d %H:%M:%S") +
                ',' + str(log) + ',' + str(rssi))
        f.write('\n')

def writeGatewayLog(timedata, log):
    with open(cfgBufferFile, 'a') as f:
        f.write('255' + ',' + str(FILE_TYPE_GW_LOG) + ',' + timedata.strftime("%Y-%m-%d %H:%M:%S") + ',' + str(log))
        f.write('\n')

# Function to compute temperature from V1 sensor message (not for V2 sensor)
def computeTemp(tempString):
    # unpack data - big endian 16 bit
    try:
        temp = struct.unpack('>h', tempString)[0]
    except struct.error:
        logger.exception('Temperature struct error.')
        return -99
    else:
        # Convert code to actual temperature
        temp = temp * 0.0625 # 1 LSB = 0.0625 °C
        return temp

# Function to extract temperature, RH and pressure data from sensor V2 message
# See sensor V2 STM32L0 firmware source to retrieve message structure
def computeData(dataStr):

    # Initialize dictionnary
    data = {}

    #fixme: return errors and check for error in calling function

    # Little endian 24-bit padded to 32
    try:
        sht = struct.unpack('<I', dataStr[0:3] + '\x00')
    except struct.error:
        logger.exception('SHT21 data decoding struct error.')
        return -99
    else:
        data['temp'] = -46.85 + 175.72 * ((sht[0] & 0x7FF) << 5) / pow(2,16)
        data['hum']  = -6.0 + 125.0 * ((sht[0] >> 11) << 5) / pow(2,16)


    # Little endian 16-bit
    try:
        ms56 = struct.unpack('<H', dataStr[3:5])
    except struct.error:
        logger.exception('MS5637 data decoding struct error.')
        return -99
    else:
        data['pres'] = ms56[0] + 85000

    return data


# Function that reads the CSV buffer file line by line and post the data to the
# webserver if it is reachable on the internet
def postMeasFromFile():
    nbLinesPosted = 0

    # open backup file in read mode
    with open(cfgBufferFile, 'r') as f:
        # Save all measurements in lines variable
        lines = f.readlines() 

        # Go back to start of file and read it line by line
        f.seek(0, 0)   
        for line in f:
            # Remove newline character
            line = line.rstrip('\n')
            # Split the line to get the items in a list
            s = line.split(',', -1)
            if len(s) != 0:
                # Try to post measurement on server
                type = int(s[1])
                if type == FILE_TYPE_GW_LOG:
                    status = postOnServer(s[0], s[1], s[2], s[3], '', '', '', '')

                elif type == FILE_TYPE_SENSOR_LOG:
                    status = postOnServer(s[0], s[1], s[2], s[3], '', '', '', s[4])
                    
                elif type == FILE_TYPE_LEGACY_DATA:
                    status = postOnServer(s[0], s[1], s[2], '', s[3], '', '', s[4])

                elif type == FILE_TYPE_DATA:
                    status = postOnServer(s[0], s[1], s[2], '', s[3], s[4], s[5], s[6])
                else:
                    logger.error('Unknow type in data file.')
                    status = 200
                
                # If posting is successful, increment variable else break
                if status != 200:
                    break
                else:
                    nbLinesPosted = nbLinesPosted + 1
            else:
                # simply ignore line
                logger.error('Invalid line in file. Skipping.')
                nbLinesPosted = nbLinesPosted + 1

    # Open the file not appending write mode
    with open(cfgBufferFile, 'w') as f:
        # Write all lines that were not posted on server
        f.writelines(lines[nbLinesPosted:])           
        
# Function to post data on websever. Uses the requests package.
def postOnServer(id_s, dataType_s, datetime_s, log_s, temp_s, hum_s, pres_s, rssi_s):
    retval = 0;
    payload = {'id': id_s, 'type': dataType_s, 'time': datetime_s,
            'temp': temp_s, 'hum': hum_s, 'pres': pres_s,
            'log': log_s, 'rssi': rssi_s, 'chk': cfgPostPwd}
    logger.debug(payload)
    try:
        r = requests.post(cfgPostUrl, data=payload, timeout=5)
    except requests.exceptions.ConnectionError:
        logger.exception('Connection error')
    except requests.exceptions.HTTPError:
        logger.exception('HTTP invalid response error.')
    except requests.exceptions.Timeout:
        logger.exception('Connection timeout error.')
    except requests.exceptions.TooManyRedirects:
        logger.exception('Too many redirects.')
    else:
        retval = r.status_code
        logger.debug(r.text)
    return retval

if __name__ == "__main__":
    main()

__author__ = "Lucas Glénat"
__copyright__ = "Copyright 2017, whmnet project"
__credits__ = ["Lucas Glénat"]
__license__ = "GPLv3"
__version__ = "1.0.0"
__maintainer__ = "Lucas Glénat"
__email__ = "lucasglenat@hotmail.com"
__status__ = "Production"

#### END OF FILE ####
