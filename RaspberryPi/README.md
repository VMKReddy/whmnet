## Raspberry Pi software

### Description

A Raspberry Pi model B+ running Raspbian Jessie is used to post the measurements on the web.
The python script readGateway.py is used to communicate with the Gateway.

The Gateway reports measurements coming from wireless sensors to the Raspberry Pi through the serial port (pins 8 & 10 of the Raspberry Pi model B header).

Communication is unidirectionnal (Gateway --> Raspberry Pi).

After receiving raw data from the Gateway, the script extracts measurements (temperature, pressure, RH) and posts them on a dedicated server on the web.

### Dependencies

The script is run with python 2.7.6.
The following external packages are needed:
- [requests](http://docs.python-requests.org/en/master/) (to perform http requests)
- [crcmod](https://pypi.python.org/pypi/crcmod) (for custom CRC calculations)

You can install these two packages with pip for instance.

### Configuration

Edit the configTemplate.py script to match your configuration (logging level, webserver parameters, ...). Then rename it to config.py or modify readGateway.py to import the right script. 

### Monitoring script execution

#### Monitoring script

The shell script [readGatewayMonitor](readGatewayMonitor) is used to restart the python script if it crashes.
Modify the paths to match your configuration if you want to use it.

#### Crontab

A crontab entry is used to start the readGatewayMonitor script at boot time.

Use ```sudo crontab -e``` to edit the crontab and then add the following entry (change path to math your installation):
```
@reboot /home/pi/templog_gateway/readGatewayMonitor
```

#### Logrotate

The python script logs data to a log file each time a message is received from the Gateway, or only on errors, depending on the logger configuration (see script).

Logrotate tool is used to keep logfile small and delete oldest logs automatically. It is not shipped with Raspbian by default so you may need to install it manually with ```apt-get install```.

Configuration of logrotate for our use is in [logrotate.d](logrotate.d/).
Simply copy [templog](logrotate.d/templog) to ```/etc/logrotate.d/``` on your Pi, and edit it so the paths match your installation.

### Additionnal considerations

In the actual setup, the Raspberry Pi is left unattented for several months. The hardware watchdog has been configured on the Pi to reboot the system in case of a major failure or undetected bugs remaining in the python script.