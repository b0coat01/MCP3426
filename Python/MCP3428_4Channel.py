# Distributed with a free-will license.
# Use it any way you want, profit or free, provided it fits in the licenses of its associated works.
# MCP3428 4-Channel I2C ADC via Raspberry Pi 3
# This code is designed to work with the MCP3428_I2CADC I2C Mini Module available from ControlEverything.com.
# https://store.ncd.io/product/4-channel-i2c-0-10v-analog-to-digital-converter-with-i2c-interface/
# https://media.ncd.io/sites/2/20170721134941/MCP3428-8.pdf

import smbus
import time
from time import sleep

# Get I2C bus
try:
  bus = smbus.SMBus(1)
except:
  print("Failed to connect to MCP3428 serial manager bus")

# Setup device and sensor settings for i2c communications
sensor_read_intercept = 3.95
sensor_read_slope = (39.5-sensor_read_intercept)/10

address = 0x6e				#0x6e is the MCP3428 address Retrieved via cmd(sudo i2cdetect -y 1)
ch1 = [0x10,(10/8.925),"ch1"]		#[0x10(00010000) is Continuous conversion mode on Channel-1 w 12-bit Resolution, Ch1 sensor calibration value]
ch2 = [0x30,(10/8.911),"ch2"]		#[0x30(00110000) is Continuous conversion mode on Channel-2 w 12-bit Resolution, Ch2 sensor calibration value]
ch3 = [0x50,(10/8.936),"ch3"]		#[0x50(01010000) is Continuous conversion mode on Channel-3 w 12-bit Resolution, Ch3 sensor calibration value]
ch4 = [0x70,(10/8.921),"ch4"]		#[0x70(01110000) is Continuous conversion mode on Channel-4 w 12-bit Resolution, Ch4 sensor calibration value]

def i2c_read_voltage(addr,ch):
  # Send configuration command
  try:
    bus.write_byte(address, ch[0])
  except:
    print("Failed to connect to MCP3428")
  # Sleep momentarily to allow settle
  sleep(0.25)
  # Read data back from 0x00(0), 2 bytes
  data = bus.read_i2c_block_data(addr, 0x00, 2)
  # Convert the data to 12-bits
  bit_adc = (data[0] & 0x0F) * 256 + data[1]
  if bit_adc > 2047 :
    bit_adc -= 4095
  # Convert and calibrate bit data to voltage
  volt_adc = bit_adc/2047*10*(ch[1]) #2047 represents 10v signal with correction factor
  # Output data to screen
  print("ADC for Voltage ("+ch[2]+") : "+str(volt_adc))
  return (volt_adc)


while(1):
        # Read voltage from ch1 sensor
        sens_volt_adc = i2c_read_voltage(address, ch1)
        # Read voltage from ch2 sensor
        sens_volt_adc = i2c_read_voltage(address, ch2)
        # Read voltage from ch3 sensor
        sens_volt_adc = i2c_read_voltage(address, ch3)
        # Read voltage from ch4 sensor
        sens_volt_adc = i2c_read_voltage(address, ch4)
                
