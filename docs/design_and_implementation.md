[Click here](../README.md) to view the README.

## Design and implementation

This code example allows collecting data from either an IMU or PDM/PCM or BMM350 or DPS368 or SHT40T or radar using the [DEEPCRAFT&trade; streaming protocol v2](https://developer.imagimob.com/getting-started/tensor-streaming-protocol/registering-sensors-using-protocolv2). The application supports transmitting data over USB to the Streaming Protocol.

The design of this application is minimalistic to get started with code examples on PSOC&trade; Edge MCU devices. All PSOC&trade; Edge E84 MCU applications have a dual-CPU three-project structure to develop code for the CM33 and CM55 cores. The CM33 core has two separate projects for the secure processing environment (SPE) and non-secure processing environment (NSPE). A project folder consists of various subfolders, each denoting a specific aspect of the project. The three project folders are as follows:

**Table 1. Application projects**

Project | Description
--------|------------------------
*proj_cm33_s* | Project for CM33 secure processing environment (SPE)
*proj_cm33_ns* | Project for CM33 non-secure processing environment (NSPE)
*proj_cm55* | CM55 project

<br>

In this code example, at device reset, the secure boot process starts from the ROM boot with the secure enclave (SE) as the root of trust (RoT). From the secure enclave, the boot flow is passed on to the system CPU subsystem where the secure CM33 application starts. After all necessary secure configurations, the flow is passed on to the non-secure CM33 application. Resource initialization for this example is performed by this CM33 non-secure project. It configures the system clocks, pins, clock to peripheral connections, and other platform resources. It then enables the CM55 core using the `Cy_SysEnableCM55()` function and the CM33 core is subsequently put to DeepSleep mode.

In the CM33 non-secure application, the clocks and system resources are initialized by the BSP initialization function. The retarget-io middleware is configured to use the debug UART. The debug UART prints a message (as shown in **Figure 1** of [Operation](../README.md#operation) section) on the terminal emulator, the onboard KitProg3 acts the USB-UART bridge to create the virtual COM port.


### IMU capture

- Code example is designed to collect data from a motion sensor (BMI270)

- Data consists of the 3-axis accelerometer and 3-axis gyroscope data obtained from the IMU. The data is read from the IMU over an I2C interface based on the configured data rate and then the data is transmitted over USB

 Supported configurations  |  Ranges / Options
:-------- | :-------------
Frequency | 50 Hz, 100 Hz, 200 Hz, 400 Hz
Accelerometer | 2 G, 4 G, 8 G, 16 G
Gyroscope | 125 dps, 250 dps, 500 dps, 1000 dps, 2000 dps
Mode | Combined, Split, Only Accelerometer, Only Gyroscope

### Microphone capture

- Code example can be configured to collect microphone data from a PDM microphone using the PDM to PCM IP

- Data is sampled at the rate configured in the DEEPCRAFT&trade; Studio and then the data is transmitted over USB

 Supported configurations  |  Ranges / Options
:-------- | :-------------
Gain |  83 dB ,  77 dB ,  71 dB ,  65 dB , 59 dB ,  53 dB ,  47 dB , 41 dB ,  35 dB , 29 dB , 23 dB , 17 dB , 11 dB , 5 dB , -1 dB , -7 dB , -13 dB , -19 dB , -25 dB , -31 dB , -37 dB , -43 dB , -49 dB , -55 dB , -61 dB , -67 dB , -73 dB , -79 dB , -85 dB , -91 dB , -97 dB , -103 dB 
Frequency | 8 kHz, 16 kHz, 22.05 kHz, 44.1 kHz, 48 kHz
Stereo (Mode) | Yes, No

### Magnetometer capture

- Code example is designed to collect data from a magnetometer sensor (BMM350)

- Data consists of the 3-axis magnetic field data obtained from the sensor. The data is read from the magnetometer over an I3C interface based on the configured data rate and then the data is transmitted over USB

 Supported configurations  |  Ranges
:-------- | :-------------
Frequency | 50 Hz, 100 Hz, 200 Hz, 400 Hz

### Pressure capture

- Code example is designed to collect data from a XENSIV&trade; digital barometric air pressure (DPS368) sensor

- Pressure data is read from the sensor over an I2C interface based on the configured data rate and then the data is transmitted over USB

 Supported configurations  |  Ranges
:-------- | :-------------
Frequency | 8 Hz, 16 Hz, 32 Hz, 64 Hz, 128 Hz

### Humidity capture

- Code example is designed to collect data from a digital temperature and humidity sensor (SHT40T)

- Temperature and humidity data are read from the sensor over an I2C interface based on the configured data rate and then the data is transmitted over USB

 Supported configurations  | Options
:-------- | :-------------
Precision | Low, Medium, High
Mode | Combined, Split, Only Humidity, Only Temperature

### Radar capture

- Code example is designed to collect data from a XENSIV&trade; 60 GHz radar (BGT60TR13C) sensor

- Radar data is read from the sensor over an SPI interface based on the use case and then the data is transmitted over USB

 Supported configurations  | Options
:-------- | :-------------
Use case | Default, Gesture, Macro Presence, Micro Presence, Custom

### Files and folders

```
|-- proj.cm55/devices             # Contains the sensor implementation files for this example.
   |- dev_pdm_pcm.c/h             # Implements the PDM to collect data.
   |- dev_bmi270.c/h              # Implements the IMU to collect data.
   |- dev_bmm350.c/h              # Implements the magnetometer to collect data.
   |- dev_dps368.c/h              # Implements the barometric pressure to collect data.
   |- dev_sht4x.c/h               # Implements the humidity and temperature to collect data.
   |- dev_bgt60trxx.c/h           # Implements the radar to collect data.
   |- dev_bgt60trxx_settings.h    # Contains the settings for radar sensor.
   |- pdm_pcm.c/h                 # Contains the implementation for PDM/PCM PDL APIs.
|-- proj.cm55/protocol            # Contains the source code for streaming data over USB.
```

<br>