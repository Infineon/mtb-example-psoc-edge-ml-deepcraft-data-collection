# PSOC&trade; Edge MCU: Machine Learning – DEEPCRAFT&trade; data collection

This code example demonstrates how to collect data by implementing the [DEEPCRAFT&trade; streaming protocol v2](https://developer.imagimob.com/getting-started/tensor-streaming-protocol/registering-sensors-using-protocolv2) for PSOC&trade; Edge E84 MCU boards, allowing the streaming of sensor data and other information from the board into [DEEPCRAFT&trade; Studio](https://www.imagimob.com/products) over USB for development and testing of Edge AI models.

The code example supports collecting data from different sources utilizing the Cortex&reg;-M55 (CM55) CPU, data can be collected from an inertial measurement unit (IMU - BMI270), magnetometer (BMM350), XENSIV&trade; digital MEMS microphones (IM73D122V01) using pulse-density modulation (PDM) to pulse-code modulation (PCM), XENSIV&trade; digital barometric air pressure sensor (DPS368), digital humidity and temperature sensor (SHT40T), and XENSIV&trade; 60 GHz radar sensor (BGT60TR13C). The data is transmitted using USB to [DEEPCRAFT&trade; Studio](https://www.imagimob.com/products). The data can then be used in [DEEPCRAFT&trade; starter projects](https://developer.imagimob.com/getting-started/starter-project) or to generate a new model. 

> **Note:** The humidity sensor is currently disabled by default, due to inaccurate readings caused by a driver issue. This will be addressed in the next revision of the code example. 

To deploy a DEEPCRAFT&trade; model onto an Infineon embedded device, see the following code examples:

- [mtb-example-psoc-edge-ml-deepcraft-deploy-audio](https://github.com/Infineon/mtb-example-psoc-edge-ml-deepcraft-deploy-audio)
- [mtb-example-psoc-edge-ml-deepcraft-deploy-motion](https://github.com/Infineon/mtb-example-psoc-edge-ml-deepcraft-deploy-motion)
- [mtb-example-psoc-edge-ml-deepcraft-deploy-radar](https://github.com/Infineon/mtb-example-psoc-edge-ml-deepcraft-deploy-radar)

This code example has a three project structure: CM33 secure, CM33 non-secure, and CM55 projects. All three projects are programmed to the external QSPI flash and executed in Execute in Place (XIP) mode. Extended boot launches the CM33 secure project from a fixed location in the external flash, which then configures the protection settings and launches the CM33 non-secure application. Additionally, CM33 non-secure application enables CM55 CPU and launches the CM55 application.

> **Note:** DEEPCRAFT&trade; Studio currently support only the Windows system.

[View this README on GitHub.](https://github.com/Infineon/mtb-example-psoc-edge-ml-deepcraft-data-collection)

[Provide feedback on this code example.](https://cypress.co1.qualtrics.com/jfe/form/SV_1NTns53sK2yiljn?Q_EED=eyJVbmlxdWUgRG9jIElkIjoiQ0UyMzkyNDciLCJTcGVjIE51bWJlciI6IjAwMi0zOTI0NyIsIkRvYyBUaXRsZSI6IlBTT0MmdHJhZGU7IEVkZ2UgTUNVOiBNYWNoaW5lIExlYXJuaW5nIOKAkyBERUVQQ1JBRlQmdHJhZGU7IGRhdGEgY29sbGVjdGlvbiIsInJpZCI6Im5pY2hvbGFzIHNoYXJwIiwiRG9jIHZlcnNpb24iOiIyLjAuMCIsIkRvYyBMYW5ndWFnZSI6IkVuZ2xpc2giLCJEb2MgRGl2aXNpb24iOiJNQ0QiLCJEb2MgQlUiOiJJQ1ciLCJEb2MgRmFtaWx5IjoiUFNPQyJ9)

See the [Design and implementation](docs/design_and_implementation.md) for the functional description of this code example.


## Requirements

- [ModusToolbox&trade;](https://www.infineon.com/modustoolbox) v3.6 or later (tested with v3.6)
- Board support package (BSP) minimum required version: 1.0.0
- Programming language: C
- Associated parts: All [PSOC&trade; Edge MCU](https://www.infineon.com/products/microcontroller/32-bit-psoc-arm-cortex/32-bit-psoc-edge-arm) parts


## Supported toolchains (make variable 'TOOLCHAIN')

- GNU Arm&reg; Embedded Compiler v14.2.1 (`GCC_ARM`) – Default value of `TOOLCHAIN`
- Arm&reg; Compiler v6.22 (`ARM`)
- IAR C/C++ Compiler v9.50.2 (`IAR`)
- LLVM Embedded Toolchain for Arm&reg; v19.1.5 (`LLVM_ARM`)


## Supported kits (make variable 'TARGET')

- [PSOC&trade; Edge E84 AI Kit](https://www.infineon.com/KIT_PSE84_AI) (`KIT_PSE84_AI`) – Default value of `TARGET`
- [PSOC&trade; Edge E84 Evaluation Kit](https://www.infineon.com/KIT_PSE84_EVAL) (`KIT_PSE84_EVAL_EPC2`)
- [PSOC&trade; Edge E84 Evaluation Kit](https://www.infineon.com/KIT_PSE84_EVAL) (`KIT_PSE84_EVAL_EPC4`)


## Hardware setup

This example uses the board's default configuration. See the kit user guide to ensure that the board is configured correctly.

Ensure the following jumper and pin configuration on board.
- BOOT SW must be in the HIGH/ON position
- J20 and J21 must be in the tristate/not connected (NC) position

> **Note:** This hardware setup is not required for the `KIT_PSE84_AI` kit.


### USB connection to DEEPCRAFT&trade; Studio for data transfer

Connect USB-C cable to USB connector as follows:

 Kit  |  USB connector
:-------- | :-------------
`KIT_PSE84_AI` | J2
`KIT_PSE84_EVAL_EPC2` | J30
`KIT_PSE84_EVAL_EPC4` | J30

<br>


## Software setup

See the [ModusToolbox&trade; tools package installation guide](https://www.infineon.com/ModusToolboxInstallguide) for information about installing and configuring the tools package.

- Install [DEEPCRAFT&trade; Studio](https://www.imagimob.com/studio) if not already installed
- Install a terminal emulator if you do not have one. Instructions in this document use [Tera Term](https://teratermproject.github.io/index-en.html)


## Operation

See [Using the code example](docs/using_the_code_example.md) for instructions on creating a project, opening it in various supported IDEs, and performing tasks, such as building, programming, and debugging the application within the respective IDEs.

1. Connect the board to your PC using the provided USB cable through the KitProg3 USB connector

2. Open a terminal program and select the KitProg3 COM port. Set the serial port parameters to 8N1 and 115200 baud

3. After programming, the application starts automatically. Confirm that "Firmware Debug Console" is displayed on the UART terminal

   **Figure 1. Terminal output on program startup**

   ![](images/firmware-startup-console.png)
   
4. Connect the USB-device port on the board to the host PC using a USB cable, which enables a serial port for sensor data collection
   
   **Figure 2. Terminal output after connecting USB-device port to host PC**

   ![](images/firmware-usb-console.png)

5. Ensure that the power LED1 turns ON, indicating the board is powered


### Data collection with DEEPCRAFT&trade; Studio
 
**Table 1. Sensors available on kit**

Kit  |  Available sensors
:-------- | :-------------
`KIT_PSE84_AI` | Inertial measurement unit (BMI270), magnetometer (BMM350), XENSIV&trade; digital MEMS microphones (IM73D122V01), XENSIV&trade; digital barometric air pressure sensor (DPS368), digital humidity and temperature sensor (SHT40T), and XENSIV&trade; 60 GHz RADAR sensor (BGT60TR13C) 
`KIT_PSE84_EVAL_EPC2` | Inertial measurement unit (BMI270), magnetometer (BMM350), XENSIV&trade; digital MEMS microphones (IM73D122V01)
`KIT_PSE84_EVAL_EPC4` | Inertial measurement unit (BMI270), magnetometer (BMM350), XENSIV&trade; digital MEMS microphones (IM73D122V01)

> **Note:** <br>
      1. For step-by-step instructions on collecting data for PSOC&trade; Edge Evaluation Kits, see [DEEPCRAFT&trade; Studio](https://developer.imagimob.com/data-preparation/data-collection/data-collection-using-new-streaming-firmware). <br>
      2. The website mentiones PSOC&trade; 6 AI kit. However, the same instructions hold good for PSOC&trade; Edge Evaluation kit as well.

> **Note:**
    To ensure the optimal performance and the data integrity when working with sensors, consider the following guidelines:<br>
    <b>1. Enabling Sensors:</b> When enabling all sensors simultaneously, it is recommended to initialize them with their default frequency or minimum frequency. This helps prevent potential data loss due to excessive data streams. By starting with a minimal data frequency, you can ensure that the DEEPCRAFT&trade; Studio runs smoothly.<br>
    <b>2. Sensor Readings:</b> 
    The sensor readings are relative to the board temperature and not relative to the ambient temperature. Since KIT_PSE84_AI has a smaller form-factor, it generates more heat on the board compared to KIT_PSE84_EVAL_EPC2 / KIT_PSE84_EVAL_EPC4. Therefore, you may notice a slight difference in the sensor readings between these kits. Care must be taken when porting the models the across the boards to compensate any deviation in the sensor readings.
    
## Status LED codes


### Normal operation

The status LED1 indicates that normal operation by flashing each time data is sent. If data is being sent at a high frequency, the LED may appear dimmed due to the rapid flashing.


### Error indication

In the event of an error, the LED1 will flash a specific error code. Each flash pattern consists of short and long flashes:
- A short flash (noted as `.`) lasts 100 milliseconds
- A long flash (noted as `-`) lasts 300 milliseconds

The sequence repeats after a delay of 3 seconds.


### Error code patterns

**Table 2. Error code patterns**

Pattern  |  Error description
:-------- | :-------------
`-    (repeated without pause)`| The wrong USB port is connected or failed to establish USB connection. Check the USB cable
`-.   (pause, repeated)`| Unspecified error
`.-   (pause, repeated)`| UART error. Data communication error
`-..  (pause, repeated)`| I2C bus error
`.-.  (pause, repeated)`| Memory error
`--.  (pause, repeated)`| Debug console error
`..-  (pause, repeated)`| Clock error
`-.-  (pause, repeated)`| SPI bus error
`.--  (pause, repeated)`| Watchdog error
`-... (pause, repeated)`| Microphone error
`.-.. (pause, repeated)`| IMU error (problem with the BMI270 chip)
`--.. (pause, repeated)`| Magnetometer error (problem with the BMM350 chip)
`..-. (pause, repeated)`| Digital pressure/temperature sensor error (problem with the DPS368 chip)
`-.-. (pause, repeated)`| RADAR error (problem with the BGT60TRxx chip)
`.--. (pause, repeated)`| Humidity sensor error (problem with the SHT40T chip)
`-..- (pause, repeated)`| I3C bus error


## Related resources

Resources  | Links
-----------|----------------------------------
Application notes  | [AN235935](https://www.infineon.com/AN235935) – Getting started with PSOC&trade; Edge E8 MCU on ModusToolbox&trade; software
Code examples  | [Using ModusToolbox&trade;](https://github.com/Infineon/Code-Examples-for-ModusToolbox-Software) on GitHub
Device documentation | [PSOC&trade; Edge MCU datasheets](https://www.infineon.com/products/microcontroller/32-bit-psoc-arm-cortex/32-bit-psoc-edge-arm#documents) <br> [PSOC&trade; Edge MCU reference manuals](https://www.infineon.com/products/microcontroller/32-bit-psoc-arm-cortex/32-bit-psoc-edge-arm#documents)
Development kits | Select your kits from the [Evaluation board finder](https://www.infineon.com/cms/en/design-support/finder-selection-tools/product-finder/evaluation-board)
Libraries  | [mtb-dsl-pse8xxgp](https://github.com/Infineon/mtb-dsl-pse8xxgp) – Device support library for PSE8XXGP <br> [retarget-io](https://github.com/Infineon/retarget-io) – Utility library to retarget STDIO messages to a UART port
Tools  | [ModusToolbox&trade;](https://www.infineon.com/modustoolbox) – ModusToolbox&trade; software is a collection of easy-to-use libraries and tools enabling rapid development with Infineon MCUs for applications ranging from wireless and cloud-connected systems, edge AI/ML, embedded sense and control, to wired USB connectivity using PSOC&trade; Industrial/IoT MCUs, AIROC&trade; Wi-Fi and Bluetooth&reg; connectivity devices, XMC&trade; Industrial MCUs, and EZ-USB&trade;/EZ-PD&trade; wired connectivity controllers. ModusToolbox&trade; incorporates a comprehensive set of BSPs, HAL, libraries, configuration tools, and provides support for industry-standard IDEs to fast-track your embedded application development

<br>


## Other resources

Infineon provides a wealth of data at [www.infineon.com](https://www.infineon.com) to help you select the right device, and quickly and effectively integrate it into your design.


## Document history

Document title: *CE239247* – *PSOC&trade; Edge MCU: Machine Learning – DEEPCRAFT&trade; data collection*

 Version | Description of change
 ------- | ---------------------
 1.x.0   | New code example <br> Early access release
 2.0.0   | GitHub release 
<br>


All referenced product or service names and trademarks are the property of their respective owners.

The Bluetooth&reg; word mark and logos are registered trademarks owned by Bluetooth SIG, Inc., and any use of such marks by Infineon is under license.

PSOC&trade;, formerly known as PSoC&trade;, is a trademark of Infineon Technologies. Any references to PSoC&trade; in this document or others shall be deemed to refer to PSOC&trade;.

---------------------------------------------------------

© Cypress Semiconductor Corporation, 2023-2025. This document is the property of Cypress Semiconductor Corporation, an Infineon Technologies company, and its affiliates ("Cypress").  This document, including any software or firmware included or referenced in this document ("Software"), is owned by Cypress under the intellectual property laws and treaties of the United States and other countries worldwide.  Cypress reserves all rights under such laws and treaties and does not, except as specifically stated in this paragraph, grant any license under its patents, copyrights, trademarks, or other intellectual property rights.  If the Software is not accompanied by a license agreement and you do not otherwise have a written agreement with Cypress governing the use of the Software, then Cypress hereby grants you a personal, non-exclusive, nontransferable license (without the right to sublicense) (1) under its copyright rights in the Software (a) for Software provided in source code form, to modify and reproduce the Software solely for use with Cypress hardware products, only internally within your organization, and (b) to distribute the Software in binary code form externally to end users (either directly or indirectly through resellers and distributors), solely for use on Cypress hardware product units, and (2) under those claims of Cypress's patents that are infringed by the Software (as provided by Cypress, unmodified) to make, use, distribute, and import the Software solely for use with Cypress hardware products.  Any other use, reproduction, modification, translation, or compilation of the Software is prohibited.
<br>
TO THE EXTENT PERMITTED BY APPLICABLE LAW, CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH REGARD TO THIS DOCUMENT OR ANY SOFTWARE OR ACCOMPANYING HARDWARE, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  No computing device can be absolutely secure.  Therefore, despite security measures implemented in Cypress hardware or software products, Cypress shall have no liability arising out of any security breach, such as unauthorized access to or use of a Cypress product. CYPRESS DOES NOT REPRESENT, WARRANT, OR GUARANTEE THAT CYPRESS PRODUCTS, OR SYSTEMS CREATED USING CYPRESS PRODUCTS, WILL BE FREE FROM CORRUPTION, ATTACK, VIRUSES, INTERFERENCE, HACKING, DATA LOSS OR THEFT, OR OTHER SECURITY INTRUSION (collectively, "Security Breach").  Cypress disclaims any liability relating to any Security Breach, and you shall and hereby do release Cypress from any claim, damage, or other liability arising from any Security Breach.  In addition, the products described in these materials may contain design defects or errors known as errata which may cause the product to deviate from published specifications. To the extent permitted by applicable law, Cypress reserves the right to make changes to this document without further notice. Cypress does not assume any liability arising out of the application or use of any product or circuit described in this document. Any information provided in this document, including any sample design information or programming code, is provided only for reference purposes.  It is the responsibility of the user of this document to properly design, program, and test the functionality and safety of any application made of this information and any resulting product.  "High-Risk Device" means any device or system whose failure could cause personal injury, death, or property damage.  Examples of High-Risk Devices are weapons, nuclear installations, surgical implants, and other medical devices.  "Critical Component" means any component of a High-Risk Device whose failure to perform can be reasonably expected to cause, directly or indirectly, the failure of the High-Risk Device, or to affect its safety or effectiveness.  Cypress is not liable, in whole or in part, and you shall and hereby do release Cypress from any claim, damage, or other liability arising from any use of a Cypress product as a Critical Component in a High-Risk Device. You shall indemnify and hold Cypress, including its affiliates, and its directors, officers, employees, agents, distributors, and assigns harmless from and against all claims, costs, damages, and expenses, arising out of any claim, including claims for product liability, personal injury or death, or property damage arising from any use of a Cypress product as a Critical Component in a High-Risk Device. Cypress products are not intended or authorized for use as a Critical Component in any High-Risk Device except to the limited extent that (i) Cypress's published data sheet for the product explicitly states Cypress has qualified the product for use in a specific High-Risk Device, or (ii) Cypress has given you advance written authorization to use the product as a Critical Component in the specific High-Risk Device and you have signed a separate indemnification agreement.
<br>
Cypress, the Cypress logo, and combinations thereof, ModusToolbox, PSoC, CAPSENSE, EZ-USB, F-RAM, and TRAVEO are trademarks or registered trademarks of Cypress or a subsidiary of Cypress in the United States or in other countries. For a more complete list of Cypress trademarks, visit www.infineon.com. Other names and brands may be claimed as property of their respective owners.