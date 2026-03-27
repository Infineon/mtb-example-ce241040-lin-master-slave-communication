# LIN master and slave communication with internal PHY

This code example demonstrates how to communicate with LIN master and LIN slave each other by using internal LIN PHY and two individual boards. It also supports the loop-back mode which is for communication between internal master channel and slave channel for test purpose.

[View this README on GitHub.](https://github.com/Infineon/mtb-example-ce241040-lin-master-slave-communication)

[Provide feedback on this code example.](https://yourvoice.infineon.com/jfe/form/SV_1NTns53sK2yiljn?Q_EED=eyJVbmlxdWUgRG9jIElkIjoiQ0UyNDEwNDAiLCJTcGVjIE51bWJlciI6IjAwMi00MTA0MCIsIkRvYyBUaXRsZSI6IkxJTiBtYXN0ZXIgYW5kIHNsYXZlIGNvbW11bmljYXRpb24gd2l0aCBpbnRlcm5hbCBQSFkiLCJyaWQiOiJzb3JhLnNhdG9AaW5maW5lb24uY29tIiwiRG9jIHZlcnNpb24iOiIxLjQuMCIsIkRvYyBMYW5ndWFnZSI6IkVuZ2xpc2giLCJEb2MgRGl2aXNpb24iOiJNQ0QiLCJEb2MgQlUiOiJBVVRPIiwiRG9jIEZhbWlseSI6IkFVVE8gUFNPQyJ9)


## Requirements

- [ModusToolbox&trade;](https://www.infineon.com/modustoolbox) v3.7 or later (tested with v3.5)
- Board support package (BSP) minimum required version: 3.1.0
- Programming language: C
- Associated parts:  [PSOC&trade; 4 HV MCU](https:///www.infineon.com/cms/en/product/microcontroller/32-bit-psoc-arm-cortex-microcontroller/32-bit-psoc-4-hv-arm-cortex-m0/)


## Supported toolchains (make variable 'TOOLCHAIN')

- GNU Arm&reg; Embedded Compiler v14.2.1 (`GCC_ARM`) – Default value of `TOOLCHAIN`
- Arm&reg; Compiler v6.22 (`ARM`)
- IAR C/C++ Compiler v9.50.2 (`IAR`)


## Supported kits (make variable 'TARGET')

- [PSOC&trade; 4 HVMS-64K Evaluation Kit](https://www.infineon.com/cms/en/product/evaluation-boards/kit_psoc4-hvms-64k_lite/) (`KIT_PSOC4-HVMS-64K_LITE`) – Default value of `TARGET`
- [PSOC&trade; 4 HVMS-128K Evaluation Kit](https://www.infineon.com/cms/en/product/evaluation-boards/kit_psoc4-hvms-128k_lite/) (`KIT_PSOC4-HVMS-128K_LITE`)
- [PSOC&trade; 4 HVPA-144K Evaluation Kit](https://www.infineon.com/cms/en/product/evaluation-boards/kit_psoc4-hvpa-144k_lite/) (`KIT_PSOC4-HVPA-144K_LITE`)
- [PSOC&trade; 4 HVPA-SPM1.0 Evaluation Kit](https://www.infineon.com/design-resources/finder-selection-tools/evaluation-board) (`KIT_PSOC-HVPA-SPM1_LITE`)

## Hardware setup

This example uses the board's default configuration. See the kit user guide to ensure that the board is configured correctly.

For the external commnication by using internal LIN PHY, this example requires two different boards; one for LIN master channel and another for LIN slave channel.

The setup can be skipped if it's loop-back mode.

**Table 1** shows how additional components should be implemented on the master board and the slave board and **Figure 1** shows where the components to be implemented on the circuit schematic.  But it can use only Slave mode in KIT_PSOC-HVPA-144K_LITE.

> **Note:** KIT_PSOC4-HVPA-144K_LITE can only operate as a slave mode. In this case, KIT_PSOC4-HVMS-128K_LITE or KIT_PSOC4-HVMS-64K_LITE can use as a master mode. KIT_PSOC-HVPA-SPM1_LITE cannot operate as a loop-back mode.

**Table 1. LIN master and slave board additional component implementation <br>(For KIT_PSOC4-HVMS-128K_LITE and KIT_PSOC4-HVMS-64K_LITE)**

| Components      | Master board                                   | Slave board             |
| --------------- | ---------------------------------------------- | ----------------------- |
| Jumper          | Implement on J19                               | Implement on J19        |
| D-SUB 9         | Implement on J20                               | Implement on J20        |
| D-SUB 9 cable   | Connect to slave board                         | Connect to master board |
| PMEG4010ET,215  | Implement on D9                                | -                       |
| 1k ohm resistor | Implement on R78                               | -                       |
| 12V adapter     | Connect to DC jack <br> Turn SW4 to ADPTR side | -                       |

<br>

 **Figure 1. Circuit schematic (only KIT_PSOC4-HVMS-128K_LITE or KIT_PSOC4-HVMS-64K_LITE)**

 <img src="./images/lin_circuit_schematic.png" width="600"><br>

**Table 2. LIN master and slave board additional component implementation <br>(only KIT_PSOC-HVPA-SPM1_LITE)**

| Components        | Master board                 | Slave board                   |
| ----------------- | -----------------------------| ----------------------------- |
| VBAT pin          | TP8 (Connect to slave board) | TP8 (Connect to master board) |
| GND pin           | TP10 (Connect to slave board)| TP10 (Connect to master board)|
| LIN pin           | TP9 (Connect to slave board) | TP9 (Connect to master board) |
| PMEG4010ET,215    | Implement on D9              | -                             |
| 0 ohm jumper      | Implement on R148            | -                             |
| 1k ohm resistor   | Implement on R150            | -                             |
| 12V adapter       | Connect to DC jack           | -                             |

<br>

 **Figure 2. Circuit schematic (only KIT_PSOC-HVPA-SPM1_LITE)**

 <img src="./images/lin_spm1_circuit_schematic.png" width="600"><br>

<br>

## Software setup

See the [ModusToolbox&trade; tools package installation guide](https://www.infineon.com/ModusToolboxInstallguide) for information about installing and configuring the tools package.

This example requires no additional software or tools.



## Using the code example

### Create the project

The ModusToolbox&trade; tools package provides the Project Creator as both a GUI tool and a command line tool.

<details><summary><b>Use Project Creator GUI</b></summary>

1. Open the Project Creator GUI tool.

   There are several ways to do this, including launching it from the dashboard or from inside the Eclipse IDE. For more details, see the [Project Creator user guide](https://www.infineon.com/ModusToolboxProjectCreator) (locally available at *{ModusToolbox&trade; install directory}/tools_{version}/project-creator/docs/project-creator.pdf*).

2. On the **Choose Board Support Package (BSP)** page, select a kit supported by this code example. See [Supported kits](#supported-kits-make-variable-target).

   > **Note:** To use this code example for a kit not listed here, you may need to update the source files. If the kit does not have the required resources, the application may not work.

3. On the **Select Application** page:

   a. Select the **Applications(s) Root Path** and the **Target IDE**.

   > **Note:** Depending on how you open the Project Creator tool, these fields may be pre-selected for you.

   b. Select this code example from the list by enabling its check box.

   > **Note:** You can narrow the list of displayed examples by typing in the filter box.

   c. (Optional) Change the suggested **New Application Name** and **New BSP Name**.

   d. Click **Create** to complete the application creation process.

</details>

<details><summary><b>Use Project Creator CLI</b></summary>

The 'project-creator-cli' tool can be used to create applications from a CLI terminal or from within batch files or shell scripts. This tool is available in the *{ModusToolbox&trade; install directory}/tools_{version}/project-creator/* directory.

Use a CLI terminal to invoke the 'project-creator-cli' tool. On Windows, use the command-line 'modus-shell' program provided in the ModusToolbox&trade; installation instead of a standard Windows command-line application. This shell provides access to all ModusToolbox&trade; tools. You can access it by typing "modus-shell" in the search box in the Windows menu. In Linux and macOS, you can use any terminal application.

The following example clones the "[mtb-example-ce241040-lin-master-slave-communication](https://github.com/Infineon/mtb-example-ce241040-lin-master-slave-communication)" application with the desired name "LIN_master_slave" configured for the *KIT_PSOC4-HVMS-64K_LITE* BSP into the specified working directory, *C:/mtb_projects*:

   ```
   project-creator-cli --board-id KIT_PSOC4-HVMS-64K_LITE --app-id mtb-example-ce241040-lin-master-slave-communication --user-app-name LIN_master_slave --target-dir "C:/mtb_projects"
   ```

The 'project-creator-cli' tool has the following arguments:

Argument | Description | Required/optional
---------|-------------|-----------
`--board-id` | Defined in the <id> field of the [BSP](https://github.com/Infineon?q=bsp-manifest&type=&language=&sort=) manifest | Required
`--app-id`   | Defined in the <id> field of the [CE](https://github.com/Infineon?q=ce-manifest&type=&language=&sort=) manifest | Required
`--target-dir`| Specify the directory in which the application is to be created if you prefer not to use the default current working directory | Optional
`--user-app-name`| Specify the name of the application if you prefer to have a name other than the example's default name | Optional

> **Note:** The project-creator-cli tool uses the `git clone` and `make getlibs` commands to fetch the repository and import the required libraries. For details, see the "Project creator tools" section of the [ModusToolbox&trade; tools package user guide](https://www.infineon.com/ModusToolboxUserGuide) (locally available at {ModusToolbox&trade; install directory}/docs_{version}/mtb_user_guide.pdf).

</details>


### Open the project

After the project has been created, you can open it in your preferred development environment.


<details><summary><b>Eclipse IDE</b></summary>

If you opened the Project Creator tool from the included Eclipse IDE, the project will open in Eclipse automatically.

For more details, see the [Eclipse IDE for ModusToolbox&trade; user guide](https://www.infineon.com/MTBEclipseIDEUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mt_ide_user_guide.pdf*).

</details>


<details><summary><b>Visual Studio (VS) Code</b></summary>

Launch VS Code manually, and then open the generated *{project-name}.code-workspace* file located in the project directory.

For more details, see the [Visual Studio Code for ModusToolbox&trade; user guide](https://www.infineon.com/MTBVSCodeUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mt_vscode_user_guide.pdf*).

</details>


<details><summary><b>Keil µVision</b></summary>

Double-click the generated *{project-name}.cprj* file to launch the Keil µVision IDE.

For more details, see the [Keil µVision for ModusToolbox&trade; user guide](https://www.infineon.com/MTBuVisionUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mt_uvision_user_guide.pdf*).

</details>


<details><summary><b>IAR Embedded Workbench</b></summary>

Open IAR Embedded Workbench manually, and create a new project. Then select the generated *{project-name}.ipcf* file located in the project directory.

For more details, see the [IAR Embedded Workbench for ModusToolbox&trade; user guide](https://www.infineon.com/MTBIARUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mt_iar_user_guide.pdf*).

</details>


<details><summary><b>Command line</b></summary>

If you prefer to use the CLI, open the appropriate terminal, and navigate to the project directory. On Windows, use the command-line 'modus-shell' program; on Linux and macOS, you can use any terminal application. From there, you can run various `make` commands.

For more details, see the [ModusToolbox&trade; tools package user guide](https://www.infineon.com/ModusToolboxUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mtb_user_guide.pdf*).

</details>


## Operation

1. Connect the board to your PC using the provided USB cable through the KitProg3 USB connector.

2. Program two boards as LIN master board and LIN slave board.
   
   1. As shown in **Figure 2**, set the macro "LIN_MODE" to "MASTER_MODE" and program one device as a LIN master.
       
      **Figure 2. Macro for master mode**

      <img src="./images/master_mode.png" width="300"><br>


   2. As shown in **Figure 3**, set the macro "LIN_MODE" to "SLAVE_MODE" and program another device as a LIN slave. KIT_PSOC4-HVPA-144K_LITE uses only this mode.
   
      **Figure 3. Macro for slave mode**

      <img src="./images/slave_mode.png" width="300"><br>

   3. If it is for the loop-back mode, as shown in **Figure 4**, set the macro "LIN_MODE" to "LOOP_BACK_MODE" and program just one device for internal LIN master and LIN slave communication.

      **Figure 4. Macro for loop-back mode**

      <img src="./images/loop_back_mode.png" width="300"><br>

3. Program the board using one of the following:

   <details><summary><b>Using Eclipse IDE</b></summary>

      1. Select the application project in the Project Explorer.

      2. In the **Quick Panel**, scroll down, and click **\<Application Name> Program (KitProg3_MiniProg4)**.
   </details>


   <details><summary><b>In other IDEs</b></summary>

   Follow the instructions in your preferred IDE.
   </details>


   <details><summary><b>Using CLI</b></summary>

     From the terminal, execute the `make program` command to build and program the application using the default toolchain to the default target. The default toolchain is specified in the application's Makefile but you can override this value manually:
      ```
      make program TOOLCHAIN=<toolchain>
      ```

      Example:
      ```
      make program TOOLCHAIN=GCC_ARM
      ```
   </details>

4. After programming, each LED of the slave board blinks when each scheduled communication has been done. **Table 2** shows LIN master, slave message frame format, and which LED blinks. 

**Table 2. LIN master board, LIN slave message frame format, and output**<br>

**KIT_PSOC4-HVMS-128K_LITE or KIT_PSOC4-HVMS-64K_LITE**
Master       | Slave        | Output (Slave LED blinking) 
:----------- | :----------- | :--------------------------
TX_HEADER    | RX_HEADER    | LED4  
TX_RESPONSE  | RX_RESPONSE  | LED7
RX_RESPONSE  | TX_RESPONSE  | LED6

**KIT_PSOC4-HVPA-144K_LITE and KIT_PSOC-HVPA-SPM1_LITE**
Master       | Slave        | Output (Slave LED blinking) 
:----------- | :----------- | :--------------------------
TX_HEADER    | RX_HEADER    | OK if LED6 and LED7 are brinking  
TX_RESPONSE  | RX_RESPONSE  | LED6
RX_RESPONSE  | TX_RESPONSE  | LED7

5. If the transmitted data and received data do not match on both the master and slave sides, the operation stops as an error.

<br>


## Debugging

You can debug the example to step through the code.


<details><summary><b>In Eclipse IDE</b></summary>

Use the **\<Application Name> Debug (KitProg3_MiniProg4)** configuration in the **Quick Panel**. For details, see the "Program and debug" section in the [Eclipse IDE for ModusToolbox&trade; user guide](https://www.infineon.com/MTBEclipseIDEUserGuide).

</details>


<details><summary><b>In other IDEs</b></summary>

Follow the instructions in your preferred IDE.

</details>



## Design and implementation


### Resources and settings


**Table 3. Application resources**

Resource  |  Alias/object     |    Purpose
:------- | :--------------------------------- | :-----------------------------------------------
LIN      | LIN                                | Communication standard 
HVSS     | HVSS                               | To initialize LIN PHY
LED      | CYBSP_LED4, CYBSP_LED6, CYBSP_LED7 | To indicate succeeded LIN communication completion

<br>


## Related resources

Resources  | Links
-----------|----------------------------------
Application notes | [AN0034 – Getting started with PSOC&trade; 4 HVMS/HVPA MCUs in ModusToolbox&trade;](https://www.infineon.com/row/public/documents/10/42/infineon-an0034-getting-started-with-psoc-4-hv-ms-mcus-in-modustoolbox-applicationnotes-en.pdf)
Device documents | [PSOC&trade; 4 HV MS documents](https://www.infineon.com/cms/en/product/microcontroller/32-bit-psoc-arm-cortex-microcontroller/32-bit-psoc-4-hv-arm-cortex-m0/psoc-4-hv-ms/#!documents) <br>[PSOC&trade; 4 HV PA documents](https://www.infineon.com/products/microcontroller/32-bit-psoc-arm-cortex/psoc-4-hv-m0/hv-pa/#!documents)
Code examples  | [Using ModusToolbox&trade;](https://github.com/Infineon/Code-Examples-for-ModusToolbox-Software) on GitHub
Device documentation | [PSOC&trade; high voltage (HV) mixed signal (MS) automotive MCU 128K datasheets](https://www.infineon.com/assets/row/public/documents/10/49/infineon-cy8c41x7-psoc-4-high-voltage-hv-mixed-signal-ms-automotive-mcu-based-on-32-bit-arm-cortex--m0-datasheet-en.pdf) <br>[PSOC&trade; high voltage (HV) mixed signal (MS) automotive MCU 64K datasheets](https://www.infineon.com/assets/row/public/documents/10/49/infineon-cy8c41x5-cy8c41x6-psoc-4-high-voltage-hv-mixed-signal-ms-automotive-mcu-based-on-32-bit-arm-cortex--m0-datasheet-en-09018a9080d1ff70.pdf) <br>[PSOC&trade; high voltage (HV) precision analog (PA) automotive MCU 144K datasheets](https://documentation.infineon.com/psoc4atv/docs/rsd1669346756301) <br>[PSOC&trade; high voltage (HV) precision analog (PA) automotive MCU SPM1.0 datasheets](https://www.infineon.com/products/microcontroller/32-bit-psoc-arm-cortex/psoc-4-hv-m0/hv-pa#documents) <br>[PSOC&trade; high voltage (HV) mixed signal (MS) MCU: PSOC&trade; HVMS-128K registers reference manuals](https://www.infineon.com/row/public/documents/10/57/infineon-psoc-high-voltagehvmixed-signal-msmcu-psoc-hvms-128k-registers-reference-manual-additionaltechnicalinformation-en.pdf) <br>[PSOC&trade; high voltage (HV) mixed signal (MS) MCU: PSOC&trade; HVMS-64K registers reference manuals](https://www.infineon.com/content/dam/infineon/row/public/documents/10/57/infineon-psoc-4-high-voltagehvmixed-signalmsmcu-psoc4hvms-64k-registers-reference-manual-additionaltechnicalinformation-en.pdf) <br>[PSOC&trade; high voltage (HV) precision analogl (PA) MCU: PSOC&trade; HVPA-144K registers reference manuals](https://www.infineon.com/products/microcontroller/32-bit-psoc-arm-cortex/psoc-4-hv-m0/hv-pa#!documents) <br>[PSOC&trade; high voltage (HV) precision analogl (PA) MCU: PSOC&trade; HVPA-SPM1.0 registers reference manuals](https://www.infineon.com/products/microcontroller/32-bit-psoc-arm-cortex/psoc-4-hv-m0/hv-pa#!documents) <br>[PSOC&trade; high voltage (HV) mixed signal (MS) MCU architecture reference manuals](https://www.infineon.com/assets/row/public/documents/10/57/infineon-psoc-high-voltage-hv-mixed-signal-ms-mcu-architecture-reference-manual-additionaltechnicalinformation-en.pdf) <br>[PSOC&trade; high voltage (HV) precision analog (PA) MCU architecture reference manuals](https://documentation.infineon.com/psoc4atv/docs/vkg1670389100008)
Development kits | Select your kits from the [Evaluation board finder](https://www.infineon.com/cms/en/design-support/finder-selection-tools/product-finder/evaluation-board) page.
Libraries on GitHub  | [retarget-io](https://github.com/Infineon/retarget-io) – Utility library to retarget STDIO messages to a UART port <br>  [mtb-pdl-cat2](https://github.com/Infineon/mtb-pdl-cat2) – PSOC&trade; 4 Peripheral Driver Library (PDL)
Tools  | [ModusToolbox&trade;](https://www.infineon.com/modustoolbox) – ModusToolbox&trade; software is a collection of easy-to-use libraries and tools enabling rapid development with Infineon MCUs for applications ranging from wireless and cloud-connected systems, edge AI/ML, embedded sense and control, to wired USB connectivity using PSOC&trade; Industrial/IoT MCUs, AIROC&trade; Wi-Fi and Bluetooth&reg; connectivity devices, XMC&trade; Industrial MCUs, and EZ-USB&trade;/EZ-PD&trade; wired connectivity controllers. ModusToolbox&trade; incorporates a comprehensive set of BSPs, HAL, libraries, configuration tools, and provides support for industry-standard IDEs to fast-track your embedded application development

<br>


## Other resources

Infineon provides a wealth of data at [www.infineon.com](https://www.infineon.com) to help you select the right device, and quickly and effectively integrate it into your design.


## Document history

Document title: *CE241040* – *LIN master and slave communication with internal PHY*

 Version | Description of change
 ------- | ---------------------
 1.0.0   | New code example
 1.1.0   | Changed PSoC HV MS kit name to PSOC4 and updated README.md description.
 1.2.0   | Added support for KIT_PSOC4-HVMS-128K_LITE-02 and KIT_PSOC4-HVMS-64K_LITE-02 and updated to support ModusToolbox&trade; v3.5.
 1.3.0   | Added support for KIT_PSOC4-HVPA-144K_LITE
 1.4.0   | Added support for KIT_PSOC-HVPA-SPM1_LITE
<br>

All referenced product or service names and trademarks are the property of their respective owners.

The Bluetooth&reg; word mark and logos are registered trademarks owned by Bluetooth SIG, Inc., and any use of such marks by Infineon is under license.

PSOC&trade;, formerly known as PSoC&trade;, is a trademark of Infineon Technologies. Any references to PSoC&trade; in this document or others shall be deemed to refer to PSOC&trade;.

---------------------------------------------------------
(c) 2025, Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
This software, associated documentation and materials ("Software") is owned by Infineon Technologies AG or one of its affiliates ("Infineon") and is protected by and subject to worldwide patent protection, worldwide copyright laws, and international treaty provisions. Therefore, you may use this Software only as provided in the license agreement accompanying the software package from which you obtained this Software. If no license agreement applies, then any use, reproduction, modification, translation, or compilation of this Software is prohibited without the express written permission of Infineon.
<br>
Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A SPECIFIC USE/PURPOSE OR MERCHANTABILITY. Infineon reserves the right to make changes to the Software without notice. You are responsible for properly designing, programming, and testing the functionality and safety of your intended application of the Software, as well as complying with any legal requirements related to its use. Infineon does not guarantee that the Software will be free from intrusion, data theft or loss, or other breaches (“Security Breaches”), and Infineon shall have no liability arising out of any Security Breaches. Unless otherwise explicitly approved by Infineon, the Software may not be used in any application where a failure of the Product or any consequences of the use thereof can reasonably be expected to result in personal injury.