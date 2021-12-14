# btsdk-mesh

### Overview

This repo contains the Mesh application library used in BTSDK 2.0 and higher. All mesh applications include this common component. This application library includes source code for application initialization, GATT database, Bluetooth&#174; configuration, Mesh provisioning, and AIROC&#8482; HCI interface. Unlike other BTSDK applications, the application source code in mesh sample applications does not include application_start, GATT database, or Bluetooth&#174; configuration. These common tasks are consolidated in the Mesh application library for ease of use. If you create a GATT database using Bluetooth&#174; Configurator, update the GATT database in the mesh application library. Mesh application library is provided as source code and users can thus re-structure their application to make it similar to other BTSDK applications if needed.

For more information about Mesh application structure, see this [link](https://github.com/cypresssemiconductorco/CypressAcademy_BT101_Files/blob/master/PDFs/WBT101-07C-Mesh-Firmware.pdf)

For Mesh topology information, see this [link](https://github.com/cypresssemiconductorco/CypressAcademy_BT101_Files/blob/master/PDFs/WBT101-07A-Mesh-Topology.pdf)
For Mesh protocol details, see this [link](https://github.com/cypresssemiconductorco/CypressAcademy_BT101_Files/blob/master/PDFs/WBT101-07B-Mesh-Details.pdf)
