# Pixie-Net Software

This release contains all software necessary to work with the Pixie-Net device. You 
will need to compile the executables using the `make` command from the project root 
directory. I.e. the same directory containing this README. These files are installed 
into the `/var/www` directory on the Pixie-Net's OS.

Firmware may be downloaded from the 
[Pixie-Net Release page](https://github.com/xiallc/Pixie-Net_Releases/releases). 
**Ensure you download the firmware associated with your software release.** 
`PixieNetDefs.h` contains the software version information in the `PS_CODE_VERSION` 
variable. To convert the hex number to a version, take the first two digits as the 
major version the second two correspond to the minor. For example, 0x0224 would be 
software version 2.24.

## Compatibility Information

### Supported Hardware variants
* Hardware Revision B, 12 and 14 bit

### Firmware

| Firmware Type | Bit Res | Firmware ID | Firmware Files |
| --------------| ------- | ------------| -------------- |
| Standard | 12 | 0xA991_0232 | sd-bootfiles-pn-STD-2p32.zip | 
| Standard | 14 | 0xA9A1_0232 | sd-bootfiles-pn-14B-2p32.zip | 
| PSA | 12 | 0xA991_1231 | sd-bootfiles-pn-PSA-2p31.zip | 
| PSA | 14 | 0xA991_1233 | sd-bootfiles-pn-14B-PSA-2p33.zip | 


## File Name Conventions

| Component name | File Prefix | Description	| Install & Update |
| -------------- | ----------- | ------------ | ----------------- |
| Software | sw-arm-pn-[version] | The setup and DAQ procedures that go into /var/www on the Pixie Net’s Linux OS. |	Unzip, then copy to /var/www on the Pixie Net's Linux OS |
| Firmware | sd-bootfiles-pn-[variant]-[version]	| The Zynq controller bootfiles for the FAT partition of the SD card. Includes the pulse processing firmware that is specific to each hardware and firmware variant |	Unzip, then copy the 4 bootfiles to the FAT partition of the SD card. Ensure it matches the hardware (STD = 12bit, 14B = 14bit, PTP = PTP Ethernet) |
| SD image | sd-image-pn-[version] | The full (zip compressed) SD card image, includes sw-arm-pn, sd-bootfiles-pn, and all Linux OS files Only updated for changes in Linux OS |	Unzip, then write to an SD card with a byte-by-byte image writer. If an older version, update software and firmware |

## Firmware Type Descriptions

### Standard (STD)

This is the standard release firmware. It contains the basic funcationality necessary 
for data collection and analysis.

### Pulse Shape Analysis (PSA)

Includes pulse shape analysis functions to distinguish gammas and neutrons. It 
also includes constant fraction timing logic. Licensing required to use this firmware.

### I2C

Modifies PMOD I/O pins to support I2C and UART from Linux. Also includes real-time 
clock driver in Linux kernel.

### PTP

Provides hardware time stamping for LinuxPTP utility. Requires use of alternate 
Ethernet port.