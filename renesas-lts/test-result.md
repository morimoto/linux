Tested version: Renesas-LTS v6.12.100
Tested Board: Salvator-XS board (R-Car H3 v3.0)

| No. | Modules | Result |
|---|---|---|
| 1 | ARCH_TIMER | OK |
| 2 | CMT | OK |
| 3 | CPU_HOTPLUG | OK |
| 4 | DMAE | OK |
| 5 | ETHERNET | OK |
| 6 | GPIO | OK |
| 7 | HSCIF_DMA | OK |
| 8 | HSCIF_PIO | OK |
| 9 | I2C | OK |
| 10 | MMC | OK |
| 11 | MSIOF_DMA_Master | OK |
| 12 | MSIOF_DMA_Slave | OK |
| 13 | MSIOF_PIO_Master | OK |
| 14 | MSIOF_PIO_Slave | OK |
| 15 | PCIe_Ether | OK |
| 16 | PM | OK |
| 17 | SATA | OK |
| 18 | SCIF_DMA | OK |
| 19 | SCIF_PIO | OK |
| 20 | SDHI | OK |
| 21 | SOUND | OK |
| 22 | THERMAL | OK |
| 23 | TMU | OK |
| 24 | USB2F | OK |
| 25 | USB3F | OK |
| 26 | USB_HOST | OK |
| 27 | V4L2 | OK |
| 28 | VIDEO_INPUT | OK |

Total/OK/NG - 28/28/0

Tested Board: White Hawk Single (R-Car V4H v3.0)

| No. | Modules | Result |
|---|---|---|
| 1 | CAN_CLASSIC | OK |
| 2 | CAN_FD | OK |
| 3 | DMAE | OK |
| 4 | ETHERNET_AVB | OK |
| 5 | ETHERNET_TSN | SKIP |
| 6 | GPIO | OK |
| 7 | I2C | OK |
| 8 | IPMMU | OK |
| 9 | MSIOF_DMA_Master | OK |
| 10 | MSIOF_DMA_Slave | OK |
| 11 | MSIOF_PIO_Master | OK |
| 12 | MSIOF_PIO_Slave | OK |
| 13 | PCIE | OK |
| 14 | PM_CPUFREQ | OK |
| 15 | PM_HOTPLUG | OK |
| 16 | PM_IDLE | OK |
| 17 | PM_RUNTIME | OK |
| 18 | PWM | OK |
| 19 | QSPI | OK |
| 20 | RWDT | OK |
| 21 | SCIF | OK |
| 22 | SD_MMC | OK |
| 23 | THERMAL | OK |
| 24 | TIMER | OK |
| 25 | TPU | OK |
| 26 | VIN | OK |

Total/OK/NG/SKIP - 26/25/0/1

**Note:**
- Kernel LTS v6.12 doesn't support TSN driver communicate to Radmoon 2 for V4H on White Hawk Single
- Please contact to duy.nguyen.rh@renesas.com if you need to know test results more details.
