.. SPDX-License-Identifier: GPL-2.0

====================================================
Firmware of Multi-Protocol PHY for Renesas R-Car X5H
====================================================

Renesas R-Car X5H (r8a78000) has a Multi-Protocol PHY, requiring a specific
firmware download during startup.

However, Renesas currently cannot distribute the firmware free of charge.

The firmware file "MPPHY_fw_offsetaddr_data_ver4p01a_patch0a.xlsx" (note that
the file name might be different between different datasheet revisions) can be
found in the datasheet encoded as spreadsheet, and as such, the file's content
must be converted back to binary form. This can be achieved using the following
example script:

.. code-block:: sh

	$ libreoffice --headless --convert-to csv MPPHY_fw_offsetaddr_data_ver4p01a_patch0a.xlsx
	$ tail -n +3 MPPHY_fw_offsetaddr_data_ver4p01a_patch0a.csv | cut -d "," -f 3 | \
		sed 's@0x\(..\)\(..\)@0x\2\1@' | xxd -r -p > rcar_gen5_mp_phy.bin

Once the text content has been converted into a binary firmware file, verify
its checksum as follows:

.. code-block:: sh

	$ sha256sum rcar_gen5_mp_phy.bin
	d9352ba55f095b7f5adaa3f2bce93f5c58eee310b0faf39c290cb5885e1aa7b8  rcar_gen5_mp_phy.bin

The resulting binary file called "rcar_gen5_mp_phy.bin" should be placed in
the "/lib/firmware" directory before the driver runs.
