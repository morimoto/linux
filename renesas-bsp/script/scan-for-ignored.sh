#!/bin/bash
#
# This script scans 'remains' for certain keywords which indicate that commits
# do not need to be upstreamed (like 'UIO'). It moves them to an existing
# section in 'ignored'. If none can be found, a new one is created.
#
# The format of the keyword table is:
# <sed-regex to match commits> <section name in 'ignored'> <reason for not upporting>
#	UIO\\|OSAL                      UIO            		UIO and OSAL are not intended for upstream.

rem_file='remains'
ign_file='ignored'
dropped_log='dropped'

[ ! -r "$rem_file" ] && echo "$rem_file not found!" && exit 1
[ ! -r "$ign_file" ] && echo "$ign_file not found!" && exit 1

while read match token reason; do
	[ "${match:0:1}" = "#" ] && continue
	sed -i -e "/$match/I s/^/\t\t/w $dropped_log" -e "/$match/I d" $rem_file
	git diff --quiet && continue
	gawk "
		BEGIN { IGNORECASE = 1 }
		/^[^\t]*$token/ { flag = 1; found = 1 }
		flag && /^\t\t/ { while (getline line < \"$dropped_log\") { print line }; flag = 0 }
		{ print }
		END { if (!found) { print \"$token\\n\\t$reason\"; while (getline < \"$dropped_log\") { print } print \"\" }}
	" $ign_file | sponge $ign_file
	gawk '/^[^\t ]/ { printf "\0" } { print }' $ign_file | sort -z | tr -d '\0' | sponge $ign_file
	git commit -a -s -m "BSP rebase: ${token//./ } patches not supported upstream" -m "$reason"
done <<-"EOF"
	ADSP				ADSP		This is a local patch file for MMP. So we don't need to upport.
	AVS				AVS		AVS code now is just in-house code.
	BRS				BRS		This is a local patch file for MMP. So we don't need to upport.
	\\<EMS\\>			EMS		This is just internal code.
	FBC				FBC		FBC is not intended for upstream.
# FCP is supported meanwhile
	GSX				GSX		This is a local patch file for MMP. So we don't need to upport.
	iVDP1C				iVDP1C		This is a local patch file for MMP. So we don't need to upport.
	\\<IMP\\>				IMP		IMP is not intended for upstream.
# ISP is not supported but ISPCS is (belongs to VIN)
	\\<ISP[^C]			ISP		ISP is not intended for upstream.
	MMNGR				MMNGR		This is a local patch file for MMP. So we don't need to upport.
	POST				POST		No user present in the upporting list.
	QOS				QOS		This is a local patch file for MMP. So we don't need to upport.
	RT-DMAC				RT-DMAC		not used by BSP DTs.
	VCP4				VCP4		This is a local patch file for MMP. So we don't need to upport.
	Vision.DSP\\|\\<PAP\\>		VisionDSP+PAP	VisionDSP+PAP is not intended for upstream.
	VSP[XBSA2M]			VSP		This is a local patch file for MMP. So we don't need to upport.
	reserved.memory.region		MMP		Reserved memory regions are not intended for upstream.
	UIO\\|OSAL			UIO		UIO and OSAL are not intended for upstream.
	Update.IPMMU.ID			UIO		UIO and OSAL are not intended for upstream.
EOF

rm $dropped_log
