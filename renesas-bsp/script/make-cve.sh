#! /bin/bash
#===============================
#
# make-cve
#
# 1) You need to prepare latest cip-kernel-sec
#	https://gitlab.com/cip-project/cip-kernel/cip-kernel-sec.git
#
# 2) checkout target branch
#    it should be LTS stable kernel
#                ^^^^^^^^^^^^^^^^^^^
#
# 3) goto cip-kernel-sec
#
# 4) call this script
#
# 5) see /tmp/cve-xx-cve
#
# 2023/02/22 Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
#===============================
V1=`head ../kernel/Makefile | grep "^VERSION"    | cut -d " " -f 3`
V2=`head ../kernel/Makefile | grep "^PATCHLEVEL" | cut -d " " -f 3`
V3=`head ../kernel/Makefile | grep "^SUBLEVEL"   | cut -d " " -f 3`

TMP="/tmp/cve-$$"

./scripts/report_affected.py stable/${V1}.${V2}:v${V1}.${V2}.${V3}	> ${TMP}-orig
cat ${TMP}-orig | sed -e "s/ /\n/g" | sed 1d				> ${TMP}-affected
(
	cd issues
	ls -1 CVE-*.yml | sed -e "s/\.yml$//g" 				> ${TMP}-all
)
cat ${TMP}-all ${TMP}-affected | sort | uniq -u | tac			> ${TMP}-cve
