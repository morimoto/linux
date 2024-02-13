#! /bin/bash
#===============================
#
# list-x5h-patch
#
# 2026/07/03 Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
#===============================
TOP=`readlink -f "$0" | xargs dirname | xargs dirname`

STRT=renesas-lts-v6.12.80
END=renesas-bsp/v6.12.80/rcar-6.1.0

git log --oneline ${STRT}..${END} > ${TOP}/full
