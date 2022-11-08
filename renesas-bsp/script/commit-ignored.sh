#! /bin/bash
#===============================
#
# commit-ignored
#
# 2022/12/01 Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
#===============================
TOP=`readlink -f "$0" | xargs dirname | xargs dirname`

egrep "[0-9a-f]{12} " ${TOP}/ignored | sed -e "s/^\t\t//g"
