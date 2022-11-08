#! /bin/bash
#===============================
#
# re-handled
#
# re-order "handled" file to be same order as "full" file.
# It is easy to check the difference.
# New file will be "handled2"
#
# 2023/02/21 Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
#===============================

# You need to use this script at ${LINUX}/renesas-bsp

: > handled2
for commit in `cut -d " " -f 1 full`
do
	grep ${commit} handled >> handled2
done
