#! /bin/bash
#===============================
#
# commit-bsp61x.sh
#
# 2022/12/01 Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
#===============================
for bsp in $@
do
	egrep "^ - [0-9a-f]{12} " ${bsp} | sed -e "s/^ - //g" | sed -e "s/ #//"
done
