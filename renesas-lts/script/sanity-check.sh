#! /bin/bash
#===============================
#
# sanity-check
#
# 2022/12/01 Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
#===============================
TOP=`readlink -f "$0" | xargs dirname | xargs dirname`

TMP=/tmp/sanity-check-$$

${TOP}/script/commit-ignored.sh			>  ${TMP}
grep -vh '^#' ${TOP}/commits/remains*		>> ${TMP}
grep -v  '^#' ${TOP}/commits/active		>> ${TMP}
grep -v  '^#' ${TOP}/commits/handled		>> ${TMP}

cat ${TMP} | sort | uniq -d

grep -v '^#' ${TOP}/commits/full		>> ${TMP}

cat ${TMP} | sort | uniq -u

rm ${TMP}
