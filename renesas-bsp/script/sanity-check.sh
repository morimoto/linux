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
grep -v '^#' ${TOP}/remains			>> ${TMP}
grep -v '^#' ${TOP}/handled			>> ${TMP}

cat ${TMP} | sort | uniq -d

cat ${TOP}/full					>> ${TMP}

cat ${TMP} | sort | uniq -u

rm ${TMP}
