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

for commit in `cut -d " " -f 1 full`
do
	sed -n < handled \
	"# Process all lines:
	 # 1. In case of a comment: save comment
	 #    H: Append pattern space to hold space,
	 /^#/H;
	 # 2. In case of a matching commit: print saved comments and commit
	 #    H: Append pattern space to hold space,
	 #    x: Exchange the contents of the hold and pattern spaces,
	 #    p: Print the current pattern space,
	 #    q: Quit.
	 /^${commit}/{H; x; p; q};
	 # 3. Any other commit: delete saved comments
	 #    s: Clear pattern space (but it keeps the newline :-(,
	 #    h: Copy pattern space to hold space.
	 /^[0-9a-f]/ { s/.*//; h}"
	# Empty lines are filtered by grep below
done | grep -v "^$" > handled2
