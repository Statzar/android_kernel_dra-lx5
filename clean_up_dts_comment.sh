#!/bin/bash
#set -x
:<<EOF
Description:
        A bash scripts to delete DTS, names and ID numbers in kernel source
Useage: run the script in the kernel source root directory
Note:   delete the .git .svn manually after all tests done
Requirement(hasn't tested with other version):
	bash-4.2, find-4.4, grep-2.10, sed-4.2, enca-1.13

Author: MaYaohui(m00194513@notesmail.huawei.com)
Date:   2012-09-15
History:
        2012-09-16: Add ".*[^^ \t]" to make "sed" non-greedy
        2012-09-16: Use an array to manage match patterns

EOF

########## configuration here ##########
PATTERN_ARRAY=(
    [0]='DTS20[0-9]\{11\}'
    [1]='\s*\<\([a-z]00[0-2][0-9]\{5\}\|00[0-2][0-9]\{5\}\|[0-2][0-9]\{5\}\|[a-z][3-6][0-9]\{4\}\|[a-z]kf[0-9]\{5\}\|[a-z]8[14]00[0-9]\{4\}\|8[14]00[0-9]\{4\}\)\>'
)
CHECK_GREP_PATTERN="\(${PATTERN_ARRAY[0]}\)\|\(${PATTERN_ARRAY[1]}\)"

ENCODING_ARRAY=(
    [0]="zh_CN.GB2312"            # windows default is GB2312
    [1]="en_US.UTF-8"             # linux default is UTF-8
)

########## start cleaning here ##########

# check if it has the comment with keywords above
function check_pattern ()
{
    grep "${CHECK_GREP_PATTERN}" \
	--exclude-dir=".git" --exclude-dir="Documentation" \
	--binary-files=without-match \
	$@ 2>/dev/null
}

function do_clean_c_style_comment ()
{
    # loop for the different chinese encodings
    # sed can't recognize GB2312 encoding automaticlly on a UTF-8 system
#--------------------------------------------------
    for ENCODING in "${ENCODING_ARRAY[@]}"; do
    export LANG=${ENCODING}
#--------------------------------------------------
	# loop for the different match pattrens
	for PATTERN in "${PATTERN_ARRAY[@]}"; do
		# delete the "//" comments
		sed -i "s/\(.*\/\*.*\*\/\)\s*\/\*.*${PATTERN}.*/\1/i" "$1"
	    sed -i "s/\(.*[^^ \t\/]\)\s*\/\/.*${PATTERN}.*/\1/i" "$1"
	    sed -i "s/\(.*\*\/\)\s*\/\/.*${PATTERN}.*/\1/i" "$1"
	    sed -i "/^\s*\/\/.*${PATTERN}.*/Id" "$1"
	    # delete the "/* */" comments
	    sed -i "/^\s*\/\*.*${PATTERN}.*\*\/\s*$/Id" "$1"
		sed -i "/^[ \t]*\/\*.*${PATTERN}.*/,/.*\*\//d" "$1"
	    sed -i ":begin; /\/\*/,/\*\// { /\*\//! { $! { N; b begin }; };
		s/\(\S*\)\s*\/\*.*${PATTERN}.*\*\/[ \t\x0d\x0e\x10\x16\x1a]*/\1/i; };" "$1"
	done
#--------------------------------------------------
    done
#--------------------------------------------------
}

function do_clean_pound_comment ()
{
#--------------------------------------------------
    for ENCODING in "${ENCODING_ARRAY[@]}"; do
    export LANG=${ENCODING}
#--------------------------------------------------
	for PATTERN in "${PATTERN_ARRAY[@]}"; do
	    sed -i "s/\(.*[^^ \t\#]\)\s*\#.*${PATTERN}.*/\1/i" "$1"
	    sed -i "/^\s*\#.*${PATTERN}.*/Id" "$1"
	done
#--------------------------------------------------
    done
#--------------------------------------------------
}

function clean_c_and_asm ()
{
    # C/C++ & Asm language
    find -type f \
	-iregex ".*\.\(c\|h\|s\|cc\|cpp\|inc\|debug\|dtsi\)$" -or \
	-name "hisi_3635fpga_defconfig" -or -name "hisi_hi6210sft_defconfig" -or \
	-name "merge_hisi_k3v3fpga_defconfig" -or -name "hisi_3635_defconfig" \
	-not -path "\./Documentation/*" | \
	while read file; do
	    check_pattern $file >/dev/null
	    if [ $? -eq "0" ]; then
		do_clean_c_style_comment $file
		#--------------------------------------------------
		# if enca ${file} | grep Chinese >/dev/null 2>&1; then
		#     echo "$file"
		# fi
		#--------------------------------------------------
	    fi
    done
}

function clean_buildfiles_and_scripts ()
{
    # Makefiles, Configs and Shell/Perl scripts
    find -type f \
	\( -iname "Makefile" -or -name "Kconfig" -or -name "Kbuild" -or \
	-name "*.sh" -or -name "*.pl" -or -name "*.py" -or -name "*.define" -or \
	-name "*.rc" -or -name "*.mk" -or -path "\./arch/*/configs/*" \) \
	-not -path "\./Documentation/*" | \
	while read file; do
	    check_pattern $file >/dev/null
	    if [ $? -eq "0" ]; then
		do_clean_pound_comment $file
		#--------------------------------------------------
		# if enca ${file} | grep Chinese >/dev/null 2>&1; then
		#     echo "$file"
		# fi
		#--------------------------------------------------
	    fi
	done
}

function clean_all ()
{
    clean_buildfiles_and_scripts
    clean_c_and_asm
}

echo "===================================="
echo "=   Chinese words in these files   ="
echo "===================================="
clean_all

echo
echo "===================================="
echo "= We left these, fix them manually ="
echo "===================================="
check_pattern -rn .

echo
echo "===================================="
echo "=  you should look at this either  ="
echo "===================================="
grep "\b\(add\|modify\|delete\) \(begin\|end\)\b" -rin . 2>/dev/null

