#!/usr/bin/env bash

echo "== Convert ctest log into md5 list =="

if [ $# -ne 1 ]; then
    echo "This script takes as input one file path. Example: 'tests/convertTestLogInListMD5.sh _build/Release/Testing/Temporary/LastTest.log'"
    exit 1
fi

logFile="$1"

if [ ! -f "$logFile" ]; then
    echo "Error: File '$logFile' not found."
    exit 1
fi

mapfile -t lines < "$logFile"

echo
echo "test_name,md5_bitstream,md5_pc;"

output_lines=()

for ((i=0; i<${#lines[@]}; i++)); do
    line="${lines[$i]}"
    if [[ "$line" == *"Test:"*"_md5_bitstream"* ]]; then
        testName=$(echo "$line" | sed -E 's/.*Test: (.*)_md5_bitstream.*/\1/')
        sha_line_index=$((i + 6))
        sha=$(echo "${lines[$sha_line_index]}" | awk '{print $1}')
        output_lines+=("${testName},${sha},;")
    fi
done

printf '%s\n' "${output_lines[@]}" | sort

echo
