#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")"
mkdir -p build/cortex_m4
# Relocated Debian toolchains may not discover their Newlib headers.
extra_includes=()
compiler_path="$(readlink -f "$(command -v arm-none-eabi-gcc)")"
newlib_include="${S2_NEWLIB_INCLUDE:-$(dirname -- "$(dirname -- "$compiler_path")")/include/newlib}"
if [[ -d "$newlib_include" ]]; then extra_includes=(-isystem "$newlib_include"); fi
rm -f build/cortex_m4/*.o build/cortex_m4/libs2_m4f.a
for file in s2_deploy main; do
  arm-none-eabi-gcc -std=c99 -O2 -Wall -Wextra -Werror -pedantic \
    -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
    -ffp-contract=off -ffunction-sections -fdata-sections -fstack-usage \
    "${extra_includes[@]}" -Ic -c "c/$file.c" -o "build/cortex_m4/$file.o"
done
arm-none-eabi-ar rcs build/cortex_m4/libs2_m4f.a build/cortex_m4/s2_deploy.o
arm-none-eabi-size build/cortex_m4/*.o
echo 'Built M4F hard-float archive; board firmware must link libm and match ABI.'
