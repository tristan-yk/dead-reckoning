# Common paths for building and flashing the dead-reckoning firmware without
# opening STM32CubeIDE. The IDE is not required, but its bundled toolchain is.
# Sourced by build.sh and flash.sh, which set SCRIPT_DIR first.
CUBEIDE="/c/ST/STM32CubeIDE_1.16.1/STM32CubeIDE/plugins"

TOOLCHAIN_BIN="$CUBEIDE/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.12.3.rel1.win32_1.0.200.202406191623/tools/bin"
MAKE_BIN="$CUBEIDE/com.st.stm32cube.ide.mcu.externaltools.make.win32_2.1.300.202402091052/tools/bin"
PROG_BIN="$CUBEIDE/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.1.400.202404281720/tools/bin"

PROGRAMMER="$PROG_BIN/STM32_Programmer_CLI.exe"
FW_DIR="$(cd "$SCRIPT_DIR/../Firmware" && pwd)"
BUILD_DIR="$FW_DIR/Debug"
ELF="$BUILD_DIR/dead_reckoning.elf"

export PATH="$TOOLCHAIN_BIN:$MAKE_BIN:$PATH"
