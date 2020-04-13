﻿param([String]$debugfile = "");

# This powershell file has been generated by the IAR Embedded Workbench
# C - SPY Debugger, as an aid to preparing a command line for running
# the cspybat command line utility using the appropriate settings.
#
# Note that this file is generated every time a new debug session
# is initialized, so you may want to move or rename the file before
# making changes.
#
# You can launch cspybat by typing Powershell.exe -File followed by the name of this batch file, followed
# by the name of the debug file (usually an ELF / DWARF or UBROF file).
#
# Read about available command line parameters in the C - SPY Debugging
# Guide. Hints about additional command line parameters that may be
# useful in specific cases :
#   --download_only   Downloads a code image without starting a debug
#                     session afterwards.
#   --silent          Omits the sign - on message.
#   --timeout         Limits the maximum allowed execution time.
#


if ($debugfile -eq "")
{
& "C:\IAR Systems\Embedded Workbench 8.0\common\bin\cspybat" -f "D:\M_2019\T21_BLE_CC2541pHX711\BLE-CC254x-1.4.0_BLE_HX711_TeB_02\Projects\ble\SBP_BLE_HX711\CC2541DB\settings\SimpleBLEPeripheral.CC2541.general.xcl" --backend -f "D:\M_2019\T21_BLE_CC2541pHX711\BLE-CC254x-1.4.0_BLE_HX711_TeB_02\Projects\ble\SBP_BLE_HX711\CC2541DB\settings\SimpleBLEPeripheral.CC2541.driver.xcl" 
}
else
{
& "C:\IAR Systems\Embedded Workbench 8.0\common\bin\cspybat" -f "D:\M_2019\T21_BLE_CC2541pHX711\BLE-CC254x-1.4.0_BLE_HX711_TeB_02\Projects\ble\SBP_BLE_HX711\CC2541DB\settings\SimpleBLEPeripheral.CC2541.general.xcl" --debug_file=$debugfile --backend -f "D:\M_2019\T21_BLE_CC2541pHX711\BLE-CC254x-1.4.0_BLE_HX711_TeB_02\Projects\ble\SBP_BLE_HX711\CC2541DB\settings\SimpleBLEPeripheral.CC2541.driver.xcl" 
}
