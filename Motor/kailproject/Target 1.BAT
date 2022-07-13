SET PATH=E:\kail\ARM\ARMCC\Bin;C:\Program Files\Common Files\Siemens\Automation\Simatic OAM\bin;C:\Program Files (x86)\Intel\Intel(R) Management Engine Components\iCLS\;C:\Program Files\Intel\Intel(R) Management Engine Components\iCLS\;C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0\;C:\Program Files (x86)\Intel\Intel(R) Management Engine Components\DAL;C:\Program Files\Intel\Intel(R) Management Engine Components\DAL;C:\Program Files (x86)\Intel\Intel(R) Management Engine Components\IPT;C:\Program Files\Intel\Intel(R) Management Engine Components\IPT;C:\Windows\System32\OpenSSH\;E:\svn\bin;C:\Program Files\dotnet\;D:\git\Git\cmd;C:\Users\admin\AppData\Local\Microsoft\WindowsApps;;E:\VS\Microsoft VS Code\bin
SET ARMCC5_ASMOPT=--diag_suppress=9931
SET ARMCC5_CCOPT=--diag_suppress=9931
SET ARMCC5_LINKOPT=--diag_suppress=9931
SET CPU_TYPE=MM32SPIN06PF
SET CPU_VENDOR=MindMotion
SET UV2_TARGET=Target 1
SET CPU_CLOCK=0x007A1200
"E:\kail\ARM\ARMCC\Bin\ArmCC" --Via ".\hex\system_mm32spin06xx_s.__i"
"E:\kail\ARM\ARMCC\Bin\ArmAsm" --Via ".\hex\startup_mm32spin06xx_s_keil._ia"
"E:\kail\ARM\ARMCC\Bin\ArmCC" --Via ".\hex\hal_gpio.__i"
"E:\kail\ARM\ARMCC\Bin\ArmCC" --Via ".\hex\hal_rcc.__i"
"E:\kail\ARM\ARMCC\Bin\ArmCC" --Via ".\hex\hal_uart.__i"
"E:\kail\ARM\ARMCC\Bin\ArmCC" --Via ".\hex\hal_misc.__i"
"E:\kail\ARM\ARMCC\Bin\ArmCC" --Via ".\hex\hal_tim.__i"
"E:\kail\ARM\ARMCC\Bin\ArmCC" --Via ".\hex\hal_spi.__i"
"E:\kail\ARM\ARMCC\Bin\ArmCC" --Via ".\hex\hal_can.__i"
"E:\kail\ARM\ARMCC\Bin\ArmCC" --Via ".\hex\delay.__i"
"E:\kail\ARM\ARMCC\Bin\ArmCC" --Via ".\hex\uart.__i"
"E:\kail\ARM\ARMCC\Bin\ArmCC" --Via ".\hex\gpio.__i"
"E:\kail\ARM\ARMCC\Bin\ArmCC" --Via ".\hex\tim3_pwm_input.__i"
"E:\kail\ARM\ARMCC\Bin\ArmCC" --Via ".\hex\spi.__i"
"E:\kail\ARM\ARMCC\Bin\ArmCC" --Via ".\hex\can.__i"
"E:\kail\ARM\ARMCC\Bin\ArmCC" --Via ".\hex\motorcan.__i"
"E:\kail\ARM\ARMCC\Bin\ArmCC" --Via ".\hex\queue.__i"
"E:\kail\ARM\ARMCC\Bin\ArmCC" --Via ".\hex\main.__i"
"E:\kail\ARM\ARMCC\Bin\ArmLink" --Via ".\HEX\motor_can.lnp"
"E:\kail\ARM\ARMCC\Bin\fromelf.exe" ".\HEX\motor_can.axf" --i32combined --output ".\HEX\motor_can.hex"
