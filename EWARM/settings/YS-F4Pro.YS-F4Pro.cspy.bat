@REM This batch file has been generated by the IAR Embedded Workbench
@REM C-SPY Debugger, as an aid to preparing a command line for running
@REM the cspybat command line utility using the appropriate settings.
@REM
@REM Note that this file is generated every time a new debug session
@REM is initialized, so you may want to move or rename the file before
@REM making changes.
@REM
@REM You can launch cspybat by typing the name of this batch file followed
@REM by the name of the debug file (usually an ELF/DWARF or UBROF file).
@REM
@REM Read about available command line parameters in the C-SPY Debugging
@REM Guide. Hints about additional command line parameters that may be
@REM useful in specific cases:
@REM   --download_only   Downloads a code image without starting a debug
@REM                     session afterwards.
@REM   --silent          Omits the sign-on message.
@REM   --timeout         Limits the maximum allowed execution time.
@REM 


@echo off 

if not "%~1" == "" goto debugFile 

@echo on 

"H:\IAR Systems\Embedded Workbench 7.2\common\bin\cspybat" -f "F:\代码备份\2019_6_03 备份F4步进电机方向控制_正本\YSF4_HAL_MOTOR-104. 四轴57&42步进电机旋转控制实现\EWARM\settings\YS-F4Pro.YS-F4Pro.general.xcl" --backend -f "F:\代码备份\2019_6_03 备份F4步进电机方向控制_正本\YSF4_HAL_MOTOR-104. 四轴57&42步进电机旋转控制实现\EWARM\settings\YS-F4Pro.YS-F4Pro.driver.xcl" 

@echo off 
goto end 

:debugFile 

@echo on 

"H:\IAR Systems\Embedded Workbench 7.2\common\bin\cspybat" -f "F:\代码备份\2019_6_03 备份F4步进电机方向控制_正本\YSF4_HAL_MOTOR-104. 四轴57&42步进电机旋转控制实现\EWARM\settings\YS-F4Pro.YS-F4Pro.general.xcl" "--debug_file=%~1" --backend -f "F:\代码备份\2019_6_03 备份F4步进电机方向控制_正本\YSF4_HAL_MOTOR-104. 四轴57&42步进电机旋转控制实现\EWARM\settings\YS-F4Pro.YS-F4Pro.driver.xcl" 

@echo off 
:end