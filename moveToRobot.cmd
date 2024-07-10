@echo off

rem empty the directory
.\platform-tools\adb shell rm /sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode/*

rem push all java files to control hub
setlocal enabledelayedexpansion
for %%f in (*.java) do (
    .\platform-tools\adb push "%%f" /sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode/
)

if 1==0 (
    rem compile all java files 
    java -version > nul 2>&1
    if %errorlevel% == 0 (

        rem compile java files
        for %%f in (*.java) do (
            javac %%f
        )
        
        rem empty compiled files directory
        .\platform-tools\adb shell rm /sdcard/FIRST/java/build/classes/org/firstinspires/ftc/teamcode/*
        
        rem push comipled class files to robot and deletes class files
        for %%f in (*.class) do (
            .\platform-tools\adb push "%%f" /sdcard/FIRST/java/build/classes/org/firstinspires/ftc/teamcode/
            del "%%f"
        )

    ) else (
        echo Java is not installed or not found in the PATH
    )
)