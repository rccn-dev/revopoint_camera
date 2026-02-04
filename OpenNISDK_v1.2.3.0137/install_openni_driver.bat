::**************************************************************************
::*                                                                          *
::*  OpenNI 2.0                                                              *
::*  Copyright (C) 2012 Chishine Optoelectronics Technology Co.,LTD.         *
::*                                                                          *
::*  This file is part of OpenNI2 installation.                              *
::*                                                                          *
::*  OpenNI is free software: you can redistribute it and/or modify          *
::*  it under the terms of the GNU Lesser General Public License as published*
::*  by the Free Software Foundation, either version 3 of the License, or    *
::*  (at your option) any later version.                                     *
::*                                                                          *
::*  OpenNI is distributed in the hope that it will be useful,               *
::*  but WITHOUT ANY WARRANTY; without even the implied warranty of          *
::*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the            *
::*  GNU Lesser General Public License for more details.                     *
::*                                                                          *
::*  You should have received a copy of the GNU Lesser General Public License*
::*  along with OpenNI. If not, see <http://www.gnu.org/licenses/>.          *
::*                                                                          *
::***************************************************************************/

@echo off

echo %~dp0

if defined OPENNI2_REDIST64 (
    copy %~dp0\lib\windows\3DCamera.dll "%OPENNI2_REDIST64%\OpenNI2\Drivers\"
    copy %~dp0\lib\windows\csdevice.dll "%OPENNI2_REDIST64%\OpenNI2\Drivers\"
    copy %~dp0\lib\windows\3DCamera.dll "%OPENNI2_REDIST64%\..\Tools\OpenNI2\Drivers\"
    copy %~dp0\lib\windows\csdevice.dll "%OPENNI2_REDIST64%\..\Tools\OpenNI2\Drivers\"
    copy %~dp0\lib\windows\3DCamera.dll "%OPENNI2_REDIST64%\..\Samples\Bin\OpenNI2\Drivers\"
    copy %~dp0\lib\windows\csdevice.dll "%OPENNI2_REDIST64%\..\Samples\Bin\OpenNI2\Drivers\"
) else (
    echo please install openni first
)

pause