@echo off
::Keilִ���ļ�λ��
set UV=D:\Keil_v5\UV4\UV4.exe
::����uvprojx�����ļ�
for /f "usebackq delims=" %%j in (`dir /s /b %cd%\apollo3b_hub.uvprojx`) do (
if exist %%j (
set UV_PRO_PATH="%%j"))
echo ---------------------------------------------------------------
echo Author:Thundercomm
echo %UV_PRO_PATH%
echo Init building ...
echo >build_log.txt
::clean project
%UV% -c %UV_PRO_PATH%

::build project
%UV% -j0 -b %UV_PRO_PATH% -l %cd%\build_log.txt
type build_log.txt

echo Done.
pause
