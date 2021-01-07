@echo off

if not exist src\apollo3_init.h (
	echo Version file not exist.
	goto end
)

for /f "tokens=2,3" %%i in ('type src\apollo3_init.h') do (
  if %%i == APOLLO3_HUB_VER1 set "ver_1=%%j"
)

for /f "tokens=2,3" %%i in ('type src\apollo3_init.h') do (
  if %%i == APOLLO3_HUB_VER2 set "ver_2=%%j"
)

echo Get version from file:
echo %ver_1%%ver_2:~-2%

@echo on
cp ./keil/bin/apollo3b_hub.bin ./out/starter_apollo3b_hub.bin
cp ./tools/scripts/keys_info0.py ./tools/scripts/keys_info.py
python ./tools/scripts/create_cust_image_blob.py --bin ./out/starter_apollo3b_hub.bin --load-address 0xc000 --magic-num 0xcb -o ./out/temp_main_nosecure_ota --version %ver_1%%ver_2:~-2%
python ./tools/scripts/ota_binary_converter.py --appbin ./out/temp_main_nosecure_ota.bin -o ./out/update_binary_apollo3b_hub
rm -rf ./out/temp_main_nosecure_ota.bin

:end