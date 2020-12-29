cp ./keil/bin/apollo3b_hub.bin ./out/starter_apollo3b_hub.bin
cp ./tools/scripts/keys_info0.py ./tools/scripts/keys_info.py
python ./tools/scripts/create_cust_image_blob.py --bin ./out/starter_apollo3b_hub.bin --load-address 0xc000 --magic-num 0xcb -o ./out/temp_main_nosecure_ota --version 0x11
python ./tools/scripts/ota_binary_converter.py --appbin ./out/temp_main_nosecure_ota.bin -o ./out/update_binary_apollo3b_hub
rm -rf ./out/temp_main_nosecure_ota.bin
