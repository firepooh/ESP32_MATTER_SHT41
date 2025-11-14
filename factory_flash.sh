#!/bin/bash
echo "Write Factory Data"

# 파라미터 체크
if [ $# -eq 0 ]; then
    echo "Usage: $0 <1|2|3|4|5>"
    exit 1
fi

# case문으로 분기
case "$1" in
    1)
        echo "write case1"
        esptool.py write_flash 0x3E0000 out/fff2_8001/63a899fa-e5c7-4cd4-bc01-131629fd35cb/63a899fa-e5c7-4cd4-bc01-131629fd35cb-partition.bin
        ;;
    2)
        echo "write case2"
        esptool.py write_flash 0x3E0000 out/fff2_8001/69e615b0-5f41-4ed9-9add-0e1c4869468f/69e615b0-5f41-4ed9-9add-0e1c4869468f-partition.bin
        ;;
    3)
        echo "write case3"
        esptool.py write_flash 0x3E0000 out/fff2_8001/0546a175-e721-433d-a854-6c07dce844a7/0546a175-e721-433d-a854-6c07dce844a7-partition.bin
        ;;
    4)
        echo "write case4"
        esptool.py write_flash 0x3E0000 out/fff2_8001/849c3da6-5af4-46c4-b3e3-1f91b5acec02/849c3da6-5af4-46c4-b3e3-1f91b5acec02-partition.bin
        ;;
    5)
        echo "write case5"
        esptool.py write_flash 0x3E0000 out/fff2_8001/db5770f0-9bbb-45ec-97fc-82b095c781f1/db5770f0-9bbb-45ec-97fc-82b095c781f1-partition.bin
        ;;
    *)
        echo "Error: Invalid parameter '$1'"
        echo "Available options: 1, 2, 3, 4"
        exit 1
        ;;
esac    

echo "Flash write completed!"