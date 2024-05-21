idf_component_register(
    SRCS
    "main.cpp"

    INCLUDE_DIRS
    "include"
)

set(image ../storage)
fatfs_create_spiflash_image(storage ${image} FLASH_IN_PROJECT PRESERVE_TIME)