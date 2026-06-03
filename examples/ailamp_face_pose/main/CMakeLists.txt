set(src_dirs        ./)

set(include_dirs    ./)

set(requires who_spiflash_fatfs
             who_recognition_app
             human_face_detect
             esp-dl
             driver
             esp_timer
             esp_http_server
             esp_wifi
             esp_netif
             esp_event
             nvs_flash)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})




