# get IDF version for comparison
set(idf_version "${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}")

set(COMPONENT_REQUIRES driver)

set(priv_include_dirs
  sensors/private_include
)
set(include_dirs
  driver/include
)
set(srcs
  driver/esp_camera_sensor.c
  driver/esp_cam_io_parl.c
  driver/sccb.c
  driver/sensor.c
  driver/xclk.c

  sensors/ov2640.c
  sensors/ov3660.c
  sensors/ov5640.c
)

set(priv_requires esp_timer freertos nvs_flash)

idf_component_register(
  SRCS ${srcs}
  INCLUDE_DIRS ${include_dirs}
  PRIV_INCLUDE_DIRS ${priv_include_dirs}
  REQUIRES driver
  PRIV_REQUIRES ${priv_requires}
)
