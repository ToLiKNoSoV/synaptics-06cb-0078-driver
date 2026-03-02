# drivers/synaptics_0078/meson.build

driver_srcs = files(
  'synaptics_0078.c',
  'synaptics_0078_data.c',
)

synaptics_0078_driver = shared_module(
  'drv_synaptics_0078',
  driver_srcs,
  include_directories: includes,
  dependencies: [libfprint_deps, nbis_dep],
  link_with: libfprint,
  name_prefix: '',
  install: true,
  install_dir: drivers_dir,  # ← проверьте, что drivers_dir определён в главном meson.build
  c_args: ['-DG_LOG_DOMAIN="synaptics_0078"'],
)
