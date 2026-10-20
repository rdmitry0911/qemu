hw_display_modules = {}


system_ss.add(when: 'CONFIG_DDC', if_true: files('i2c-ddc.c'))
system_ss.add(when: 'CONFIG_EDID', if_true: files('edid-generate.c', 'edid-region.c'))

system_ss.add(when: 'CONFIG_FW_CFG_DMA', if_true: files('ramfb.c'), if_false: files('ramfb-stubs.c'))
system_ss.add(when: 'CONFIG_FW_CFG_DMA', if_true: files('ramfb-standalone.c'))

system_ss.add(when: 'CONFIG_VGA_CIRRUS', if_true: files('cirrus_vga.c'))
system_ss.add(when: ['CONFIG_VGA_CIRRUS', 'CONFIG_VGA_ISA'], if_true: files('cirrus_vga_isa.c'))
system_ss.add(when: 'CONFIG_G364FB', if_true: files('g364fb.c'))
system_ss.add(when: 'CONFIG_JAZZ_LED', if_true: files('jazz_led.c'))
system_ss.add(when: 'CONFIG_PL110', if_true: files('pl110.c'))
system_ss.add(when: 'CONFIG_SII9022', if_true: files('sii9022.c'))
system_ss.add(when: 'CONFIG_SSD0303', if_true: files('ssd0303.c'))
system_ss.add(when: 'CONFIG_SSD0323', if_true: files('ssd0323.c'))
system_ss.add(when: 'CONFIG_XEN_BUS', if_true: files('xenfb.c'))

system_ss.add(when: 'CONFIG_VGA_PCI', if_true: files('vga-pci.c'))
system_ss.add(when: 'CONFIG_VGA_ISA', if_true: files('vga-isa.c'))
system_ss.add(when: 'CONFIG_VGA_MMIO', if_true: files('vga-mmio.c'))
system_ss.add(when: 'CONFIG_VMWARE_VGA', if_true: files('vmware_vga.c'))
system_ss.add(when: 'CONFIG_BOCHS_DISPLAY', if_true: files('bochs-display.c'))

system_ss.add(when: 'CONFIG_EXYNOS4', if_true: files('exynos4210_fimd.c'))
system_ss.add(when: 'CONFIG_FRAMEBUFFER', if_true: files('framebuffer.c'))

system_ss.add(when: 'CONFIG_RASPI', if_true: files('bcm2835_fb.c'))
system_ss.add(when: 'CONFIG_SM501', if_true: files('sm501.c'))
system_ss.add(when: 'CONFIG_TCX', if_true: files('tcx.c'))
system_ss.add(when: 'CONFIG_CG3', if_true: files('cg3.c'))
system_ss.add(when: 'CONFIG_MACFB', if_true: files('macfb.c'))
system_ss.add(when: 'CONFIG_NEXTCUBE', if_true: files('next-fb.c'))

system_ss.add(when: 'CONFIG_VGA', if_true: files('vga.c'))
system_ss.add(when: 'CONFIG_VIRTIO', if_true: files('virtio-dmabuf.c'))
system_ss.add(when: 'CONFIG_DM163', if_true: files('dm163.c'))

if (config_all_devices.has_key('CONFIG_VGA_CIRRUS') or
    config_all_devices.has_key('CONFIG_VGA_PCI') or
    config_all_devices.has_key('CONFIG_VMWARE_VGA') or
    config_all_devices.has_key('CONFIG_ATI_VGA')
   )
  system_ss.add(when: 'CONFIG_ACPI', if_true: files('acpi-vga.c'),
                                      if_false: files('acpi-vga-stub.c'))
endif

if config_all_devices.has_key('CONFIG_QXL')
  qxl_ss = ss.source_set()
  qxl_ss.add(when: 'CONFIG_QXL', if_true: [files('qxl.c', 'qxl-logger.c', 'qxl-render.c'),
                                           pixman, spice])
  qxl_ss.add(when: 'CONFIG_ACPI', if_true: files('acpi-vga.c'),
                                  if_false: files('acpi-vga-stub.c'))
  hw_display_modules += {'qxl': qxl_ss}
endif

system_ss.add(when: 'CONFIG_DPCD', if_true: files('dpcd.c'))
system_ss.add(when: 'CONFIG_XLNX_DISPLAYPORT', if_true: files('xlnx_dp.c'))

system_ss.add(when: 'CONFIG_ARTIST', if_true: files('artist.c'))

system_ss.add(when: 'CONFIG_ATI_VGA', if_true: [files('ati.c', 'ati_2d.c', 'ati_dbg.c'), pixman])

# Apple ParavirtualizedGraphics QEMU Device (QMetal Unified Backend)
#
# Links with:
#   - mlapi (qmetal): Metal→Vulkan translation with unified API
#
# To use in QEMU:
# 1. Copy apple-gfx-ml.h and apple-gfx-ml.c to hw/display/
# 2. Add this content to hw/display/meson.build
# 3. Add Kconfig entry to hw/display/Kconfig
# 4. Build QEMU with Vulkan support

if host_os == 'darwin'
  system_ss.add(
    when: [pvg, 'CONFIG_MAC_PVG_PCI'],
    if_true: files('apple-gfx.m', 'apple-gfx-pci.m')
  )
  system_ss.add(
    when: [pvg, 'CONFIG_MAC_PVG_MMIO'],
    if_true: files('apple-gfx.m', 'apple-gfx-mmio.m')
  )
else

    # hw/vmapple/meson.build

    apple_gfx_ml_ss = ss.source_set()
    apple_gfx_ml_ss.add(files(
        'apple-gfx-ml.c',
        'apple-gfx-eeee-process-vm.c',
    ))

    # ──────────────── mlapi/qmetal unified ────────────────
    # This device is valid only with the reviewed QMetal source and its exact
    # build output.  A host-installed mlapi or a neighboring checkout would
    # make the QEMU/QMetal ABI unprovable, so neither is a fallback.
    if config_all_devices.has_key('CONFIG_APPLE_GFX_ML')
        qmetal_source_dir = get_option('apple_virgl_qmetal_source_dir')
        qmetal_build_dir = get_option('apple_virgl_qmetal_build_dir')
        if qmetal_source_dir == '' or qmetal_build_dir == ''
            error('apple-gfx-ml requires explicit -Dapple_virgl_qmetal_source_dir ' +
                  'and -Dapple_virgl_qmetal_build_dir')
        endif
        if not qmetal_source_dir.startswith('/') or not qmetal_build_dir.startswith('/')
            error('Apple GFX ML QMetal source and build directories must be absolute')
        endif
        qmetal_header = qmetal_source_dir / 'include/qmu/qmetal_unified.h'
        qmetal_pvg_header = qmetal_source_dir / 'include/pvg/pvg_fifo.h'
        mlapi_lib_so = qmetal_build_dir / 'libmlapi.so'
        if not fs.is_dir(qmetal_source_dir) or not fs.is_dir(qmetal_build_dir) or \
           not fs.exists(qmetal_header) or not fs.exists(qmetal_pvg_header) or not fs.exists(mlapi_lib_so)
            error('Apple GFX ML reviewed QMetal tuple is incomplete: source=' +
                  qmetal_source_dir + ' build=' + qmetal_build_dir)
        endif

        vulkan_dep = dependency('vulkan', required: true)
        mlapi_inc = include_directories(
            qmetal_source_dir / 'include',
            qmetal_source_dir / 'include/pvg'
        )
        mlapi_dep = declare_dependency(
            include_directories: mlapi_inc,
            dependencies: [vulkan_dep],
            link_args: [mlapi_lib_so, '-Wl,-rpath,' + qmetal_build_dir],
        )
        apple_gfx_ml_ss.add(mlapi_dep)
        message('Using reviewed Apple GFX ML QMetal tuple: ' + mlapi_lib_so)
    endif

    system_ss.add_all(when: 'CONFIG_APPLE_GFX_ML', if_true: apple_gfx_ml_ss)


endif

if config_all_devices.has_key('CONFIG_VIRTIO_GPU')
  virtio_gpu_ss = ss.source_set()
  virtio_gpu_ss.add(when: 'CONFIG_VIRTIO_GPU',
                    if_true: [files('virtio-gpu-base.c', 'virtio-gpu.c'), pixman])
  if host_os == 'linux'
    virtio_gpu_ss.add(files('virtio-gpu-udmabuf.c'))
  else
    virtio_gpu_ss.add(files('virtio-gpu-udmabuf-stubs.c'))
  endif
  virtio_gpu_ss.add(when: 'CONFIG_VHOST_USER_GPU', if_true: files('vhost-user-gpu.c'))
  hw_display_modules += {'virtio-gpu': virtio_gpu_ss}

  if virgl.found() and opengl.found()
    virtio_gpu_gl_ss = ss.source_set()
    virtio_gpu_gl_ss.add(when: ['CONFIG_VIRTIO_GPU', virgl, opengl],
                         if_true: [files('virtio-gpu-gl.c', 'virtio-gpu-virgl.c'), pixman, virgl])
    hw_display_modules += {'virtio-gpu-gl': virtio_gpu_gl_ss}
  endif

  if rutabaga.found()
    virtio_gpu_rutabaga_ss = ss.source_set()
    virtio_gpu_rutabaga_ss.add(when: ['CONFIG_VIRTIO_GPU', rutabaga],
                               if_true: [files('virtio-gpu-rutabaga.c'), pixman])
    hw_display_modules += {'virtio-gpu-rutabaga': virtio_gpu_rutabaga_ss}
  endif
endif

if config_all_devices.has_key('CONFIG_VIRTIO_PCI')
  virtio_gpu_pci_ss = ss.source_set()
  virtio_gpu_pci_ss.add(when: ['CONFIG_VIRTIO_GPU', 'CONFIG_VIRTIO_PCI'],
                        if_true: [files('virtio-gpu-pci.c'), pixman])
  virtio_gpu_pci_ss.add(when: ['CONFIG_VHOST_USER_GPU', 'CONFIG_VIRTIO_PCI'],
                        if_true: files('vhost-user-gpu-pci.c'))
  hw_display_modules += {'virtio-gpu-pci': virtio_gpu_pci_ss}

  if virgl.found() and opengl.found()
    virtio_gpu_pci_gl_ss = ss.source_set()
    virtio_gpu_pci_gl_ss.add(when: ['CONFIG_VIRTIO_GPU', 'CONFIG_VIRTIO_PCI', virgl, opengl],
                             if_true: [files('virtio-gpu-pci-gl.c'), pixman])
    hw_display_modules += {'virtio-gpu-pci-gl': virtio_gpu_pci_gl_ss}
  endif
  if rutabaga.found()
    virtio_gpu_pci_rutabaga_ss = ss.source_set()
    virtio_gpu_pci_rutabaga_ss.add(when: ['CONFIG_VIRTIO_GPU', 'CONFIG_VIRTIO_PCI', rutabaga],
                                   if_true: [files('virtio-gpu-pci-rutabaga.c'), pixman])
    hw_display_modules += {'virtio-gpu-pci-rutabaga': virtio_gpu_pci_rutabaga_ss}
  endif
endif

if config_all_devices.has_key('CONFIG_VIRTIO_VGA')
  virtio_vga_ss = ss.source_set()
  virtio_vga_ss.add(when: 'CONFIG_VIRTIO_VGA',
                    if_true: [files('virtio-vga.c'), pixman])
  virtio_vga_ss.add(when: 'CONFIG_VHOST_USER_VGA',
                    if_true: files('vhost-user-vga.c'))
  virtio_vga_ss.add(when: 'CONFIG_ACPI', if_true: files('acpi-vga.c'),
                                         if_false: files('acpi-vga-stub.c'))
  hw_display_modules += {'virtio-vga': virtio_vga_ss}

  if virgl.found() and opengl.found()
    virtio_vga_gl_ss = ss.source_set()
    virtio_vga_gl_ss.add(when: ['CONFIG_VIRTIO_VGA', virgl, opengl],
                         if_true: [files('virtio-vga-gl.c'), pixman])
    virtio_vga_gl_ss.add(when: 'CONFIG_ACPI', if_true: files('acpi-vga.c'),
                                              if_false: files('acpi-vga-stub.c'))
    hw_display_modules += {'virtio-vga-gl': virtio_vga_gl_ss}
  endif

  if rutabaga.found()
    virtio_vga_rutabaga_ss = ss.source_set()
    virtio_vga_rutabaga_ss.add(when: ['CONFIG_VIRTIO_VGA', rutabaga],
                               if_true: [files('virtio-vga-rutabaga.c'), pixman])
    virtio_vga_rutabaga_ss.add(when: 'CONFIG_ACPI', if_true: files('acpi-vga.c'),
                                                    if_false: files('acpi-vga-stub.c'))
    hw_display_modules += {'virtio-vga-rutabaga': virtio_vga_rutabaga_ss}
  endif
endif

system_ss.add(when: 'CONFIG_OMAP', if_true: files('omap_lcdc.c'))

modules += { 'hw-display': hw_display_modules }
