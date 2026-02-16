/*
 * Apple Paravirtualized Graphics - QEMU PCI Device (Minimal)
 *
 * This is the QEMU-side device that:
 * - Registers PCI device with proper IDs
 * - Sets up MMIO BAR and proxies to qmetal unified
 * - Handles interrupts and display
 * - Loads OptionROM via inherited romfile property
 *
 * All protocol logic is in qmetal library (libmlapi).
 */

#ifndef HW_DISPLAY_APPLE_GFX_ML_H
#define HW_DISPLAY_APPLE_GFX_ML_H

#include "qom/object.h"
#include "hw/pci/pci_device.h"
#include <pthread.h>  /* v96: for pthread_mutex_t */

/* Forward declaration - defined in qmetal unified API */
typedef struct qmu_session qmu_session;

#define TYPE_APPLE_GFX_ML "apple-gfx-ml"
OBJECT_DECLARE_SIMPLE_TYPE(AppleGfxMLState, APPLE_GFX_ML)

/* PCI IDs - must match guest driver expectations */
#define APPLE_GFX_ML_VENDOR_ID          0x106b  /* Apple */
#define APPLE_GFX_ML_DEVICE_ID          0xeeee  /* Paravirt GPU */
#define APPLE_GFX_ML_CLASS_ID           0x0380  /* Display controller */
/* v88: Removed REVISION, SUBSYSTEM_VENDOR, SUBSYSTEM_ID - Apple doesn't set them */

/* Memory regions */
#define APPLE_GFX_ML_MMIO_SIZE          (16 * 1024)         /* BAR0: 16KB (Apple standard) */
/* v88: Removed SHMEM BAR - Apple doesn't use it */
#define APPLE_GFX_ML_MSI_CAP_OFFSET     0x0   /* v87: Auto-select (Apple style) */
#define APPLE_GFX_ML_DEFAULT_VRAM_MB    512

/* Device state - minimal, all logic in qmetal */
struct AppleGfxMLState {
    /*< private >*/
    PCIDevice parent_obj;

    /*< public >*/
    /* PCI memory regions */
    MemoryRegion mmio;      /* BAR0 - MMIO registers */
    MemoryRegion shmem;     /* BAR2 - Shared memory */
    MemoryRegion host_vram; /* Internal VRAM (not a BAR) */

    /* QMetal unified library handle - contains ALL protocol logic */
    qmu_session *qmu_dev;

    /* QEMU display */
    QemuConsole *con;

    /* v96: Double-buffered framebuffer for thread-safe display updates.
     * 
     * Problem: qmetal calls present_frame from pthread, but QEMU display
     * operations (dpy_gfx_replace_surface/dpy_gfx_update_full) must run
     * in QEMU main loop.
     * 
     * Solution: Use staging buffer + BH (bottom half) scheduling.
     * - staging_fb: receives pixels from qmetal pthread
     * - display_fb: used by QEMU DisplaySurface
     * - BH copies staging->display and updates QEMU display
     */
    uint8_t *staging_fb;        /* Receives pixels from pthread */
    size_t staging_fb_size;
    uint8_t *display_fb;        /* Used by QEMU DisplaySurface */
    size_t display_fb_size;
    
    /* Frame parameters (protected by frame_mutex) */
    pthread_mutex_t frame_mutex;
    uint32_t pending_width;
    uint32_t pending_height;
    uint32_t pending_stride;
    bool frame_pending;         /* New frame waiting to be displayed */
    
    /* Current display parameters */
    uint32_t fb_width;
    uint32_t fb_height;
    uint32_t fb_stride;
    bool has_rendered_frame;

    /* Configuration properties */
    uint32_t vram_size_mb;
    uint32_t display_width;
    uint32_t display_height;
    /* NOTE: romfile is inherited from PCIDevice, don't redefine! */
    char *renderer;         /* "vulkan" or "software" */
    bool vsync_enabled;
    uint32_t debug_level;
    bool direct_scanout;    /* v3.90b: Enable direct Vulkan scanout window */
    char *spirv_cache_dir;  /* SPIR-V + VkPipelineCache disk cache directory */

    /* Runtime state */
    bool msi_used;
    bool display_enabled;
    uint64_t frame_count;
    uint64_t present_count;     /* v96: Counter for present_frame calls */
    bool trace_dma;
    
    /* v83: Display refresh is now handled by qmetal library's internal thread */
};

/* Properties macro for device registration
 * NOTE: romfile is already defined by PCIDevice parent class!
 * Use: -device apple-gfx-ml,romfile=/path/to/AppleParavirtEFI.rom
 */
#define APPLE_GFX_ML_PROPS \
    DEFINE_PROP_UINT32("vram_size_mb", AppleGfxMLState, vram_size_mb, APPLE_GFX_ML_DEFAULT_VRAM_MB), \
    DEFINE_PROP_UINT32("xres", AppleGfxMLState, display_width, 1920), \
    DEFINE_PROP_UINT32("yres", AppleGfxMLState, display_height, 1080), \
    DEFINE_PROP_STRING("renderer", AppleGfxMLState, renderer), \
    DEFINE_PROP_BOOL("vsync", AppleGfxMLState, vsync_enabled, true), \
    DEFINE_PROP_UINT32("debug", AppleGfxMLState, debug_level, 0), \
    DEFINE_PROP_BOOL("direct_scanout", AppleGfxMLState, direct_scanout, false), \
    DEFINE_PROP_STRING("spirv_cache_dir", AppleGfxMLState, spirv_cache_dir)

#endif /* HW_DISPLAY_APPLE_GFX_ML_H */
