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
#include "qemu/thread.h"      /* QemuThread, QemuSemaphore, QemuMutex */

/* Forward declaration - defined in qmetal unified API */
typedef struct qmu_session qmu_session;

#define TYPE_APPLE_GFX_ML "apple-gfx-ml"
OBJECT_DECLARE_SIMPLE_TYPE(AppleGfxMLState, APPLE_GFX_ML)

/* PCI IDs - must match guest driver expectations */
#define APPLE_GFX_ML_VENDOR_ID          0x106b  /* Apple */
#define APPLE_GFX_ML_DEVICE_ID          0xeeee  /* Paravirt GPU */
#define APPLE_GFX_ML_CLASS_ID           0x0380  /* Display controller */
/* Apple doesn't set REVISION, SUBSYSTEM_VENDOR, SUBSYSTEM_ID */

/* Memory regions */
#define APPLE_GFX_ML_MMIO_SIZE          (16 * 1024)         /* BAR0: 16KB (Apple standard) */
/* Apple doesn't use a shared memory BAR */
#define APPLE_GFX_ML_MSI_CAP_AUTO       0x0   /* Let QEMU auto-select capability offset */
#define APPLE_GFX_ML_DEFAULT_VRAM_MB    512

/* Device state - minimal, all logic in qmetal */
struct AppleGfxMLState {
    /*< private >*/
    PCIDevice parent_obj;

    /*< public >*/
    /* PCI memory regions */
    MemoryRegion mmio;      /* BAR0 - MMIO registers */
    MemoryRegion host_vram; /* Internal VRAM (not a BAR) */

    /* QMetal unified library handle - contains ALL protocol logic */
    qmu_session *qmu_dev;

    /* QEMU display */
    QemuConsole *con;

    /* Double-buffered framebuffer for thread-safe display updates.
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
    QemuMutex frame_mutex;
    uint32_t pending_width;
    uint32_t pending_height;
    uint32_t pending_stride;
    bool frame_pending;         /* New frame waiting to be displayed */

    /* Current display parameters */
    uint32_t fb_width;
    uint32_t fb_height;
    uint32_t fb_stride;
    bool new_frame_ready;       /* Frame copied to display_fb, awaiting gfx_update poll */
    bool gfx_update_requested;  /* gfx_update was called while frame was in-flight */
    int pending_frames;         /* Number of frames in flight (max 2, reference pattern) */

    /* Configuration properties */
    uint32_t vram_size_mb;
    uint32_t display_width;
    uint32_t display_height;
    /* NOTE: romfile is inherited from PCIDevice, don't redefine! */
    bool vsync_enabled;
    uint32_t debug_level;
    bool direct_scanout;    /* Enable direct Vulkan scanout window */
    char *spirv_cache_dir;  /* SPIR-V + VkPipelineCache disk cache directory */

    /* Runtime state */
    bool msi_used;
    bool display_enabled;
    uint64_t frame_count;
    uint64_t present_count;     /* Counter for present_frame calls */
    uint64_t irq_count;         /* Counter for IRQ deliveries */

    /* Async MMIO worker (replaces GCD dispatch_async_f from reference) */
    QemuThread mmio_worker;
    QemuSemaphore mmio_sem;        /* Wake worker when job available */
    bool mmio_worker_stop;          /* Signal worker to exit */

    /* Current MMIO job for worker (per-device, replaces global pointer) */
    struct AppleGfxMLMMIOJob *volatile pending_job;
    QemuMutex mmio_job_mutex;  /* Serializes MMIO dispatch (BQL released during AIO_WAIT_WHILE) */
};

/* Properties macro for device registration
 * NOTE: romfile is already defined by PCIDevice parent class!
 * Use: -device apple-gfx-ml,romfile=/path/to/AppleParavirtEFI.rom
 */
#define APPLE_GFX_ML_PROPS \
    DEFINE_PROP_UINT32("vram_size_mb", AppleGfxMLState, vram_size_mb, APPLE_GFX_ML_DEFAULT_VRAM_MB), \
    DEFINE_PROP_UINT32("xres", AppleGfxMLState, display_width, 1920), \
    DEFINE_PROP_UINT32("yres", AppleGfxMLState, display_height, 1080), \
    DEFINE_PROP_BOOL("vsync", AppleGfxMLState, vsync_enabled, true), \
    DEFINE_PROP_UINT32("debug", AppleGfxMLState, debug_level, 0), \
    DEFINE_PROP_BOOL("direct_scanout", AppleGfxMLState, direct_scanout, false), \
    DEFINE_PROP_STRING("spirv_cache_dir", AppleGfxMLState, spirv_cache_dir)

#endif /* HW_DISPLAY_APPLE_GFX_ML_H */
