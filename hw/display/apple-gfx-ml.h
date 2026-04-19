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
#include "qemu/typedefs.h"
#include "qemu/thread.h"      /* QemuThread, QemuSemaphore, QemuMutex */

/* Forward declaration - defined in qmetal unified API */
typedef struct qmu_session qmu_session;
struct AppleGfxMLFrameCompletionJob;
struct AgfxBootstrapPresentCommand;
typedef struct ThreadPool ThreadPool;

typedef struct AgfxBootstrapPresentSource {
    bool pending;
} AgfxBootstrapPresentSource;

typedef struct AgfxBootstrapPresentTimer {
    bool active;
    int64_t next_fire_us;
} AgfxBootstrapPresentTimer;

typedef struct AgfxCompletionJob {
    struct AgfxCompletionJob *next;
    void (*fn)(void *);
    void *ctx;
} AgfxCompletionJob;

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
    QEMUCursor *cursor;
    bool cursor_show;
    uint32_t cursor_display_id;
    uint32_t cursor_x;
    uint32_t cursor_y;

    /* display_fb is published to the QEMU surface from main-loop BHs only.
     * Completed frame payloads stay frame-owned until the matching completion
     * BH copies them into display_fb, instead of going through one shared
     * staging buffer that later callbacks can overwrite. */
    uint8_t *display_fb;        /* Used by QEMU DisplaySurface */
    size_t display_fb_size;
    
    /* Current display parameters */
    uint32_t fb_width;
    uint32_t fb_height;
    uint32_t fb_stride;
    uint32_t rendering_frame_width;
    uint32_t rendering_frame_height;
    uint32_t fb_iosurface_pixel_format;
    uint64_t fb_protection_requirements;
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

    /* MMIO job queue owned by the wrapper. */
    struct AppleGfxMLSessionJob *session_job_head;
    struct AppleGfxMLSessionJob *session_job_tail;
    QemuMutex mmio_job_mutex;  /* Protects MMIO session job queue */
    QemuMutex session_mutex;   /* Serializes wrapper-owned owner-render capture/submit */
    int mmio_wait_active;      /* Main thread is inside AIO_WAIT_WHILE for MMIO */

    /* Reference PGDisplayDescriptor.queue is one serial host queue for display
     * callbacks. schedule_display_completion is the wrapper-side analogue of
     * that queue boundary, so keep its jobs in one FIFO main-loop drain instead
     * of independent oneshot BH deliveries. */
    QemuMutex completion_mutex;
    AgfxCompletionJob *completion_head;
    AgfxCompletionJob *completion_tail;
    bool completion_bh_scheduled;
    QEMUBH *completion_bh;

    int iosfc_bootstrap_active;

    /* Reference apple_gfx_render_new_frame dispatches one async owner-render
     * block per captured frame onto the background queue. Mirror that with one
     * wrapper-owned thread pool instead of a serial render worker plane. */
    ThreadPool *render_pool;

    /* Reference PGEFIPresentQueue: serial bootstrap present queue owning both
     * scheduleFramePresents' 100ms timer source and the mergeable present
     * source. */
    QemuThread bootstrap_present_worker;
    QemuMutex bootstrap_present_mutex;
    QemuCond bootstrap_present_cond;
    bool bootstrap_present_worker_stop;
    struct AgfxBootstrapPresentCommand *bootstrap_present_cmd_head;
    struct AgfxBootstrapPresentCommand *bootstrap_present_cmd_tail;
    AgfxBootstrapPresentTimer bootstrap_present_timer;
    AgfxBootstrapPresentSource bootstrap_present_source;

    /* Async log sink: hot paths enqueue formatted lines, one worker serializes
     * qemu_log() writes off the producer threads. */
    QemuThread log_writer;
    QemuSemaphore log_sem;
    QemuMutex log_mutex;
    bool log_writer_started;
    bool log_writer_stop;
    struct AgfxLogEntry *log_head;
    struct AgfxLogEntry *log_tail;
    uint32_t log_depth;
    uint64_t log_dropped;

    uint64_t frame_completed_log_count;
    uint64_t new_frame_handler_log_count;
    uint64_t new_frame_signal_log_count;
    uint64_t render_worker_log_count;
    uint64_t bootstrap_present_log_count;
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
