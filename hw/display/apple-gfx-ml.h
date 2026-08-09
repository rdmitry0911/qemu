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
#include "qemu/notify.h"

/* Forward declaration - defined in qmetal unified API */
typedef struct qmu_session qmu_session;
struct AppleGfxMLFrameCompletionJob;
struct AgfxBootstrapPresentCommand;
typedef struct ThreadPool ThreadPool;

#define AGFX_GUEST_SCHED_APV_EVENT_WP_SEEN_MAX 64

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
    int32_t cursor_x;
    int32_t cursor_y;

    /* display_fb is published to the QEMU surface from main-loop BHs only.
     * Owner completion BH late-reads the latest mutable frame state from
     * qmetal on the BH edge, mirroring reference frame_completed_bh reading
     * from one shared display texture object instead of consuming immutable
     * per-submit frame payload snapshots. */
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
    bool exiting;
    bool shutdown_notifier_registered;
    Notifier shutdown_notifier;
    bool msi_used;
    bool display_enabled;
    uint64_t frame_count;
    uint64_t present_count;     /* Counter for present_frame calls */
    uint64_t irq_count;         /* Counter for IRQ deliveries */
    uint64_t stamp_irq_request_seq;
    uint64_t stamp_irq_delivery_seq;
    uint64_t stamp_irq_pending_bh;
    uint64_t stamp_irq_requests_since_event_read;
    uint64_t stamp_irq_deliveries_since_event_read;
    uint64_t stamp_irq_event_read_seq;
    uint64_t stamp_irq_display_irq_read_seq;
    uint64_t guest_sched_callsite_seq;
    uint64_t guest_sched_event_read_seq;
    uint64_t guest_sched_display_irq_read_seq;
    uint64_t guest_sched_last_event_read_seq;
    uint64_t guest_sched_last_event_read_value;
    uint64_t guest_sched_last_display_irq_read_seq;
    uint64_t guest_sched_last_display_irq_read_value;
    uint64_t guest_sched_kick_seq;
    uint64_t guest_sched_last_kick_seq;
    uint64_t guest_sched_last_kick_channel;
    uint64_t guest_sched_last_kick1_seq;
    uint64_t guest_sched_last_kick2_seq;
    uint64_t guest_sched_last_kick5_seq;
    uint64_t guest_sched_ch5_interval_id;
    uint64_t guest_sched_last_fifo_read_value;
    uint64_t guest_sched_last_fifo_write_value;
    uint64_t guest_sched_render_sample_interval;
    uint64_t guest_sched_render_sample_kick1_count;
    uint64_t guest_sched_render_sample_kick2_count;
    uint64_t guest_sched_render_sample_event_marker;
    uint64_t guest_sched_render_sample_event_remaining;
    uint64_t guest_sched_apv_entry_bp_addr;
    uint64_t guest_sched_apv_entry_bp_arm_seq;
    uint64_t guest_sched_apv_entry_bp_hit_seq;
    uint64_t guest_sched_apv_entry_bp_arm_ch5_interval;
    uint64_t guest_sched_apv_entry_bp_last_param;
    uint64_t guest_sched_apv_entry_bp_last_event_mask;
    uint64_t guest_sched_apv_entry_bp_last_event_e0;
    uint64_t guest_sched_apv_entry_bp_last_event_e1;
    uint64_t guest_sched_apv_entry_bp_last_event_e2;
    uint64_t guest_sched_apv_entry_bp_last_event_e3;
    uint32_t guest_sched_apv_entry_bp_armed;
    uint64_t guest_sched_apv_begin_bp_addr;
    uint64_t guest_sched_apv_begin_bp_arm_seq;
    uint64_t guest_sched_apv_begin_bp_hit_seq;
    uint64_t guest_sched_apv_begin_bp_arm_ch5_interval;
    uint64_t guest_sched_apv_begin_bp_last_event_ptr;
    uint64_t guest_sched_apv_begin_bp_last_event_mask;
    uint64_t guest_sched_apv_begin_bp_last_event_e0;
    uint32_t guest_sched_apv_begin_bp_armed;
    uint64_t guest_sched_apv_validate_bp_addr;
    uint64_t guest_sched_apv_validate_bp_arm_seq;
    uint64_t guest_sched_apv_validate_bp_hit_seq;
    uint64_t guest_sched_apv_validate_bp_arm_ch5_interval;
    uint64_t guest_sched_apv_validate_bp_last_param;
    uint64_t guest_sched_apv_validate_bp_last_event_mask;
    uint64_t guest_sched_apv_validate_bp_last_event_e0;
    uint32_t guest_sched_apv_validate_bp_armed;
    uint64_t guest_sched_apv_base_cached;
    uint64_t guest_sched_exec3_author_bp_base;
    uint64_t guest_sched_exec3_author_bp_addr;
    uint64_t guest_sched_exec3_author_bp_arm_seq;
    uint64_t guest_sched_exec3_author_bp_hit_seq;
    uint64_t guest_sched_exec3_author_bp_arm_ch5_interval;
    uint64_t guest_sched_exec3_author_rearm_bp_addr;
    uint32_t guest_sched_exec3_author_rearm_bp_armed;
    uint32_t guest_sched_exec3_author_bp_armed;
    uint64_t guest_sched_submit_buffer_bp_base;
    uint64_t guest_sched_submit_buffer_bp_addr;
    uint64_t guest_sched_submit_buffer_bp_arm_seq;
    uint64_t guest_sched_submit_buffer_bp_hit_seq;
    uint64_t guest_sched_submit_buffer_bp_arm_ch5_interval;
    uint64_t guest_sched_submit_buffer_rearm_bp_addr;
    uint32_t guest_sched_submit_buffer_rearm_bp_armed;
    uint32_t guest_sched_submit_buffer_bp_armed;
    uint64_t guest_sched_pcb_call_bp_base;
    uint64_t guest_sched_pcb_call_bp_addr;
    uint64_t guest_sched_pcb_call_bp_arm_seq;
    uint64_t guest_sched_pcb_call_bp_hit_seq;
    uint64_t guest_sched_pcb_call_bp_arm_ch5_interval;
    uint64_t guest_sched_pcb_call_rearm_bp_addr;
    uint32_t guest_sched_pcb_call_rearm_bp_armed;
    uint32_t guest_sched_pcb_call_bp_armed;
    uint64_t guest_sched_pcb_parse_seg_bp_base;
    uint64_t guest_sched_pcb_parse_seg_bp_addr;
    uint64_t guest_sched_pcb_parse_seg_return_bp_addr;
    uint64_t guest_sched_pcb_parse_seg_bp_arm_seq;
    uint64_t guest_sched_pcb_parse_seg_bp_hit_seq;
    uint64_t guest_sched_pcb_parse_seg_bp_arm_ch5_interval;
    uint64_t guest_sched_pcb_parse_seg_last_hit_seq;
    uint64_t guest_sched_pcb_parse_seg_last_rbp;
    uint64_t guest_sched_pcb_parse_seg_last_this;
    uint64_t guest_sched_pcb_parse_seg_last_param;
    uint64_t guest_sched_pcb_parse_seg_last_segment_entry;
    uint64_t guest_sched_pcb_parse_seg_last_segment_shmem;
    uint64_t guest_sched_pcb_parse_seg_last_kernel_hash;
    uint64_t guest_sched_pcb_parse_seg_last_segment_hash;
    uint32_t guest_sched_pcb_parse_seg_return_bp_armed;
    uint32_t guest_sched_pcb_parse_seg_bp_armed;
    uint64_t guest_sched_submit_cbs_bp_base;
    uint64_t guest_sched_submit_cbs_bp_addr;
    uint64_t guest_sched_submit_cbs_bp_arm_seq;
    uint64_t guest_sched_submit_cbs_bp_hit_seq;
    uint64_t guest_sched_submit_cbs_bp_arm_ch5_interval;
    uint64_t guest_sched_submit_cbs_bp_scan_seq;
    uint64_t guest_sched_submit_cbs_bp_scan_last_ch5_interval;
    uint64_t guest_sched_submit_cbs_rearm_bp_addr;
    uint32_t guest_sched_submit_cbs_rearm_bp_armed;
    uint32_t guest_sched_submit_cbs_bp_armed;
    uint64_t guest_sched_segment_init_bp_base;
    uint64_t guest_sched_segment_init_bp_addr;
    uint64_t guest_sched_segment_init_bp_arm_seq;
    uint64_t guest_sched_segment_init_bp_hit_seq;
    uint64_t guest_sched_segment_init_bp_arm_ch5_interval;
    uint64_t guest_sched_segment_init_last_this;
    uint64_t guest_sched_segment_init_last_header;
    uint64_t guest_sched_segment_init_last_hit_seq;
    uint64_t guest_sched_segment_init_rearm_bp_addr;
    uint32_t guest_sched_segment_init_rearm_bp_armed;
    uint32_t guest_sched_segment_init_bp_armed;
    uint64_t guest_sched_segment_header_wp_addr;
    uint64_t guest_sched_segment_header_wp_len;
    uint64_t guest_sched_segment_header_wp_arm_seq;
    uint64_t guest_sched_segment_header_wp_hit_seq;
    uint64_t guest_sched_segment_header_wp_hits_in_arm;
    uint64_t guest_sched_segment_header_wp_arm_init_hit_seq;
    uint64_t guest_sched_segment_header_wp_arm_ch5_interval;
    uint64_t guest_sched_segment_header_wp_header;
    uint64_t guest_sched_segment_header_wp_kernel_addr;
    uint64_t guest_sched_segment_header_wp_kernel_header;
    uint64_t guest_sched_segment_header_wp_client_base;
    uint64_t guest_sched_segment_header_wp_client_header;
    uint64_t guest_sched_segment_header_wp_header_off;
    uint64_t guest_sched_segment_header_wp_arm_q0;
    uint64_t guest_sched_segment_header_wp_arm_q1;
    uint32_t guest_sched_segment_header_wp_seg_index;
    uint32_t guest_sched_segment_header_wp_res_index;
    uint32_t guest_sched_segment_header_wp_res_id;
    uint32_t guest_sched_segment_header_wp_armed;
    uint64_t guest_sched_metal_author_bp_addr;
    uint64_t guest_sched_metal_author_return_bp_addr;
    uint64_t guest_sched_metal_author_bp_arm_seq;
    uint64_t guest_sched_metal_author_bp_hit_seq;
    uint64_t guest_sched_metal_author_bp_arm_ch5_interval;
    uint64_t guest_sched_metal_author_last_hit_seq;
    uint64_t guest_sched_metal_author_last_arg0;
    uint64_t guest_sched_metal_author_last_arg1;
    uint64_t guest_sched_metal_author_last_arg2;
    uint64_t guest_sched_metal_author_last_arg3;
    uint64_t guest_sched_metal_author_last_ret;
    uint32_t guest_sched_metal_author_kind;
    uint32_t guest_sched_metal_author_return_bp_armed;
    uint32_t guest_sched_metal_author_bp_armed;
    uint64_t guest_sched_apv_event_wp_base;
    uint64_t guest_sched_apv_event_wp_addr;
    uint64_t guest_sched_apv_event_wp_arm_seq;
    uint64_t guest_sched_apv_event_wp_hit_seq;
    uint64_t guest_sched_apv_event_wp_arm_ch5_interval;
    uint64_t guest_sched_apv_event_wp_arm_entry_hit_seq;
    uint64_t guest_sched_apv_event_wp_hits_in_arm;
    uint64_t guest_sched_apv_event_wp_seen_count;
    uint64_t guest_sched_apv_event_wp_seen[
        AGFX_GUEST_SCHED_APV_EVENT_WP_SEEN_MAX];
    uint32_t guest_sched_apv_event_wp_armed;
    uint64_t guest_sched_apv_inner_bp_base;
    uint64_t guest_sched_apv_inner_bp_arm_seq;
    uint64_t guest_sched_apv_inner_bp_hit_seq;
    uint32_t guest_sched_apv_inner_bp_armed;
    uint64_t guest_sched_iogpu_event_bp_base;
    uint64_t guest_sched_iogpu_event_bp_arm_seq;
    uint64_t guest_sched_iogpu_event_bp_hit_seq;
    uint32_t guest_sched_iogpu_event_bp_armed;
    uint64_t guest_sched_ioaccel_merge_bp_base;
    uint64_t guest_sched_ioaccel_merge_bp_addr;
    uint64_t guest_sched_ioaccel_merge_bp_arm_seq;
    uint64_t guest_sched_ioaccel_merge_bp_hit_seq;
    uint64_t guest_sched_ioaccel_merge_bp_arm_ch5_interval;
    uint64_t guest_sched_ioaccel_merge_bp_arm_validate_hit_seq;
    uint64_t guest_sched_ioaccel_merge_bp_tracked_param;
    uint64_t guest_sched_ioaccel_merge_bp_tracked_dst;
    uint32_t guest_sched_ioaccel_merge_bp_armed;
    uint64_t guest_sched_resource_life_bp_apv_base;
    uint64_t guest_sched_resource_life_bp_ioaccel_base;
    uint64_t guest_sched_resource_life_bp_arm_seq;
    uint64_t guest_sched_resource_life_bp_hit_seq;
    uint64_t guest_sched_resource_life_bp_hits_in_arm;
    uint64_t guest_sched_resource_life_bp_arm_ch5_interval;
    uint64_t guest_sched_resource_life_bp_arm_inner_hit_seq;
    uint64_t guest_sched_resource_life_bp_source_txn;
    uint64_t guest_sched_resource_life_bp_src_obj;
    uint64_t guest_sched_resource_life_bp_src_event;
    uint64_t guest_sched_resource_life_bp_src_e0;
    uint32_t guest_sched_resource_life_bp_src_lane;
    uint32_t guest_sched_resource_life_bp_armed;
    uint64_t guest_sched_resource_state_wp_obj;
    uint64_t guest_sched_resource_state_wp_state_addr;
    uint64_t guest_sched_resource_state_wp_ref_addr;
    uint64_t guest_sched_resource_state_wp_dirty_addr;
    uint64_t guest_sched_resource_state_wp_arm_seq;
    uint64_t guest_sched_resource_state_wp_hit_seq;
    uint64_t guest_sched_resource_state_wp_hits_in_arm;
    uint64_t guest_sched_resource_state_wp_arm_ch5_interval;
    uint64_t guest_sched_resource_state_wp_arm_inner_hit_seq;
    uint64_t guest_sched_resource_state_wp_source_txn;
    uint64_t guest_sched_resource_state_wp_src_event;
    uint64_t guest_sched_resource_state_wp_src_e0;
    uint32_t guest_sched_resource_state_wp_src_lane;
    uint32_t guest_sched_resource_state_wp_armed;
    uint64_t guest_sched_pcb_source_wp_addr;
    uint64_t guest_sched_pcb_source_wp_len;
    uint64_t guest_sched_pcb_source_wp_ptr;
    uint64_t guest_sched_pcb_source_wp_entries;
    uint64_t guest_sched_pcb_source_wp_arm_seq;
    uint64_t guest_sched_pcb_source_wp_hit_seq;
    uint64_t guest_sched_pcb_source_wp_hits_in_arm;
    uint64_t guest_sched_pcb_source_wp_arm_row_seq;
    uint64_t guest_sched_pcb_source_wp_arm_cmd_stamp;
    uint64_t guest_sched_pcb_source_wp_arm_ch5_interval;
    uint64_t guest_sched_pcb_source_wp_arm_q0;
    uint64_t guest_sched_pcb_source_wp_arm_q1;
    uint32_t guest_sched_pcb_source_wp_kind;
    uint32_t guest_sched_pcb_source_wp_slot;
    uint32_t guest_sched_pcb_source_wp_entry_index;
    uint32_t guest_sched_pcb_source_wp_res_id;
    uint32_t guest_sched_pcb_source_wp_armed;

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

    /* Reference PGDisplayDescriptor.queue publishes one callback item at a time
     * on a serial host queue. Keep FIFO ordering without collapsing several
     * queued items into one BH drain. */
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
