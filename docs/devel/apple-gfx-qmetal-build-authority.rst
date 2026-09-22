Apple GFX ML QMetal build authority
===================================

FIX_CANDIDATE
-------------

* Symptom and stage: the Apple GFX ML Meson stanza could discover a
  host-installed ``mlapi`` or derive a neighboring ``../../../qmetal`` source
  and build directory.  The QEMU/QMetal ABI was therefore not attributable to
  the reviewed source pair before any guest or graphics stage began.
* Reference expectation: the host-control tuple has one reviewed QEMU source,
  one reviewed QMetal source, and the library built from that QMetal source;
  the build must not select either endpoint implicitly.
* Actual behavior before this candidate: an implicit system dependency was
  attempted first and source-relative QMetal was a fallback.
* Divergence boundary: QEMU Meson dependency selection, before device
  realization, MMIO, task translation, QEMU presentation, or guest execution.
* Runtime evidence: none is admissible while this source-recoverable boundary
  is open.  This candidate is deliberately source/configure-only.
* Source evidence: the active host assembly auditor requires
  ``apple_virgl_qmetal_source_dir`` pairing, and prior recorded QEMU build
  receipts already carry the paired source/build options.  The current source
  previously did not enforce them.
* Root cause: QMetal's headers and ``libmlapi.so`` jointly define the host ABI;
  discovery or a relative fallback can select a different pair without a
  reviewed QEMU change.  The cause is build-pair ownership, not a rendering
  symptom.
* Proposed closure: when ``CONFIG_APPLE_GFX_ML`` is enabled, require absolute
  source and build paths, require the unified/PVG headers and exact shared
  library in that pair, and link that library by its explicit path.  Reject all
  system, name-based, and source-relative fallback paths.
* Verification plan: run the static verifier, then configure an x86_64 Apple
  GFX ML build with an explicit local QMetal pair and confirm that the resulting
  device configuration and Meson option set retain both paths.  A later build
  receipt must bind the QEMU revision, QMetal revision, header hashes, library
  hash, and these two options before any ``eeee`` runtime phase.

This closes build-pair selection only.  It does not claim closure of the
physical session, callbacks, routes, task aliases, presentation, guest driver,
or visual parity.
