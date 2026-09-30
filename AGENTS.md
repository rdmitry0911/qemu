# QEMU host-stack instructions

## Main integration, run packages, and handoff

For the apple-virgl host stack, `main` is the starting point and integration
authority in each of AppleVirgl, QEMU, QMetal, and air2spirv.

After a causal layer is source-complete and has passed the applicable component
gates, merge its reviewed host commits into that repository's `main` and
immediately push `main` to the host-authoritative `origin`. Record the merge
SHA, remote main SHA, and closure evidence. A completed layer must not remain
only in a feature branch or worktree. Record an exact push failure if publication
is blocked.

Build every package intended for a guest run or another agent from explicit,
committed `main` revisions of all its components. Record those source SHAs,
dependency revisions, generated-source provenance where applicable, and artifact
hashes in the package receipt. Candidate builds in working branches are for
pre-integration component checks; they are not the package for a guest run.

Runtime and visual verification use the resulting main-built package. Source
closure and a merge do not by themselves prove runtime or graphical parity;
record that verification separately against the exact package tuple.

Every handoff starts with the main SHAs and package/build/run receipts. List
unfinished work, its branch or worktree, evidence, and the next action separately
so the next agent can resume from main without reconstructing accepted layers.
If an existing repository lacks main or its accepted history is not reconciled,
record that migration gap before producing a package; do not silently use a
different branch as main.

## Existing-history migration

The initial main baseline is the already-published origin/master revision
ee63d467738a6025589425c0d9c3ff77f2739caf, not an unpublished candidate tip.
Creating main does not close the unconnected EEEE process-VM carrier or the
QMetal owning-session graph. Keep those candidates separate until their exact
source and component gates close, then integrate with an ancestry-preserving
merge. No guest runtime or graphical parity is claimed by this migration.
