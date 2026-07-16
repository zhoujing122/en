# Pre-consolidation source inventory (2026-07-16)

## Scope and safety boundary

This inventory records the local `robot_slamd` source branches archived before any
single-codebase consolidation. It is descriptive only: no merge, rebase,
cherry-pick, algorithm integration, history rewrite, or source deletion was
performed.

The repository origin was verified as `git@github.com:zhoujing122/en.git`. The
repository is publicly visible, and every branch below was created or updated by
a normal, non-forced push. Remote SHAs were checked with `git ls-remote` after
each push.

`Ahead` and `behind` are measured as `<branch>...49cb33e`, with the branch on the
left. A formal checkpoint is a completed development-stage snapshot; only
`m3-d3b-relocalization-recovery` is the latest complete M3-D3B line. The WIP
archive is intentionally not integrated into that line.

## Published source branches

| Remote branch | Full HEAD SHA | Development stage | Classification | Archived uncommitted source | Ahead | Behind | Common ancestor with `49cb33e` |
|---|---|---|---|---:|---:|---:|---|
| `main` | `2bc7f471d9df3916c377bc04aad92b3dff80f7bd` | M3-D1 sparse three-ToF occupancy baseline | Formal checkpoint | No | 0 | 26 | `2bc7f471d9df3916c377bc04aad92b3dff80f7bd` |
| `m3-d1.1-runtime-wiring` | `cd3c24b58b129c8cadca17da9d420b1d70af3707` | M3-D1.1 runtime wiring | Formal checkpoint | No | 0 | 24 | `cd3c24b58b129c8cadca17da9d420b1d70af3707` |
| `m3-d2a0-initialization-frames-timed-pose` | `d83573c5ea19efba237629b3d2e51b3b5564d7a5` | M3-D2A0 initialization, frames, timed pose | Formal checkpoint | No | 0 | 21 | `d83573c5ea19efba237629b3d2e51b3b5564d7a5` |
| `m3-d2a1-phase-aware-observation-bundle` | `e348a30f9a25faab2e1ebcc4f552894119f925f5` | M3-D2A1 phase-aware observations | Formal checkpoint | No | 0 | 18 | `e348a30f9a25faab2e1ebcc4f552894119f925f5` |
| `m3-d2b0-reference-map-matcher-contracts` | `0e22ff38a8917d5a76c8669d1f5f0e333f43af1b` | M3-D2B0 reference-map and matcher contracts | Formal checkpoint | No | 0 | 15 | `0e22ff38a8917d5a76c8669d1f5f0e333f43af1b` |
| `m3-d2b1-yaw-only-local-matcher` | `dc655d27835a8f182672cc58046a10c96856a95d` | M3-D2B1 bounded yaw-only matcher | Formal checkpoint | No | 0 | 13 | `dc655d27835a8f182672cc58046a10c96856a95d` |
| `m3-d2c-atomic-correction-keyframe-commit` | `b489e2d275937002c510777d7a151f666022445a` | M3-D2C atomic correction/keyframe commit | Formal checkpoint | No | 0 | 11 | `b489e2d275937002c510777d7a151f666022445a` |
| `m3-d3a-extrinsics-map-lifecycle` | `380053d2eb1546ab78b94a86f1e8951b85d9cede` | M3-D3A extrinsics and sparse-map lifecycle | Formal checkpoint | No | 0 | 9 | `380053d2eb1546ab78b94a86f1e8951b85d9cede` |
| `m3-e-simulation-autonomous-exploration` | `23df78a9c88cd91e8f6d7facffb2f59f44572699` | M3-E deterministic autonomous exploration | Formal checkpoint | No | 0 | 6 | `23df78a9c88cd91e8f6d7facffb2f59f44572699` |
| `m3-e1-exploration-quality` | `89944c82ee0f3697cab83749f6a75857def239df` | M3-E1 exploration quality and completion | Formal checkpoint | No | 0 | 4 | `89944c82ee0f3697cab83749f6a75857def239df` |
| `m3-d3b-relocalization-recovery` | `49cb33eaa755c56da6576da17580d1383a507761` | M3-D3B relocalization and lost recovery | Latest formal completion | No | 0 | 0 | `49cb33eaa755c56da6576da17580d1383a507761` |
| `archive/main-wip-20260716` | `ea3bf057737683826a69301c0a92601a343a1fab` | Alternate local M3-D2 matching/keyframe work | Experimental WIP snapshot | Yes | 1 | 26 | `2bc7f471d9df3916c377bc04aad92b3dff80f7bd` |

The review branch containing this document is intentionally not assigned its own
SHA in the table: embedding a commit's SHA in that same commit is self-referential.
Its verified remote SHA is recorded in the publication report.

## WIP snapshot contents

`archive/main-wip-20260716` preserves the original dirty `main` worktree without
committing in or cleaning that original worktree. The archive commit contains:

- three tracked edits: `CMakeLists.txt`, `docs/PROJECT_ROADMAP.md`, and
  `include/robot_slamd/mapping/sparse_tof/sparse_grid_types.hpp`;
- two source-related design/interface documents;
- ten localization, mapping, and deterministic backend headers;
- two test helper headers and twenty-one focused C++ tests.

In total, the WIP archive has 38 files changed, 3,749 insertions, and 4
deletions. The persistent backup is outside the repository at
`/home/lab/Desktop/小恩/recovery_backups/github_publish_20260716/main_wip/` and
contains binary tracked/cached patches, the untracked-file manifest, and checked
source copies. These backup artifacts were not pushed.

## Recommended review order

1. Use `m3-d3b-relocalization-recovery` as the complete, validated mainline
   reference through M3-D3B.
2. Review `archive/main-wip-20260716` separately for potentially useful alternate
   M3-D2 SE(2) matching, keyframe, deterministic backend, and test ideas. Do not
   merge it wholesale: it forked at `2bc7f47` and is 26 mainline commits behind.
3. Use the intermediate `m3-*` branches as audit checkpoints. They are ancestors
   of `49cb33e` and contain no commits unique from the latest complete branch.

## Excluded and scanned content

The following original-worktree generated directories were explicitly excluded:

- `build/`
- `board_map_preview_20210101_191144/`

No `build*`, `cmake-build*`, `Testing/`, `.cache/`, `compile_commands.json`, log,
core dump, bag, DB3, MCAP, sparse-map runtime output, patch, tar, backup, IDE cache,
or binary runtime artifact was included in an archive commit. No file larger than
50 MB exists in the published local Git objects or the archived WIP source set.

High-confidence scans found no OpenAI/GitHub/API tokens, authentication headers,
private-key markers, `.env` files, SSH keys, or private-key certificates in the
published history or WIP source. A broad keyword scan matched only the existing
`.dockerignore` rule for `**/secrets.dev.yaml`; it did not contain a credential.

## Key-stage commit coverage

Each required key commit is present on the named canonical remote checkpoint and
also on the later complete branch where indicated:

| Key commit | Canonical remote branch | Also contained by |
|---|---|---|
| `2bc7f471d9df3916c377bc04aad92b3dff80f7bd` | `main` | every `m3-*` branch, `archive/main-wip-20260716`, and this review branch |
| `cd3c24b58b129c8cadca17da9d420b1d70af3707` | `m3-d1.1-runtime-wiring` | all later formal `m3-*` branches and this review branch |
| `d83573c5ea19efba237629b3d2e51b3b5564d7a5` | `m3-d2a0-initialization-frames-timed-pose` | all later formal `m3-*` branches and this review branch |
| `e348a30f9a25faab2e1ebcc4f552894119f925f5` | `m3-d2a1-phase-aware-observation-bundle` | all later formal `m3-*` branches and this review branch |
| `0e22ff38a8917d5a76c8669d1f5f0e333f43af1b` | `m3-d2b0-reference-map-matcher-contracts` | all later formal `m3-*` branches and this review branch |
| `dc655d27835a8f182672cc58046a10c96856a95d` | `m3-d2b1-yaw-only-local-matcher` | all later formal `m3-*` branches and this review branch |
| `b489e2d275937002c510777d7a151f666022445a` | `m3-d2c-atomic-correction-keyframe-commit` | all later formal `m3-*` branches and this review branch |
| `380053d2eb1546ab78b94a86f1e8951b85d9cede` | `m3-d3a-extrinsics-map-lifecycle` | later exploration/M3-D3B branches and this review branch |
| `23df78a9c88cd91e8f6d7facffb2f59f44572699` | `m3-e-simulation-autonomous-exploration` | `m3-e1-exploration-quality`, `m3-d3b-relocalization-recovery`, and this review branch |
| `89944c82ee0f3697cab83749f6a75857def239df` | `m3-e1-exploration-quality` | `m3-d3b-relocalization-recovery` and this review branch |
| `49cb33eaa755c56da6576da17580d1383a507761` | `m3-d3b-relocalization-recovery` | this review branch |

No required commit was unreferenced, and `git fsck --full --no-reflogs
--unreachable` found no unreachable commit or tree requiring a recovered branch.
