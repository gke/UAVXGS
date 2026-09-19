# Session Report: Log-File Debris & GitHub Push Block (Sep 13)

## Problem

`git push origin main` on the GitHub UAVXGS repo was rejected:

```
remote: error: GH001: Large files detected
remote: error: File macos/src/log.txt is 5295.75 MB; this exceeds GitHub's file size limit of 100.00 MB
```

The push was a small fast-forward (`5e810e7 Update 2026-08-25`, `1d0ad5a Update 2026-08-29` over `origin/main`), but those two commits carried a single **5,553,001,422-byte blob** (`9e6dfda7`) at `macos/src/log.txt` AND `windows/src/log.txt` (same blob, two paths; `git rev-list --objects` lists it once). GitHub counts the *uncompressed* file against the 100 MB limit, so the 122 MiB packed transfer was rejected pre-receive. A prior `RPC failed; curl 55` symptom was a red herring — the GH001 blobs are the real blocker.

## Propagation chain (root cause)

1. `core/packet_logger.py` → `PacketLogger.log()` writes **every** sent/received serial packet to `src_dir/log.txt` (append, unbounded) from `main_window` on both `log_sent` and `log_received`.
2. That grows `uavx-python/src/log.txt` to ~10 GB (measured: 10,046,387,509 B).
3. `scripts/sync_kits.sh` rsyncs `uavx-python/src/` into `linux/ macos/ windows/` — `log.txt` was **not** in the rsync exclude list.
4. All four `log.txt` files are **SVN-**versioned** (`NODES.presence = normal` in `.svn/wc.db`) — so they were committed to SVN at some point.
5. `scripts/update_repo.sh` does `svn export --force "$kit_src/" "$kit_repo/"` — svn export copies **versioned** files, re-injecting `log.txt` into the git repo.
6. `scripts/push_github.sh` does `git add -A && git commit && git push` → GH001 rejection.

So the git-history purge alone would NOT have fixed recurrence: every later update would re-inject the file from SVN. Three coordinated fixes required.

## Fixes applied

### 1. Git history rewritten (blobs dropped)
In `~/Documents/Flight/gitHub/UAVXGS`:

```
FILTER_BRANCH_SQUELCH_WARNING=1 git filter-branch -f \
  --index-filter 'git rm --cached --ignore-unmatch -- "*/log.txt" "log.txt" linux/src/log.txt macos/src/log.txt windows/src/log.txt' \
  HEAD~2..HEAD
```

- First pass removed only `macos/` + `windows/` log.txt — **left `linux/src/log.txt` pointing at the same giant blob** (the rev-list scan caught it still referenced). The second pass (the `*/log.txt` + linux explicit form) is the one that finished the job.
- Dropped `refs/original/refs/heads/main` (the filter-branch backup ref — it pinned the old chain), then `git reflog expire --expire=now --all && git gc --prune=now`.
- Verified: `git rev-list --objects --all | grep log.txt` → 0; no object >2 GB in any ref; `.git` 123 MB → **2.5 MB**; `main` ahead 2 / behind 0; largest blob in `main` tree = 210 KB (`windows/src/ui/parameter_window.py`); a full-pack of `main` measures ~2.25 MiB → push-safe.
- `origin/main` was already clean (0 log.txt) — the rejections never landed anything.

### 2. Runtime log files can never ship again (scripts)
- **NEW `scripts/sweep_logs.sh`**: deletes every `log.txt` / `*.log` under a root (physical `rm`), and when the file is still SVN-versioned does `svn rm --force` (also handles **already-missing** versioned nodes via `svn status '^!'`, so a stale tracked node can't wedge the next commit). Wired in as:
  - **Step 0** of `master_update.sh` (runs before both project cycles, over `$GS_DIR` + the gitHub repo).
  - **Step 0** of `update_all.sh` (before the SVN commit — the standalone-invocation path).
  - Inside `update_repo.sh` after the svn export loop (belt-and-braces: svn export copies unversioned files too).
  - Inside `push_github.sh` immediately before `git add -A` (last line of defence).
- `sync_kits.sh`: rsync now excludes `log.txt` and `*.log`.
- gitHub repo `.gitignore`: added `**/log.txt` and `**/*.log`.

### 3. Logger bounded (regeneration root cause)
`core/packet_logger.py`: added `_LOG_MAX_BYTES = 2 * 1024 * 1024`; once the file exceeds it the next write removes and restarts it, so it can never regrow to GBs. (The always-on `.rawlog` in `~/UAVX` is the lasting record — `log.txt` is only a per-packet diagnostic.)

### 4. On-disk debris removed
Deleted the four ~10 GB `log.txt` work copies (`uavx-python/src`, `linux/src`, `macos/src`, `windows/src`) → ~40 GB reclaimed. The live GCS session re-created `uavx-python/src/log.txt` at once (it holds the file open through the old code path); that file stays bounded from the next GCS launch onwards.

## Verification status

- `bash -n` clean on all six edited scripts; `py_compile` clean on `packet_logger.py`.
- Git repo: history clean, worktree clean (only the intended `.gitignore` edit pending), push range carries no object >50 MB.
- **Next action (user, on the host):** `git push origin main` (or run `master_update.sh` which commits the `.gitignore` edit then pushes) — expect success now.
- The four SVN-tracked `log.txt` nodes are now physically gone; the next host `master_update.sh` step-0 sweep will `svn rm --force` the missing nodes and commit the removal, so `update_repo.sh`'s svn export stops copying them.

## Decisions & rationale
- **History rewrite rather than force-push-with-deletions:** a plain `git rm` in the new commits would shrink the tracked tree but the 5.5 GB blob would remain in the object store and still be shipped in the pack on every push. Only purging it from the committed reachable objects (filter-branch + gc) stops GitHub from ever seeing it. `git filter-repo` was not installed; `filter-branch` over a 2-commit range is contained and safe here (no tags, linear history, remote base intact).
- **Sweep at step 0 of both master_update.sh and update_all.sh:** master_update is the documented entry point, but update_all.sh is also invocable standalone; putting the sweep before the SVN commit in both means the deletion itself is committed rather than fighting the export.
- **`scripts/sweep_logs.sh` does `svn rm --force`, not plain `rm`:**
  - plain `rm` would leave SVN showing `!` (missing) tracked nodes → the next `svn commit` fails;
  - `svn rm --force` records the deletion so the next commit removes them from version control (and therefore from all future svn exports).
  - The already-missing branch (svn status `!`) covers nodes deleted out-of-band before a sweep ever ran.
- **Logger bound at 2 MB (delete-and-restart, not rotate-and-keep):** the tail of a 2 MB diagnostic log is ample; keeping N generations adds clutter with no pushing benefit. `.rawlog` is the real archive.
- **`.gitignore` alone is insufficient** (files were committed before the ignore patterns existed, and svn export copies unversioned files too) — ignore + rsync exclude + sweep at three pipeline points together close every reinjection vector.

## Cross-reference
- AGENTS.md GCS Conventions → "Runtime log files (`log.txt` / `*.log`) are NEVER shipped or pushed (2026-09-13)".
- The `macos/src/log.txt` 5.5 GB blob and the four ~10 GB kit copies are gone; nothing in any git ref references a `log.txt` anymore.