# CI flakes

Failures seen once on CI that did not reproduce, kept so a second occurrence
is recognised. A second occurrence of an entry gets its own fix PR.

## tst_waterfield: SEGFAULT on the Windows row (1 occurrence)

- Row: `Windows build + tests (MSYS2 UCRT64, GCS-only)`.
- Seen: 2026-11-25, workflow run 45686598681. ctest reported SEGFAULT for
  `tst_waterfield` at about 0.16 s; the workflow's individual rerun of that
  executable then passed 15 of 15. The PR under test did not compile any file
  `tst_waterfield` uses.
- Not reproduced: 40 of 40 on Linux (20 idle, 20 with the CPU starved). Linux
  only, so it says nothing about the MinGW build.
- Same head, rerun of the failed job only: green.
- Suspicion, unproven: `thread_local` handling in the MinGW runtime.
- If it recurs: capture the crashing test's own output (ctest swallows it; the
  workflow's rerun prints it) and open a fix PR.
