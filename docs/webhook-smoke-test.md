# Reviewer webhook smoke test

This sample pull request checks webhook delivery for FTC team 15707 Bots By
the Creek, targeting `15707_Main_Coding_2026-2027`.

Expected intake behavior:

- Opening a draft pull request queues one build job.
- Marking it ready for review also queues a teaching-review job.
- New commits supersede queued work for older commits.
- Closing the pull request cancels its queued work.

The deployed reviewer currently receives and stores events. Automatic build
execution and review publishing are still being connected.

This documentation-only sample can be closed after testing without merging.
