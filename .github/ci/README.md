# Robot verification checks

PRs into `15707_Main_Coding_2026-2027` run **FTC Compile**, **FTC Unit Tests**, and
**FTC Lint** on separate fresh GitHub-hosted Ubuntu VMs, including draft PRs.
The checks use the PR combined with its destination branch. Source commit IDs
are recorded before executing the project's Gradle code.

The workflow pins its actions and SDK archive and uses JDK 17, Gradle 8.13,
Android SDK 34, and Build Tools 35.0.0. The GitHub token is read-only and checkout
credentials are removed before building. No team secrets, Codex credentials,
robot devices, or production signing keys are provided. Gradle caches are not
shared across jobs. Logs/XML reports are retained for one day; APKs are not
uploaded.

Unit-test counts and lint diagnostics appear in the check steps and job summary.
**No tests found** is reported as no coverage, even when Gradle exits successfully.
Known lint warnings remain visible. Compiler/test/lint failures fail their check.
New commits cancel earlier in-progress checks for the same PR.

Title/body edits do not repeat builds. Retargeting does. If the destination
branch advances, old results do not establish that the new candidate merge
passes; update the PR branch to run verification again. Configure required checks
and an up-to-date-branch policy only after reviewing and merging this workflow.

CI source/report data are observations, not proof of hardware behavior or
competition-rule compliance. Changes to this workflow or its check driver need
mentor review; the local reviewer also verifies their trusted fingerprints.

Title/body-only edits use a separate concurrency group, so their skipped jobs
do not cancel a real build already running for the same PR.

The revision gate verifies the candidate merge's first parent against the current
fixed destination branch on the public remote. PR event `base.sha` may be stale
for an older open PR after the target advances. A stale event value is disclosed
as a notice; reports use the actual tested base. A candidate merge against an
older target still fails with a rerun message. Regression tests exercise both
cases, including PR #35's original revisions.
