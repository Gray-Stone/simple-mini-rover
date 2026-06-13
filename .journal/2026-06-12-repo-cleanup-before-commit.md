## 2026-06-12 repo cleanup before commit

- Object: clean commit surface before more docking work.
- Removed accidental repo-local Codex npm package files. Codex is now user-home level, not project dependency.
- `.gitignore` now hides `node_modules/`, live visual servo run logs, exposure sweeps, and provisional interpolated camera models.
- Remaining visible files are source/docs/journal changes that look intentional for commit.
- Verified Python tools compile and touched command-line tools can print `--help` without opening camera/serial.
