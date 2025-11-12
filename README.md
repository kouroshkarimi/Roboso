# Roboso

A ROS-based robotics project (work-in-progress) containing multiple packages for robot control and description. This repository currently includes the `roboso_controller` and `roboso_description` packages and is intended to be extended to use VLAs with several robots — especially the `so101` platform.

## Current status

- Minimal workspace layout present under `src/`:
  - `src/roboso_controller/`
  - `src/roboso_description/`
- Build artifacts present in `build/`, `install/`, and several `log/` folders (from colcon builds).
- This README is the initial project documentation and includes build + Git push instructions.

## Assumptions

- The term "VLA" is used in this project to mean "Vision / Localization / Autonomy" algorithms and integrations. If you mean a different expansion (for example, Variable-Length Arrays in C), tell me and I'll update the docs accordingly.
- This repo is a workspace root (Colcon/ROS layout): keep package sources under `src/` and avoid committing build/ install/ log/ directories.

## Goals

- Provide reusable ROS packages for controlling robots and describing their URDF/SRDF assets.
- Integrate VLAs (vision/localization/autonomy) for supported platforms, notably `so101`.
- Keep the repo CI-friendly, with clear instructions for building, testing and pushing to GitHub.

## Quickstart — local build

1. Source your ROS 2 distro environment (replace `humble` with your distro):

```bash
# Example (adjust to your distro and installation)
source /opt/ros/humble/setup.bash
# If using an overlay install, source local setup after building:
# source install/local_setup.bash
```

2. From the repository root (this directory):

```bash
# Create and build the workspace
colcon build --symlink-install

# After build, source the overlay
source install/local_setup.bash
```

3. Run nodes or launch files using ROS 2 CLI or ros2 launch as appropriate for the packages in `src/`.

## Recommended `.gitignore`

Add a `.gitignore` at the project root to avoid committing build artifacts:

```
# Colcon / ROS workspace
/build/
/install/
/log/

# Python
__pycache__/
*.pyc

# IDEs
.vscode/
.idea/

# OS files
.DS_Store
```

## GitHub: how to push this workspace

You can either push the whole workspace as one repository (simple) or create one repository per ROS package (recommended for reusability). Below are the common workflows.

Option A — push the entire workspace to one GitHub repo

1. Create a new repository on GitHub (for example: `kouroshkarimi/Roboso`).
2. On your local machine, from the repository root run:

```bash
git init
git add .
git commit -m "Initial commit: Roboso workspace"
git branch -M main
# Use HTTPS or SSH depending on your GitHub setup
git remote add origin git@github.com:kouroshkarimi/Roboso.git
git push -u origin main
```

If you prefer the `gh` CLI and want to create and push in one step:

```bash
gh repo create kouroshkarimi/Roboso --public --source=. --remote=origin --push
```

Option B — split packages into separate repositories (recommended for package reuse)

1. For each ROS package you want to publish separately, create a new GitHub repository (e.g. `roboso_controller`).
2. In each package directory (for example `src/roboso_controller`) run the git init / add / commit / remote / push sequence described above.

Notes when splitting:
- Keep the workspace `Roboso` as either a meta-repo (with submodule/subtree) or just as a local development workspace.
- Consider adding CI to each package repo that runs build and basic lint/tests.

## Commit & branch guidance

- Use terse, descriptive commit messages. Example: "Add base controller node for so101".
- Protect `main` on GitHub and use feature branches for work in progress.

## Suggested next steps

- Add a `.gitignore` (see above) and commit it before pushing large build directories.
- Clarify the exact meaning of "VLA" in the repo and add a design doc under `docs/` describing algorithms and interfaces for `so101`.
- Add small unit/integration tests and a CI workflow (GitHub Actions) that runs `colcon test` and basic linters.

## Contact / contributors

If you want, I can:

- Create the `.gitignore` file and commit it.
- Create a `docs/` subfolder with a VLA design.md draft.
- Prepare a GitHub Actions CI template for ROS (colcon) builds.

Tell me which of the above you'd like next and whether "VLA" should be expanded differently. I can update the README accordingly.

---

_Generated: initial README — please review and tell me any corrections or extra details to add (ROS distro, precise VLA meaning, intended license)._ 
