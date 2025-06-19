# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a modified version of the Qualisys Drone SDK (qfly), a Python library for tracking and controlling drones using Qualisys motion capture systems. The project enables real-time drone control with safety features like geofencing, speed limits, and smooth takeoff/landing.

## Core Architecture

### Main Components
- **qfly/**: Core SDK modules
  - `crazyflie.py`: QualisysCrazyflie class - main drone control wrapper
  - `qtm.py`: QtmWrapper class - asynchronous Qualisys motion capture connection  
  - `world.py`: World class - safety and airspace management
  - `pose.py`: Pose class - position/orientation representation
  - `deck.py`, `traqr.py`: Hardware-specific tracking components
  - `parallel_contexts.py`: Concurrent execution utilities
  - `utils.py`: Helper functions

- **api/**: REST API service for external control
  - `service.py`: API client functions using requests
  - `schema.py`: Pydantic models for request/response validation

- **examples/**: Demo scripts and test implementations
  - `original/`: Original example scripts from upstream
  - `flapper/`: Modified examples for specific drone configurations
  - `_future/`: Experimental features

### Key Design Patterns
- Threading-based architecture with `QtmWrapper` and `QualisysCrazyflie` as Thread subclasses
- Callback-driven pose tracking via `on_pose` functions
- Safety-first design with `World` object enforcing boundaries and speed limits
- Real-time keyboard interrupt handling for emergency landing (ESC key)

## Development Commands

### Environment Setup
```bash
# Install uv package manager first, then:
uv sync
```

### Code Formatting
```bash
./format.sh        # Runs isort and black on api/, examples/, qfly/
```

### Manual Formatting Commands
```bash
isort api/ examples/ qfly
black api/ examples/ qfly
```

### Documentation Generation
```bash
# Windows commands for rebuilding docs:
del docs\*.html
pdoc qfly --force --html --output-dir docs
move .\docs\qfly\* .\docs\
```

## Configuration

### Drone Configuration
- Crazyflie radio addresses typically: `radio://0/80/2M/E7E7E7E7XX`
- Active marker IDs: `[1, 2, 3, 4]` (front, right, back, left)
- QTM rigid body names must match between QTM software and code

### YAML Configurations
Example in `config/250516v1.yaml`:
- `radius`: Circular trajectory radius
- `omega`: Angular velocity (deg/s)  
- `trajectory_type`: Flight pattern ("XZ", "XY", etc.)
- `tracking_sec`: Duration of tracking phase

### Network Setup
- Default QTM IP: `127.0.0.1` (local) or specify remote IP
- API service runs on `localhost:8000`
- Port forwarding for remote access: `ssh -L 8000:localhost:8000 user@host`

## Development Notes

### Safety Requirements
- Always place Crazyflie flat on floor before takeoff, front pointing in +X direction
- ESC key triggers emergency landing in all example scripts
- Ctrl+C terminates programs immediately
- Speed limits and geofencing enforced by World object

### Hardware Dependencies
- Crazyradio PA dongle required for radio communication
- Active Marker Deck or Motion Capture Marker Deck for tracking
- Zadig drivers needed on Windows for Crazyflie/Crazyradio

### Coding Conventions
- Use threading for real-time operations
- Implement safety checks in all flight control logic
- Follow existing naming patterns for rigid bodies and radio addresses
- Configuration via YAML files preferred over hardcoded values

### API Integration
- External control via REST API at `/compute_target` endpoint
- Request/response validation using Pydantic schemas
- Position commands return Pose objects with status indicators

## Common Issues

- Radio address conflicts when flying multiple drones
- QTM rigid body names must exactly match code references
- Tracking loss tolerance managed by World.tracking_tolerance parameter
- Modified `set_speed_limit` function in crazyflie.py (outdated parameters fixed)

## Development Procedures

### Single Issue Development
1. update main branch
```
git checkout main
git pull
```
2. create branch on the issue (one-by-one relationship between branch and issue)
3. checkout the branch
```
git fetch origin
git checkout <branch-name>
```
4. develop!
5. create PR for the issue

### Multiple Issue Development (Sequential)
When working on multiple related issues:

1. **Sequential Branch Creation**: Create new branches from the previously edited branch (not main)
```bash
# After completing issue #62
git checkout feature/issue-62-task-dependencies-pipeline
git checkout -b feature/issue-61-status-tracking-pipeline

# After completing issue #61  
git checkout feature/issue-61-status-tracking-pipeline
git checkout -b feature/issue-60-cost-prediction-pipeline
```

2. **Frequent Commits**: Make small, focused commits during development
   - Commit after implementing each major component
   - Commit after fixing bugs or adding tests
   - Commit before attempting risky refactoring
   - **Guidelines**: Aim for commits every 30-60 minutes of active work

3. **Commit Message Format**:
```bash
# Good examples
git commit -m "feat: implement TaskNode data model with validation"
git commit -m "test: add unit tests for dependency analyzer"
git commit -m "fix: resolve circular dependency detection bug"
git commit -m "refactor: extract team analysis into separate method"
```

4. **Branch Dependencies**: Each branch builds on the previous one
   - This allows for incremental feature development
   - Reduces merge conflicts between related features
   - Enables testing of integrated functionality

5. **Final Integration**: After all issues are complete, create a final integration PR from the last branch to main

### Complex Issue Development with Sub-Issues
For large, complex features that benefit from being broken down into smaller, manageable parts:

1. **Create Parent Issue**: Main feature issue with comprehensive scope and requirements
   ```
   Title: feat: implement mindmap local persistence and fast loading system #111
   ```

2. **Create Sub-Issues**: Break down into logical components with clear dependencies
   ```bash
   # Example sub-issue structure
   feat: implement mindmap persistence foundation (database and configuration) #111-1
   feat: implement mindmap cache management system #111-2  
   feat: implement efficient mindmap serialization system #111-3
   feat: implement incremental loading and update system #111-4
   feat: implement lazy loading and performance optimization #111-5
   ```

3. **Sub-Issue Requirements**:
   - **Title Format**: `feat: [description] #[parent-issue]-[sub-number]`
   - **Parent Reference**: Include "**Closes part of #[parent-issue]**" in description
   - **GitHub Relationships**: Set parent/child relationships via GitHub Web UI:
     - Navigate to child issue → Development section → Add parent #[parent-issue]
     - Or use GitHub API: `gh api repos/:owner/:repo/issues/:issue_number/relationships`
   - **Clear Scope**: Define specific, focused scope for each sub-issue
   - **Dependencies**: Explicitly list dependencies on other sub-issues with issue numbers
   - **Acceptance Criteria**: Checkboxes for measurable completion criteria

4. **Sub-Issue Development Process**:
   
   **IMPORTANT: Always start from latest state to prevent conflicts**
   
   ```bash
   # Step 1: Always update to latest main branch
   git checkout main
   git pull origin main
   
   # Step 2a: For FIRST sub-issue - create from main
   git checkout -b feat/issue-111-1-persistence-foundation
   
   # Step 2b: For SUBSEQUENT sub-issues - create from previous sub-issue branch
   # This ensures all previous work is included and prevents conflicts
   git fetch origin
   git checkout feat/issue-111-1-persistence-foundation  # Previous sub-issue branch
   git pull origin feat/issue-111-1-persistence-foundation
   git checkout -b feat/issue-111-2-cache-management     # New sub-issue branch
   
   # Alternative: If previous sub-issue branch is not available, merge its content
   git checkout main
   git pull origin main
   git checkout -b feat/issue-111-2-cache-management
   git merge feat/issue-111-1-persistence-foundation     # Merge previous work
   
   # Step 3: Make fine-grained commits
   git commit -m "feat: add persistence module foundation"
   git commit -m "feat: implement persistence configuration system"
   git commit -m "feat: implement SQLite database layer"
   
   # Step 4: Create PR for sub-issue
   gh pr create --title "feat: implement mindmap persistence foundation #111-1"
   ```

5. **Sub-Issue PR Requirements**:
   - **Fine-grained commits**: Each commit should represent a single logical change
   - **Clear commit messages**: Following conventional commit format
   - **References**: Link to parent issue and related sub-issues
   - **Testing**: Include tests specific to the sub-issue scope
   - **Documentation**: Update relevant documentation for the component

6. **Integration Strategy**:
   - **Sequential Development**: Sub-issues build on each other to prevent conflicts
   - **Latest State Requirement**: Always start from the most recent sub-issue branch
   - **Conflict Prevention**: Merge or rebase from previous sub-issue branches before starting
   - **Dependency Management**: Wait for prerequisite PRs to be merged when possible
   - **Final Integration**: Parent issue is closed when all sub-issues are completed
   - **Testing**: Each sub-issue includes focused testing, final integration covers complete feature

7. **Conflict Prevention Best Practices**:
   
   **Before Starting Any New Sub-Issue:**
   ```bash
   # Check what branches exist
   git branch -a | grep issue-111
   
   # Always start from the latest related work
   git fetch origin
   git checkout [latest-sub-issue-branch]  # e.g., feat/issue-111-2-cache-management
   git pull origin [latest-sub-issue-branch]
   git checkout -b feat/issue-111-3-serialization-system
   
   # Verify you have all previous work
   ls -la [relevant-directories]  # Check that previous files are present
   ```
   
   **If Previous Branch Not Available:**
   ```bash
   # Start from main and merge needed branches
   git checkout main && git pull origin main
   git checkout -b feat/issue-111-3-serialization-system
   git merge feat/issue-111-1-persistence-foundation
   git merge feat/issue-111-2-cache-management
   ```
   
   **Before Each Commit:**
   ```bash
   # Ensure your branch is up to date
   git status  # Check for any unstaged changes
   git pull origin [current-branch] || echo "Remote branch doesn't exist yet - OK"
   ```

### Example Sub-Issue Template

```markdown
## Parent Issue
**Closes part of #111** - [Parent issue title]

## Overview
This is sub-issue [N] of Issue #111: [Parent issue description]

## Scope
[Specific scope for this sub-issue]

## Acceptance Criteria
- [ ] [Specific, measurable criteria]
- [ ] [Another criteria]

## Dependencies
- Depends on: #111-[X] ([Previous sub-issue])

## Related
- **Parent issue**: #111
- **Previous**: #111-[X] ([Previous sub-issue])
- **Next**: #111-[Y] ([Next sub-issue])
```

**Real Example from Issue #111:**
```bash
# Sub-issue #111-1 (Persistence Foundation)
git checkout main && git pull origin main
git checkout -b feat/issue-111-1-persistence-foundation
# ... implement and create PR #128

# Sub-issue #111-2 (Cache Management) 
git fetch origin
git checkout feat/issue-111-1-persistence-foundation
git pull origin feat/issue-111-1-persistence-foundation
git checkout -b feat/issue-111-2-cache-management
# ... implement and create PR #129

# Sub-issue #111-3 (Serialization) - builds on both previous
git fetch origin  
git checkout feat/issue-111-2-cache-management
git pull origin feat/issue-111-2-cache-management
git merge feat/issue-111-1-persistence-foundation  # Ensure all previous work
git checkout -b feat/issue-111-3-serialization-system
# ... implement and create PR
```

This approach ensures:
- **Conflict Prevention**: Each branch contains all previous work
- **Manageable complexity**: Large features broken into digestible pieces
- **Sequential Development**: Dependencies are naturally resolved
- **Clear tracking**: Easy to see progress on complex features  
- **Quality control**: Each component gets focused review and testing
- **Maintainability**: Easier to debug and maintain modular implementations
- **Consistency**: All sub-issues follow the same development pattern