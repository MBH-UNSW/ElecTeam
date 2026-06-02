# Electrical Team Repository

This repository contains firmware, code, documentation, and resources for the Electrical Team.

---

# Development Workflow

To keep the codebase stable and avoid conflicts, **nobody should work directly on the `main` branch**.

All changes must follow the workflow below.

---

# Quick Reference

## Start New Work

```bash
git checkout main
git pull
git checkout -b feature/my-feature
```

## Save Changes

```bash
git add .
git commit -m "Describe changes"
```

## Upload Changes

```bash
git push origin feature/my-feature
```

## Create Pull Request

GitHub → Compare & Pull Request

Wait for review and approval before merging.

---

# Step 1: Update Your Local Repository

Before starting any work, make sure your local copy is up to date.

```bash
git checkout main
git pull
```

---

# Step 2: Create a Branch

Create a new branch for the feature, bug fix, or task you are working on.

Examples:

```bash
git checkout -b feature/can-driver
git checkout -b feature/temp-sensor
git checkout -b bugfix/pwm-output
```

### Branch Naming Convention

| Type | Example |
|--------|---------|
| New Feature | `feature/can-driver` |
| Bug Fix | `bugfix/pwm-output` |
| Documentation | `docs/readme-update` |
| Testing | `test/adc-validation` |

---

# Step 3: Make Your Changes

Write and test your code on your branch.

Do **not** commit broken or untested code.

For STM32 projects, make sure:
- The project builds successfully
- No compiler warnings are introduced
- Basic functionality has been tested

---

# Step 4: Commit Your Changes

Save your work with meaningful commit messages.

Example:

```bash
git add .
git commit -m "Add CAN communication driver"
```

Good commit messages:

```text
Add ADC temperature monitoring
Fix PWM duty cycle calculation
Implement UART debug output
Update pin assignments
```

Avoid:

```text
Update
Stuff
Fixed it
Changes
```

---

# Step 5: Push Your Branch

Push your branch to GitHub.

```bash
git push origin feature/can-driver
```

This uploads your work without affecting the `main` branch.

---

# Step 6: Create a Pull Request

Once your work is complete:

1. Open GitHub
2. Select your branch
3. Click **"Compare & Pull Request"**
4. Create a Pull Request (PR)

Include:

### Summary
What was changed?

### Testing
How was it tested?

### Notes
Anything reviewers should know.

Example:

```text
Summary:
Implemented CAN communication driver.

Testing:
Tested on NUCLEO-G431KB board.
Verified successful communication at 500 kbps.

Notes:
Still requires integration with motor controller.
```

---

# Step 7: Code Review

Before code is merged into `main`, another team member must review it.

Reviewers should check:

- Does the code work?
- Is it readable?
- Are comments adequate?
- Does it follow project standards?
- Are there any obvious bugs?

If changes are requested:

1. Make the requested changes
2. Commit them
3. Push to the same branch

The Pull Request will update automatically.

---

# Step 8: Merge Into Main

Once the Pull Request has been approved:

1. Merge the Pull Request
2. Use **Squash and Merge** if available
3. Delete the branch after merging

Only reviewed code should be merged into `main`.

---