# Git Configuration for This Repository

## Setup (Run Once After Cloning)
```
git config pull.rebase true
git config branch.autoSetupRebase always
```

## Why?

These settings keep our commit history clean and linear by using rebase instead of merge. This makes it easier to track changes, debug problems, and review code.

## What Each Command Does

**`git config pull.rebase true`**
- Makes `git pull` use rebase instead of merge
- Avoids creating extra merge commits that clutter history

**`git config branch.autoSetupRebase always`**
- Automatically applies rebase to new branches
- You don't need to configure each branch manually

Questions? Ask a mentor or programming lead.
