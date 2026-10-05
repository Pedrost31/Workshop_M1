<!--
Please fill in this checklist before requesting a review.
-->

## Description

<!-- Which menu / test? What behavior is verified? -->

## Checklist

- [ ] Branch up to date (`git pull --rebase origin main`), no commit on `main`.
- [ ] **Pattern B** naming: `test_<methodName>_<state>_<expectedBehavior>`.
- [ ] **Bilingual docstring** EN mandatory `---` + FR optional.
- [ ] Test **green locally** (`python -m pytest <path> -v --headed`).
- [ ] **No secret** committed (`git status` does not show `.env`).

## CI

- [ ] The **Validate** check is green.
