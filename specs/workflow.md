# Agent Workflow

## 1. Planning Phase
- **Understand**: Read `task.md` and user request.
- **Research**: Check `specs/context.md` and relevant code files.
- **Plan**: Create or update `implementation_plan.md` for complex tasks.
- **Confirm**: Get user buy-in for the plan.

## 2. Execution Phase
- **Task Tracking**: Update `task.md` to reflect current progress.
- **Implement**: Write code in small, verifiable steps.
- **Refine**: Ensure code aligns with `specs/constitution.md`.

## 3. Verification Phase
- **Verify**: Check that changes work as expected (e.g., build success, UI check).
- **Document**: Update `walkthrough.md` if significant UI/feature changes occurred.
- **Clean Up**: Remove temporary files or unused code.

## 4. Communication
- **Updates**: Use `task_boundary` to keep the user informed of state changes.
- **Notify**: Use `notify_user` only when blocking input is needed or the task is done.
