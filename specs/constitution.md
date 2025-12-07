# Project Constitution

## 1. Core Principles
- **Quality Over Speed**: Prefer robust, maintainable solutions over quick hacks.
- **User Experience First**: All changes must preserve or enhance the existing UI/UX.
- **Accessibility**: Ensure all web interfaces are accessible and responsive.

## 2. Tech Stack Standards
- **Framework**: Docusaurus 3.x (React 19 based).
- **Language**: TypeScript for all logic; minimal JavaScript.
- **Styling**: Standard CSS Modules or global custom CSS (`custom.css`). Avoid inline styles where possible.
- **Documentation**: Markdown/MDX for all content pages.

## 3. Code Quality
- **Type Safety**: No `any` types unless absolutely necessary.
- **Clean Code**: Follow DRY (Don't Repeat Yourself) and KISS (Keep It Simple, Stupid) principles.
- **Formatting**: Respect existing Prettier/ESLint configurations.

## 4. AI Behavior
- **Context Awareness**: Always check `package.json` and `docusaurus.config.ts` before making architectural decisions.
- **Safety**: Do not delete user data or overwrite critical configuration without explicit confirmation.
- **Transparency**: Document all significant changes in `task.md` or `implementation_plan.md`.
