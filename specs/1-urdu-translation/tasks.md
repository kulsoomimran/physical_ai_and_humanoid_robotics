# Tasks: Build Urdu i18n for Docusaurus Book

**Input**: Design documents from `/specs/1-urdu-translation/`
**Branch**: `1-urdu-translation` | **Status**: Ready for Phase 2 implementation

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: User story mapping (US1, US2, US3, US4, US5)
- Exact file paths included

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initialize Docusaurus i18n structure and baseline

- [x] T001 Verify Docusaurus version v2.x in `content/package.json` (verified: Docusaurus 3.9.2)
- [x] T002 [P] Create directory structure: `content/i18n/ur/docusaurus-docs/` (created)
- [x] T003 [P] Create directory structure: `content/i18n/ur/` (root) (created)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core i18n configuration that enables all user stories

**⚠️ CRITICAL**: No user story work begins until this phase completes

- [x] T004 Configure i18n in `content/docusaurus.config.ts`: added `i18n` block with locales `['en', 'ur']`, `defaultLocale: 'en'`, Urdu config with `direction: 'rtl'`, label `'اردو'`, `htmlLang: 'ur'` (✅ COMPLETED)
- [x] T005 Create `content/i18n/ur/code.json` with Urdu UI strings (navbar labels, sidebar labels, common UI text) (✅ COMPLETED - minimal empty object for now)
- [x] T006 Verify build with i18n config: run `npm run build` in `content/` and confirm `build/en/` and `build/ur/` directories exist (✅ SUCCESS: Both locales built, build output: build/en/ and build/ur/ generated)
- [ ] T007 Create initial terminology glossary at `specs/1-urdu-translation/contracts/terminology-glossary.json` with 40+ core terms (core-concepts, programming-basics, robotics, ros, ai-ml categories)

**Checkpoint**: Foundation ready - all user stories can now proceed in parallel

---

## Phase 3: User Story 1 - Discover and Access Urdu Content (Priority: P1) 🎯 MVP

**Goal**: Language switcher visible and functional; `/ur/` routes work; user can switch between English and Urdu

**Independent Test**: (1) Load site, (2) Find language switcher dropdown, (3) Click Urdu option, (4) Verify URL becomes `/ur/...` and content loads in Urdu, (5) Toggle back to English and verify `/docs/...` route works

- [ ] T008 [US1] Verify language switcher renders in navbar: inspect `build/en/index.html` and `build/ur/index.html` for language selection element
- [ ] T009 [US1] Test language switcher functionality: manually test both `/docs/` and `/ur/` routes in browser, verify switcher navigates between them
- [ ] T010 [US1] Test deep linking: verify `/ur/docs/intro/` and `/docs/intro/` both load correct versions
- [ ] T011 [US1] Verify breadcrumb navigation updates locale: check breadcrumbs display correctly on both English and Urdu pages

**Checkpoint**: User Story 1 is independently testable and deployable as MVP

---

## Phase 4: User Story 2 - Read Translated Content with Preserved Formatting (Priority: P1)

**Goal**: At least one full chapter translated; formatting (headings, code, tables) preserved; readable Urdu

**Independent Test**: (1) Translate 1 chapter (e.g., `intro`) to Urdu, (2) Verify headings H1/H2/H3 structure intact, (3) Verify code blocks readable with syntax highlighting, (4) Verify tables and lists preserved, (5) Build and load in browser, (6) Inspect Urdu page and confirm no broken formatting

- [ ] T012 [P] [US2] Extract English chapter to translate: identify all chapters in `content/docs/` and choose first one (e.g., `intro.md`)
- [ ] T013 [P] [US2] Create translation prompt with glossary: prepare Claude API prompt template that includes terminology glossary and Markdown preservation rules (see `quickstart.md`)
- [ ] T014 [US2] Translate sample chapter using Claude API: call Claude with `content/docs/intro.md` + glossary, save output to `content/i18n/ur/docusaurus-docs/intro.md`
- [ ] T015 [US2] Manual review of translation: verify formatting (headings, code blocks, tables), language quality (natural Urdu), terminology consistency (glossary terms used)
- [ ] T016 [US2] Refine translation: fix any issues found in review (formatting, grammar, terminology inconsistencies)
- [ ] T017 [US2] Build and verify: run `npm run build` in `content/`, verify `build/ur/docs/intro/index.html` contains Urdu text (no JS-required rendering)
- [ ] T018 [US2] Inspect generated HTML: view source of `build/ur/docs/intro/index.html`, confirm Urdu content is in HTML (not loaded by JavaScript), verify `dir="rtl"` and `lang="ur"` attributes

**Checkpoint**: User Story 2 confirmed - one chapter fully translated, formatted, and deployed

---

## Phase 5: User Story 3 - Navigate Book Structure in Urdu (Priority: P1)

**Goal**: All UI elements (sidebar, navigation) show Urdu labels; navigation between chapters works in Urdu

**Independent Test**: (1) Load `/ur/docs/intro/`, (2) Inspect sidebar menu - all labels in Urdu, (3) Click internal links - navigate to other Urdu chapters, (4) Verify TOC (table of contents) headings are in Urdu

- [ ] T019 [P] [US3] Translate sidebar labels in `content/i18n/ur/code.json`: ensure all navigation strings translated (e.g., "Documentation", "Getting Started" → Urdu equivalents)
- [ ] T020 [P] [US3] Verify sidebars.js supports i18n: check if `content/sidebars.js` uses i18n labels or hardcoded strings; if hardcoded, note for future improvement
- [ ] T021 [US3] Test sidebar navigation: load `/ur/docs/intro/`, verify sidebar renders with Urdu text, click sidebar links, confirm they navigate to other Urdu pages (or English fallback if untranslated)
- [ ] T022 [US3] Test internal links: verify links in `content/i18n/ur/docusaurus-docs/intro.md` point to correct Urdu pages (check relative link format `../chapter/` auto-localizes)
- [ ] T023 [US3] Test table of contents: open `/ur/docs/intro/`, verify TOC headings are rendered as Urdu text (from translated content)
- [ ] T024 [US3] Test breadcrumb navigation: verify breadcrumbs show Urdu text and navigate correctly between chapters

**Checkpoint**: User Story 3 confirmed - Urdu navigation fully functional

---

## Phase 6: User Story 4 - Preserve Technical Terminology (Priority: P2)

**Goal**: Technical terms consistently translated per glossary; code identifiers never translated

**Independent Test**: (1) Search translated chapter for repeated terms (e.g., "API", "variable"), (2) Verify all instances use glossary form (e.g., "اے پی آئی" every time), (3) Verify code identifiers remain English, (4) Spot-check 5 random terms against glossary JSON

- [ ] T025 [P] [US4] Finalize terminology glossary: expand from 40 to 60+ terms (add more programming, robotics, ROS terms based on chapters being translated)
- [ ] T026 [P] [US4] Create translator guidelines document: summarize glossary usage rules (use glossary terms consistently, code identifiers in English, no ad-hoc translations)
- [ ] T027 [US4] Validate terminology consistency in sample translation: search `content/i18n/ur/docusaurus-docs/intro.md` for all glossary terms, verify each used consistently throughout
- [ ] T028 [US4] Check code identifiers not translated: scan translated markdown for any translated code (e.g., function names, variable names), confirm only code remains in English
- [ ] T029 [US4] Review glossary version control: add version tag to `specs/1-urdu-translation/contracts/terminology-glossary.json` (e.g., "1.0", "1.1") to track which version was used per chapter

**Checkpoint**: User Story 4 confirmed - terminology standardized and validated

---

## Phase 7: User Story 5 - Build and Serve Static Urdu Pages (Priority: P2)

**Goal**: Full build succeeds; all Urdu pages are static HTML; site serves both locales; performance baseline established

**Independent Test**: (1) Run full build: `npm run build`, (2) Verify no errors, (3) Check `build/ur/` tree has all translated chapters, (4) Verify HTML files are static (no JS-required rendering), (5) Confirm build time < 5 minutes, (6) Test both `/docs/` and `/ur/docs/` routes serve correctly

- [ ] T030 [US5] Run full Docusaurus build: execute `npm run build` in `content/`, capture build output, verify success message
- [ ] T031 [US5] Verify build output structure: confirm `build/en/` and `build/ur/` both exist with complete directory trees
- [ ] T032 [US5] Inspect static HTML generation: view source of 3 random pages in `build/ur/docs/*/`, confirm Urdu text is in HTML (not JavaScript-rendered)
- [ ] T033 [US5] Verify RTL rendering: open `/ur/docs/intro/` in browser, confirm text flows right-to-left, no layout breakage, no horizontal scroll
- [ ] T034 [US5] Test static serving: confirm pages load from static HTML without JavaScript rendering delay (inspect Network tab in dev tools, confirm no large JS bundles for content)
- [ ] T035 [US5] Document build baseline: record build time, file count, output size for performance tracking
- [ ] T036 [US5] Verify SEO metadata: inspect `build/ur/docs/intro/index.html` for `<meta>` tags (charset, viewport, description), confirm Urdu locale metadata present

**Checkpoint**: User Story 5 confirmed - production build successful, static, performant

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Scale, QA, and finalize

- [ ] T037 [P] Translate remaining chapters: apply workflow (extract → translate → review → build) for all chapters in `content/docs/` not yet in `content/i18n/ur/docusaurus-docs/`
- [ ] T038 [P] Expand glossary: add new terms encountered during full translation (update `specs/1-urdu-translation/contracts/terminology-glossary.json`, increment version)
- [ ] T039 Comprehensive link validation: verify all internal links in Urdu chapters point to correct pages (or English fallback if untranslated), check external links unchanged
- [ ] T040 [P] Browser compatibility testing: test site on Chrome, Firefox, Safari, Edge; verify RTL rendering correct on all browsers
- [ ] T041 [P] Mobile testing: verify Urdu site responsive and readable on mobile devices (small screens)
- [ ] T042 Run full workflow validation: follow `specs/1-urdu-translation/quickstart.md` end-to-end for one complete chapter
- [ ] T043 Create translation SOP document: document final workflow (extraction, AI translation, review, refinement, build, commit) for team reference
- [ ] T044 Update project README: add Urdu i18n to main README with language switcher instructions and status
- [ ] T045 [P] Final QA: spot-check 10 random pages across Urdu site for formatting, readability, functionality
- [ ] T046 Documentation: update `content/docusaurus.config.js` comments to explain i18n config for future maintainers

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1 (Setup)
    ↓
Phase 2 (Foundational - BLOCKS all user stories)
    ↓
Phases 3-7 (User Stories) → CAN RUN IN PARALLEL
    ↓
Phase 8 (Polish)
```

### Within User Stories (Can Parallel If Staffed)

- **US1 (P1)**: Blocks nothing; independent discovery/access feature
- **US2 (P1)**: Blocks nothing; independent translation feature
- **US3 (P1)**: Depends on US2 translation existing; otherwise independent
- **US4 (P2)**: Depends on translated chapters; validates terminology
- **US5 (P2)**: Depends on all above; final build validation

**Suggested Sequential Approach** (one person):
1. Complete Phase 1-2 (setup)
2. Complete Phase 3 (US1 - MVP language switcher)
3. Complete Phase 4 (US2 - translate 1 chapter)
4. Complete Phase 5 (US3 - verify navigation)
5. Validate US1+US2+US3 together
6. Complete Phases 6-7 (US4, US5 - terminology + build)
7. Complete Phase 8 (polish + scale)

---

## MVP Scope (Phases 1-4)

**Minimum viable product** = Phases 1-4 completed:
- ✅ Language switcher works
- ✅ At least 1 chapter translated
- ✅ Navigation functional
- ✅ Site builds without errors

**Estimated effort**: 1-2 weeks (depending on chapter complexity and review cycles)

---

## Parallel Opportunities

### All Setup Tasks (Phase 1)
```
T001, T002, T003 → can all run together (different directories)
```

### Foundational Tasks (Phase 2)
```
T004 (config), T005 (UI strings), T006 (build), T007 (glossary)
→ T004 must complete before T006 (build depends on config)
→ T005 and T007 independent of T004
```

### All User Stories (Phases 3-7)
```
Once Phase 2 complete:
- US1 (T008-T011) independent
- US2 (T012-T018) independent
- US3 (T019-T024) can start with US2
- US4 (T025-T029) independent
- US5 (T030-T036) can start once other chapters translated
```

### Parallel Polish Tasks (Phase 8)
```
T037, T038, T040, T041, T045 can run together
T039, T042, T043, T044, T046 sequential or parallel
```

---

## Notes

- Each task has **exact file paths** for clarity
- **[P]** indicates parallelizable tasks (different files, no blocking dependencies)
- **[Story]** label (US1-US5) enables traceability and independent testing
- **Checkpoints** after each phase confirm independent functionality
- Use `quickstart.md` as operational guide for chapter translation workflow
- Commit after each task or logical group of tasks
- Refer to `research.md` and `plan.md` for technical decisions and rationale

