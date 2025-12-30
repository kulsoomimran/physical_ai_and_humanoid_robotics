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
- [x] T007 Create initial terminology glossary at `specs/1-urdu-translation/contracts/terminology-glossary.json` with 40+ core terms (✅ COMPLETED: Created with 45 terms across all categories)

**Checkpoint**: Foundation ready - all user stories can now proceed in parallel

---

## Phase 3: User Story 1 - Discover and Access Urdu Content (Priority: P1) 🎯 MVP

**Goal**: Language switcher visible and functional; `/ur/` routes work; user can switch between English and Urdu

**Independent Test**: (1) Load site, (2) Find language switcher dropdown, (3) Click Urdu option, (4) Verify URL becomes `/ur/...` and content loads in Urdu, (5) Toggle back to English and verify `/docs/...` route works

- [x] T008 [US1] Verify language switcher renders in navbar: inspect `build/en/index.html` and `build/ur/index.html` for language selection element
- [x] T009 [US1] Test language switcher functionality: manually test both `/docs/` and `/ur/` routes in browser, verify switcher navigates between them
- [x] T010 [US1] Test deep linking: verify `/ur/docs/intro/` and `/docs/intro/` both load correct versions
- [x] T011 [US1] Verify breadcrumb navigation updates locale: check breadcrumbs display correctly on both English and Urdu pages

**Checkpoint**: User Story 1 is independently testable and deployable as MVP

---

## Phase 4: User Story 2 - Read Translated Content with Preserved Formatting (Priority: P1)

**Goal**: At least one full chapter translated; formatting (headings, code, tables) preserved; readable Urdu

**Independent Test**: (1) Translate 1 chapter (e.g., `intro`) to Urdu, (2) Verify headings H1/H2/H3 structure intact, (3) Verify code blocks readable with syntax highlighting, (4) Verify tables and lists preserved, (5) Build and load in browser, (6) Inspect Urdu page and confirm no broken formatting

- [x] T012 [P] [US2] Extract English chapter to translate: identify all chapters in `content/docs/` and choose first one (e.g., `intro.md`)
- [x] T013 [P] [US2] Create translation prompt with glossary: prepare Claude API prompt template that includes terminology glossary and Markdown preservation rules (see `quickstart.md`)
- [x] T014 [US2] Translate sample chapter using Claude API: call Claude with `content/docs/intro.md` + glossary, save output to `content/i18n/ur/docusaurus-docs/intro.md`
- [x] T015 [US2] Manual review of translation: verify formatting (headings, code blocks, tables), language quality (natural Urdu), terminology consistency (glossary terms used)
- [x] T016 [US2] Refine translation: fix any issues found in review (formatting, grammar, terminology inconsistencies)
- [x] T017 [US2] Build and verify: run `npm run build` in `content/`, verify `build/ur/docs/intro/index.html` contains Urdu text (no JS-required rendering)
- [x] T018 [US2] Inspect generated HTML: view source of `build/ur/docs/intro/index.html`, confirm Urdu content is in HTML (not loaded by JavaScript), verify `dir="rtl"` and `lang="ur"` attributes

**Checkpoint**: User Story 2 confirmed - one chapter fully translated, formatted, and deployed

---

## Phase 5: User Story 3 - Navigate Book Structure in Urdu (Priority: P1)

**Goal**: All UI elements (sidebar, navigation) show Urdu labels; navigation between chapters works in Urdu

**Independent Test**: (1) Load `/ur/docs/intro/`, (2) Inspect sidebar menu - all labels in Urdu, (3) Click internal links - navigate to other Urdu chapters, (4) Verify TOC (table of contents) headings are in Urdu

- [x] T019 [P] [US3] Translate sidebar labels in `content/i18n/ur/code.json`: ensure all navigation strings translated (e.g., "Documentation", "Getting Started" → Urdu equivalents)
- [x] T020 [P] [US3] Verify sidebars.js supports i18n: check if `content/sidebars.js` uses i18n labels or hardcoded strings; if hardcoded, note for future improvement
- [x] T021 [US3] Test sidebar navigation: load `/ur/docs/intro/`, verify sidebar renders with Urdu text, click sidebar links, confirm they navigate to other Urdu pages (or English fallback if untranslated)
- [x] T022 [US3] Test internal links: verify links in `content/i18n/ur/docusaurus-docs/intro.md` point to correct Urdu pages (check relative link format `../chapter/` auto-localizes)
- [x] T023 [US3] Test table of contents: open `/ur/docs/intro/`, verify TOC headings are rendered as Urdu text (from translated content)
- [x] T024 [US3] Test breadcrumb navigation: verify breadcrumbs show Urdu text and navigate correctly between chapters

**Checkpoint**: User Story 3 confirmed - Urdu navigation fully functional

---

## Phase 6: User Story 4 - Preserve Technical Terminology (Priority: P2)

**Goal**: Technical terms consistently translated per glossary; code identifiers never translated

**Independent Test**: (1) Search translated chapter for repeated terms (e.g., "API", "variable"), (2) Verify all instances use glossary form (e.g., "اے پی آئی" every time), (3) Verify code identifiers remain English, (4) Spot-check 5 random terms against glossary JSON

- [x] T025 [P] [US4] Finalize terminology glossary: expand from 40 to 60+ terms (add more programming, robotics, ROS terms based on chapters being translated)
- [x] T026 [P] [US4] Create translator guidelines document: summarize glossary usage rules (use glossary terms consistently, code identifiers in English, no ad-hoc translations)
- [x] T027 [US4] Validate terminology consistency in sample translation: search `content/i18n/ur/docusaurus-docs/intro.md` for all glossary terms, verify each used consistently throughout
- [x] T028 [US4] Check code identifiers not translated: scan translated markdown for any translated code (e.g., function names, variable names), confirm only code remains in English
- [x] T029 [US4] Review glossary version control: add version tag to `specs/1-urdu-translation/contracts/terminology-glossary.json` (e.g., "1.0", "1.1") to track which version was used per chapter

**Checkpoint**: User Story 4 confirmed - terminology standardized and validated

---

## Phase 7: User Story 5 - Build and Serve Static Urdu Pages (Priority: P2)

**Goal**: Full build succeeds; all Urdu pages are static HTML; site serves both locales; performance baseline established

**Independent Test**: (1) Run full build: `npm run build`, (2) Verify no errors, (3) Check `build/ur/` tree has all translated chapters, (4) Verify HTML files are static (no JS-required rendering), (5) Confirm build time < 5 minutes, (6) Test both `/docs/` and `/ur/docs/` routes serve correctly

- [x] T030 [US5] Run full Docusaurus build: execute `npm run build` in `content/`, capture build output, verify success message
- [x] T031 [US5] Verify build output structure: confirm `build/en/` and `build/ur/` both exist with complete directory trees
- [x] T032 [US5] Inspect static HTML generation: view source of 3 random pages in `build/ur/docs/*/`, confirm Urdu text is in HTML (not JavaScript-rendered)
- [x] T033 [US5] Verify RTL rendering: open `/ur/docs/intro/` in browser, confirm text flows right-to-left, no layout breakage, no horizontal scroll
- [x] T034 [US5] Test static serving: confirm pages load from static HTML without JavaScript rendering delay (inspect Network tab in dev tools, confirm no large JS bundles for content)
- [x] T035 [US5] Document build baseline: record build time, file count, output size for performance tracking
- [x] T036 [US5] Verify SEO metadata: inspect `build/ur/docs/intro/index.html` for `<meta>` tags (charset, viewport, description), confirm Urdu locale metadata present

**Checkpoint**: User Story 5 confirmed - production build successful, static, performant

---

## Phase 8: User Story 6 - Translate Module 1 Chapters (Priority: P1)

**Goal**: Translate all 6 chapters of Module 1 (The Robotic Nervous System - ROS 2) from English to Urdu with proper formatting and terminology

**Independent Test**: (1) Translate each chapter in sequence, (2) Verify headings, code blocks, and lists preserved, (3) Confirm terminology consistency with glossary, (4) Build and verify each chapter renders correctly in Urdu

- [x] T037 [P] [US6] Extract English chapter: identify and prepare `content/docs/modules/module1/chapter1.md` for translation
- [x] T038 [P] [US6] Translate chapter1 using Claude API: call Claude with `content/docs/modules/module1/chapter1.md` + glossary, save output to `content/i18n/ur/docusaurus-docs/modules/module1/chapter1.md`
- [x] T039 [US6] Manual review of chapter1 translation: verify formatting, language quality, and terminology consistency
- [x] T040 [US6] Refine chapter1 translation: fix any issues found in review
- [x] T041 [P] [US6] Extract English chapter: identify and prepare `content/docs/modules/module1/chapter2.md` for translation
- [x] T042 [P] [US6] Translate chapter2 using Claude API: call Claude with `content/docs/modules/module1/chapter2.md` + glossary, save output to `content/i18n/ur/docusaurus-docs/modules/module1/chapter2.md`
- [x] T043 [US6] Manual review of chapter2 translation: verify formatting, language quality, and terminology consistency
- [x] T044 [US6] Refine chapter2 translation: fix any issues found in review
- [x] T045 [P] [US6] Extract English chapter: identify and prepare `content/docs/modules/module1/chapter3.md` for translation
- [x] T046 [P] [US6] Translate chapter3 using Claude API: call Claude with `content/docs/modules/module1/chapter3.md` + glossary, save output to `content/i18n/ur/docusaurus-docs/modules/module1/chapter3.md`
- [x] T047 [US6] Manual review of chapter3 translation: verify formatting, language quality, and terminology consistency
- [x] T048 [US6] Refine chapter3 translation: fix any issues found in review
- [x] T049 [P] [US6] Extract English chapter: identify and prepare `content/docs/modules/module1/chapter4.md` for translation
- [x] T050 [P] [US6] Translate chapter4 using Claude API: call Claude with `content/docs/modules/module1/chapter4.md` + glossary, save output to `content/i18n/ur/docusaurus-docs/modules/module1/chapter4.md`
- [x] T051 [US6] Manual review of chapter4 translation: verify formatting, language quality, and terminology consistency
- [x] T052 [US6] Refine chapter4 translation: fix any issues found in review
- [x] T053 [P] [US6] Extract English chapter: identify and prepare `content/docs/modules/module1/chapter5.md` for translation
- [x] T054 [P] [US6] Translate chapter5 using Claude API: call Claude with `content/docs/modules/module1/chapter5.md` + glossary, save output to `content/i18n/ur/docusaurus-docs/modules/module1/chapter5.md`
- [x] T055 [US6] Manual review of chapter5 translation: verify formatting, language quality, and terminology consistency
- [x] T056 [US6] Refine chapter5 translation: fix any issues found in review
- [x] T057 [P] [US6] Extract English chapter: identify and prepare `content/docs/modules/module1/chapter6.md` for translation
- [x] T058 [P] [US6] Translate chapter6 using Claude API: call Claude with `content/docs/modules/module1/chapter6.md` + glossary, save output to `content/i18n/ur/docusaurus-docs/modules/module1/chapter6.md`
- [x] T059 [US6] Manual review of chapter6 translation: verify formatting, language quality, and terminology consistency
- [x] T060 [US6] Refine chapter6 translation: fix any issues found in review
- [x] T061 [US6] Build and verify Module 1: run `npm run build` in `content/`, verify all Module 1 Urdu chapters exist in `build/ur/docs/modules/module1/`
- [x] T062 [US6] Inspect Module 1 HTML: verify all 6 Module 1 Urdu chapters render correctly with proper RTL support

**Checkpoint**: User Story 6 confirmed - Module 1 fully translated and deployed

---

## Phase 9: User Story 7 - Translate Module 2 Chapters (Priority: P1)

**Goal**: Translate all 6 chapters of Module 2 (The Digital Twin - Gazebo & Unity) from English to Urdu with proper formatting and terminology

**Independent Test**: (1) Translate each chapter in sequence, (2) Verify headings, code blocks, and lists preserved, (3) Confirm terminology consistency with glossary, (4) Build and verify each chapter renders correctly in Urdu

- [x] T063 [P] [US7] Extract English chapter: identify and prepare `content/docs/modules/module2/chapter1.md` for translation
- [x] T064 [P] [US7] Translate chapter1 using Claude API: call Claude with `content/docs/modules/module2/chapter1.md` + glossary, save output to `content/i18n/ur/docusaurus-docs/modules/module2/chapter1.md`
- [x] T065 [US7] Manual review of chapter1 translation: verify formatting, language quality, and terminology consistency
- [x] T066 [US7] Refine chapter1 translation: fix any issues found in review
- [x] T067 [P] [US7] Extract English chapter: identify and prepare `content/docs/modules/module2/chapter2.md` for translation
- [x] T068 [P] [US7] Translate chapter2 using Claude API: call Claude with `content/docs/modules/module2/chapter2.md` + glossary, save output to `content/i18n/ur/docusaurus-docs/modules/module2/chapter2.md`
- [x] T069 [US7] Manual review of chapter2 translation: verify formatting, language quality, and terminology consistency
- [x] T070 [US7] Refine chapter2 translation: fix any issues found in review
- [x] T071 [P] [US7] Extract English chapter: identify and prepare `content/docs/modules/module2/chapter3.md` for translation
- [x] T072 [P] [US7] Translate chapter3 using Claude API: call Claude with `content/docs/modules/module2/chapter3.md` + glossary, save output to `content/i18n/ur/docusaurus-docs/modules/module2/chapter3.md`
- [x] T073 [US7] Manual review of chapter3 translation: verify formatting, language quality, and terminology consistency
- [x] T074 [US7] Refine chapter3 translation: fix any issues found in review
- [x] T075 [P] [US7] Extract English chapter: identify and prepare `content/docs/modules/module2/chapter4.md` for translation
- [x] T076 [P] [US7] Translate chapter4 using Claude API: call Claude with `content/docs/modules/module2/chapter4.md` + glossary, save output to `content/i18n/ur/docusaurus-docs/modules/module2/chapter4.md`
- [x] T077 [US7] Manual review of chapter4 translation: verify formatting, language quality, and terminology consistency
- [x] T078 [US7] Refine chapter4 translation: fix any issues found in review
- [x] T079 [P] [US7] Extract English chapter: identify and prepare `content/docs/modules/module2/chapter5.md` for translation
- [x] T080 [P] [US7] Translate chapter5 using Claude API: call Claude with `content/docs/modules/module2/chapter5.md` + glossary, save output to `content/i18n/ur/docusaurus-docs/modules/module2/chapter5.md`
- [x] T081 [US7] Manual review of chapter5 translation: verify formatting, language quality, and terminology consistency
- [x] T082 [US7] Refine chapter5 translation: fix any issues found in review
- [x] T083 [P] [US7] Extract English chapter: identify and prepare `content/docs/modules/module2/chapter6.md` for translation
- [x] T084 [P] [US7] Translate chapter6 using Claude API: call Claude with `content/docs/modules/module2/chapter6.md` + glossary, save output to `content/i18n/ur/docusaurus-docs/modules/module2/chapter6.md`
- [x] T085 [US7] Manual review of chapter6 translation: verify formatting, language quality, and terminology consistency
- [x] T086 [US7] Refine chapter6 translation: fix any issues found in review
- [x] T087 [US7] Build and verify Module 2: run `npm run build` in `content/`, verify all Module 2 Urdu chapters exist in `build/ur/docs/modules/module2/`
- [x] T088 [US7] Inspect Module 2 HTML: verify all 6 Module 2 Urdu chapters render correctly with proper RTL support

**Checkpoint**: User Story 7 confirmed - Module 2 fully translated and deployed

---

## Phase 10: User Story 8 - Translate Module 3 Chapters (Priority: P1)

**Goal**: Translate all 6 chapters of Module 3 (The AI-Robot Brain - NVIDIA Isaac™) from English to Urdu with proper formatting and terminology

**Independent Test**: (1) Translate each chapter in sequence, (2) Verify headings, code blocks, and lists preserved, (3) Confirm terminology consistency with glossary, (4) Build and verify each chapter renders correctly in Urdu

- [x] T089 [P] [US8] Extract English chapter: identify and prepare `content/docs/modules/module3/chapter1.md` for translation
- [x] T090 [P] [US8] Translate chapter1 using Claude API: call Claude with `content/docs/modules/module3/chapter1.md` + glossary, save output to `content/i18n/ur/docusaurus-docs/modules/module3/chapter1.md`
- [x] T091 [US8] Manual review of chapter1 translation: verify formatting, language quality, and terminology consistency
- [x] T092 [US8] Refine chapter1 translation: fix any issues found in review
- [x] T093 [P] [US8] Extract English chapter: identify and prepare `content/docs/modules/module3/chapter2.md` for translation
- [x] T094 [P] [US8] Translate chapter2 using Claude API: call Claude with `content/docs/modules/module3/chapter2.md` + glossary, save output to `content/i18n/ur/docusaurus-docs/modules/module3/chapter2.md`
- [x] T095 [US8] Manual review of chapter2 translation: verify formatting, language quality, and terminology consistency
- [x] T096 [US8] Refine chapter2 translation: fix any issues found in review
- [x] T097 [P] [US8] Extract English chapter: identify and prepare `content/docs/modules/module3/chapter3.md` for translation
- [x] T098 [P] [US8] Translate chapter3 using Claude API: call Claude with `content/docs/modules/module3/chapter3.md` + glossary, save output to `content/i18n/ur/docusaurus-docs/modules/module3/chapter3.md`
- [x] T099 [US8] Manual review of chapter3 translation: verify formatting, language quality, and terminology consistency
- [x] T100 [US8] Refine chapter3 translation: fix any issues found in review
- [x] T101 [P] [US8] Extract English chapter: identify and prepare `content/docs/modules/module3/chapter4.md` for translation
- [x] T102 [P] [US8] Translate chapter4 using Claude API: call Claude with `content/docs/modules/module3/chapter4.md` + glossary, save output to `content/i18n/ur/docusaurus-docs/modules/module3/chapter4.md`
- [x] T103 [US8] Manual review of chapter4 translation: verify formatting, language quality, and terminology consistency
- [x] T104 [US8] Refine chapter4 translation: fix any issues found in review
- [x] T105 [P] [US8] Extract English chapter: identify and prepare `content/docs/modules/module3/chapter5.md` for translation
- [x] T106 [P] [US8] Translate chapter5 using Claude API: call Claude with `content/docs/modules/module3/chapter5.md` + glossary, save output to `content/i18n/ur/docusaurus-docs/modules/module3/chapter5.md`
- [x] T107 [US8] Manual review of chapter5 translation: verify formatting, language quality, and terminology consistency
- [x] T108 [US8] Refine chapter5 translation: fix any issues found in review
- [x] T109 [P] [US8] Extract English chapter: identify and prepare `content/docs/modules/module3/chapter6.md` for translation
- [x] T110 [P] [US8] Translate chapter6 using Claude API: call Claude with `content/docs/modules/module3/chapter6.md` + glossary, save output to `content/i18n/ur/docusaurus-docs/modules/module3/chapter6.md`
- [x] T111 [US8] Manual review of chapter6 translation: verify formatting, language quality, and terminology consistency
- [x] T112 [US8] Refine chapter6 translation: fix any issues found in review
- [x] T113 [US8] Build and verify Module 3: run `npm run build` in `content/`, verify all Module 3 Urdu chapters exist in `build/ur/docs/modules/module3/`
- [x] T114 [US8] Inspect Module 3 HTML: verify all 6 Module 3 Urdu chapters render correctly with proper RTL support

**Checkpoint**: User Story 8 confirmed - Module 3 fully translated and deployed

---

## Phase 11: User Story 9 - Translate Module 4 Chapters (Priority: P1)

**Goal**: Translate all 6 chapters of Module 4 (Vision-Language-Action - VLA) from English to Urdu with proper formatting and terminology

**Independent Test**: (1) Translate each chapter in sequence, (2) Verify headings, code blocks, and lists preserved, (3) Confirm terminology consistency with glossary, (4) Build and verify each chapter renders correctly in Urdu

- [x] T115 [P] [US9] Extract English chapter: identify and prepare `content/docs/modules/module4/chapter1.md` for translation
- [x] T116 [P] [US9] Translate chapter1 using Claude API: call Claude with `content/docs/modules/module4/chapter1.md` + glossary, save output to `content/i18n/ur/docusaurus-docs/modules/module4/chapter1.md`
- [x] T117 [US9] Manual review of chapter1 translation: verify formatting, language quality, and terminology consistency
- [x] T118 [US9] Refine chapter1 translation: fix any issues found in review
- [x] T119 [P] [US9] Extract English chapter: identify and prepare `content/docs/modules/module4/chapter2.md` for translation
- [x] T120 [P] [US9] Translate chapter2 using Claude API: call Claude with `content/docs/modules/module4/chapter2.md` + glossary, save output to `content/i18n/ur/docusaurus-docs/modules/module4/chapter2.md`
- [x] T121 [US9] Manual review of chapter2 translation: verify formatting, language quality, and terminology consistency
- [x] T122 [US9] Refine chapter2 translation: fix any issues found in review
- [x] T123 [P] [US9] Extract English chapter: identify and prepare `content/docs/modules/module4/chapter3.md` for translation
- [x] T124 [P] [US9] Translate chapter3 using Claude API: call Claude with `content/docs/modules/module4/chapter3.md` + glossary, save output to `content/i18n/ur/docusaurus-docs/modules/module4/chapter3.md`
- [x] T125 [US9] Manual review of chapter3 translation: verify formatting, language quality, and terminology consistency
- [x] T126 [US9] Refine chapter3 translation: fix any issues found in review
- [x] T127 [P] [US9] Extract English chapter: identify and prepare `content/docs/modules/module4/chapter4.md` for translation
- [x] T128 [P] [US9] Translate chapter4 using Claude API: call Claude with `content/docs/modules/module4/chapter4.md` + glossary, save output to `content/i18n/ur/docusaurus-docs/modules/module4/chapter4.md`
- [x] T129 [US9] Manual review of chapter4 translation: verify formatting, language quality, and terminology consistency
- [x] T130 [US9] Refine chapter4 translation: fix any issues found in review
- [x] T131 [P] [US9] Extract English chapter: identify and prepare `content/docs/modules/module4/chapter5.md` for translation
- [x] T132 [P] [US9] Translate chapter5 using Claude API: call Claude with `content/docs/modules/module4/chapter5.md` + glossary, save output to `content/i18n/ur/docusaurus-docs/modules/module4/chapter5.md`
- [x] T133 [US9] Manual review of chapter5 translation: verify formatting, language quality, and terminology consistency
- [x] T134 [US9] Refine chapter5 translation: fix any issues found in review
- [x] T135 [P] [US9] Extract English chapter: identify and prepare `content/docs/modules/module4/chapter6.md` for translation
- [x] T136 [P] [US9] Translate chapter6 using Claude API: call Claude with `content/docs/modules/module4/chapter6.md` + glossary, save output to `content/i18n/ur/docusaurus-docs/modules/module4/chapter6.md`
- [x] T137 [US9] Manual review of chapter6 translation: verify formatting, language quality, and terminology consistency
- [x] T138 [US9] Refine chapter6 translation: fix any issues found in review
- [x] T139 [US9] Build and verify Module 4: run `npm run build` in `content/`, verify all Module 4 Urdu chapters exist in `build/ur/docs/modules/module4/`
- [x] T140 [US9] Inspect Module 4 HTML: verify all 6 Module 4 Urdu chapters render correctly with proper RTL support

**Checkpoint**: User Story 9 confirmed - Module 4 fully translated and deployed

---

## Phase 12: Polish & Cross-Cutting Concerns

**Purpose**: Scale, QA, and finalize

- [x] T141 [P] Expand glossary: add new terms encountered during full translation (update `specs/1-urdu-translation/contracts/terminology-glossary.json`, increment version)
- [x] T142 Comprehensive link validation: verify all internal links in Urdu chapters point to correct pages (or English fallback if untranslated), check external links unchanged
- [x] T143 [P] Browser compatibility testing: test site on Chrome, Firefox, Safari, Edge; verify RTL rendering correct on all browsers
- [x] T144 [P] Mobile testing: verify Urdu site responsive and readable on mobile devices (small screens)
- [x] T145 Run full workflow validation: follow `specs/1-urdu-translation/quickstart.md` end-to-end for one complete chapter
- [x] T146 Create translation SOP document: document final workflow (extraction, AI translation, review, refinement, build, commit) for team reference
- [x] T147 Update project README: add Urdu i18n to main README with language switcher instructions and status
- [x] T148 [P] Final QA: spot-check 10 random pages across Urdu site for formatting, readability, functionality
- [x] T149 Documentation: update `content/docusaurus.config.js` comments to explain i18n config for future maintainers

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1 (Setup)
    ↓
Phase 2 (Foundational - BLOCKS all user stories)
    ↓
Phases 3-7 (User Stories 1-5) → CAN RUN IN PARALLEL
    ↓
Phases 8-11 (User Stories 6-9 - Translation Phases) → CAN RUN IN PARALLEL
    ↓
Phase 12 (Polish)
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
7. Complete Phases 8-11 (US6-US9 - translate all modules)
8. Complete Phase 12 (polish + scale)

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

### Translation Phases (Phases 8-11)
```
Once foundational work complete (Phases 1-7):
- US6 (T037-T062) - Module 1 translation tasks (chapters 1-6)
- US7 (T063-T088) - Module 2 translation tasks (chapters 1-6)
- US8 (T089-T114) - Module 3 translation tasks (chapters 1-6)
- US9 (T115-T140) - Module 4 translation tasks (chapters 1-6)
- All translation phases (US6-US9) can run in parallel
- Each module's chapters can run in parallel [P] within their phase
```

### Parallel Polish Tasks (Phase 12)
```
T141, T143, T144, T148 can run together
T142, T145, T146, T147, T149 sequential or parallel
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

