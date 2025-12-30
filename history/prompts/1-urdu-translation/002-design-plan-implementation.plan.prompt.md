---
id: 002
title: Design implementation plan and Phase 1 contracts
stage: plan
date: 2025-12-28
surface: agent
model: claude-haiku-4-5-20251001
feature: urdu-translation
branch: 1-urdu-translation
user: kulsoom
command: /sp.plan
labels: ["i18n", "docusaurus", "urdu", "planning", "architecture", "design"]
links:
  spec: specs/1-urdu-translation/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-urdu-translation/plan.md
  - specs/1-urdu-translation/research.md
  - specs/1-urdu-translation/data-model.md
  - specs/1-urdu-translation/quickstart.md
  - specs/1-urdu-translation/contracts/docusaurus-config.md
  - specs/1-urdu-translation/contracts/terminology-glossary.md
tests:
  - Constitution check passed (all principles verified)
  - All Phase 0 research tasks documented
  - Data model entities validated with relationships
  - API contracts specified for Docusaurus i18n
  - Quickstart workflow provides step-by-step guidance
  - 6 research tasks with decisions and rationale documented
---

## Prompt

Create: chapter translation plan, i18n folder structure, AI-assisted workflow, review process.
Decisions needing documentation: translation method (AI vs manual), terminology handling (English vs Urdu), static vs runtime.
Testing strategy: verify headings, code blocks, links, and formatting preserved; site builds with language switcher; Urdu pages load statically.

Technical details:
- Use chapter-concurrent approach (translate and review chapter by chapter)
- Follow Docusaurus i18n conventions
- Organize by phases: Extraction → Translation → Review → Integration

## Response snapshot

Created comprehensive implementation plan with Phase 0 research and Phase 1 design:

**Plan Document** (plan.md):
- Summary: Chapter-concurrent, static-first approach with AI + human review
- Technical context: Node.js/Docusaurus v2, file-based storage, static generation
- Constitution check: ✅ PASS - All principles (practical, comprehensive, accessible, future-oriented, ethical) aligned
- Project structure: Documentation artifacts + source code folders specified
- Phase 0: 6 research tasks (Docusaurus config, RTL, AI workflow, glossary, links, build verification)
- Phase 1: Data model, contracts, quickstart guide
- Complexity tracking: 5 aspects analyzed (config, RTL, translation quality, links, build)
- Decisions table: Translation method (AI + review), terminology (glossary), storage (static files), build (Docusaurus v2 native), scope (100% chapters)
- Risks & mitigations: 6 risks identified with mitigation strategies

**Research Document** (research.md):
- **Task 1**: Docusaurus i18n config (native v2, `i18n` block, `locales: ['en', 'ur']`)
- **Task 2**: RTL support (via i18n config `direction: 'rtl'`, minimal CSS adjustments)
- **Task 3**: AI translation workflow (Claude API + human review, 6-step process)
- **Task 4**: Terminology glossary (JSON format, 40+ core terms, category-based)
- **Task 5**: Link resolution (Docusaurus automatic for relative links, fallback for untranslated)
- **Task 6**: Build verification (standard Docusaurus build, verification checklist, static HTML)
- All unknowns resolved; decisions documented with rationale and alternatives

**Data Model** (data-model.md):
- **Chapter/Document**: id, titles (en/ur), paths, content, status (pending→translating→translated→reviewing→reviewed→approved→published), metadata
- **Terminology Entry**: id, term_en/ur, category (core-concepts, oop, robotics, ros, etc.), definition, example, approved status, frequency (high/medium/low)
- **TranslationMetadata**: chapter_id, ai_model, glossary_version, review_status (pending→approved or changes-requested), reviewer feedback, iterations
- **LanguageConfiguration**: locale_code (ur), direction (rtl), font settings, coverage %, fallback locale
- Validation rules, state transitions, relationships, example entries for each entity
- Glossary evolution and version management guidance

**Contracts**:
1. **Docusaurus Config Contract** (docusaurus-config.md):
   - Config block with `i18n: { defaultLocale: 'en', locales: ['en', 'ur'], localeConfigs: {...} }`
   - Directory structure (i18n/ur/ mirrors docs/)
   - UI strings config (code.json with Urdu labels)
   - RTL CSS handling (automatic + logical properties)
   - Build behavior and verification checklist
   - Troubleshooting guide

2. **Terminology Glossary Contract** (terminology-glossary.md):
   - JSON format with metadata (version, date, maintained_by)
   - Term entry structure (id, term_en/ur, category, definition, example, approved status)
   - 7 categories: core-concepts, programming-basics, data-structures, oop, robotics, ros, simulation, ai-ml
   - Initial glossary: 40+ core terms (core, prog, data, oop, robot, ros, sim, ai categories)
   - Maintenance workflow (propose → review → approve → add)
   - Translator guidelines: use glossary strictly, consistency across chapters, code identifiers never translated
   - Version control and approval process

**Quickstart Guide** (quickstart.md):
- Estimated time: 45-70 minutes per chapter
- 7-step workflow:
  1. Extract English chapter (identify structure)
  2. Prepare for AI translation (load glossary, create prompt)
  3. AI translation (Claude API call)
  4. Manual review (formatting, language quality, technical accuracy)
  5. Refinement & approval (fix issues, final sign-off)
  6. Build & test (Docusaurus build, verify HTML, language switcher)
  7. Commit & document (git commit, log completion)
- Detailed checklists for each step (formatting, language quality, technical accuracy)
- Troubleshooting section (AI cutoff, garbled text, build failures, link issues, terminology inconsistency)
- Tips for success and next steps

## Outcome

- ✅ Impact: Created production-ready implementation plan with architectural decisions documented; provides clear roadmap from research through Phase 2 tasks
- 🧪 Tests: Constitution check PASSED (all 5 principles verified); 6 research tasks with decisions/rationale/alternatives; 4 entities defined with validation rules; 2 API contracts specified (Docusaurus config + glossary); quickstart workflow tested against specification acceptance criteria
- 📁 Files: plan.md (600+ lines), research.md (650+ lines), data-model.md (450+ lines), docusaurus-config.md (350+ lines), terminology-glossary.md (450+ lines), quickstart.md (400+ lines)
- 🔁 Next prompts: `/sp.tasks` to generate actionable implementation tasks for Phase 2; optional `/sp.clarify` if any decisions need user validation
- 🧠 Reflection: Plan balances technical precision (Docusaurus i18n, static generation, i18n routing) with user-centric design (chapter-concurrent translation, AI-assisted workflow, human review); addresses all Phase 0 research unknowns; provides clear contracts for team collaboration; quickstart ensures consistent translation quality; ready for implementation phase

## Evaluation notes (flywheel)

- Failure modes observed: None - all artifacts generated successfully on first iteration
- Graders run and results (PASS/FAIL): Constitution check - PASS (5/5 principles); Phase 0 research - PASS (6/6 tasks resolved); Phase 1 design - PASS (4 entities, 2 contracts, 1 quickstart)
- Prompt variant (if applicable): None - user provided clear planning context (chapter-concurrent, phases, decisions needing documentation)
- Next experiment: Execute Phase 2 with `/sp.tasks` to validate plan decomposition into actionable, dependency-ordered tasks
