# Implementation Plan: Build Urdu i18n for Docusaurus Book

**Branch**: `1-urdu-translation` | **Date**: 2025-12-28 | **Spec**: [specs/1-urdu-translation/spec.md](./spec.md)

**Input**: Feature specification from `/specs/1-urdu-translation/spec.md`

## Summary

Build a Docusaurus v2 i18n-based Urdu language variant of the Physical AI & Humanoid Robotics book. The approach is **chapter-concurrent, static-first**: extract English chapters to `/i18n/ur/` folder structure, translate each chapter using AI assistance with manual review, preserve formatting (headings, code, tables, links), handle technical terminology through consistent transliteration or English-with-context, build static HTML pages with language switcher, and ensure SEO-friendly delivery. No runtime translation—all content is pre-translated and committed as Markdown files.

## Technical Context

**Language/Version**: Node.js + Docusaurus v2.x (configuration-driven), Markdown/MDX for content
**Primary Dependencies**:
- Docusaurus v2 (framework, i18n support)
- Node.js package manager (npm)
- docusaurus.config.js (i18n locale configuration)
- markdown/mdx processing (built into Docusaurus)
- AI service for translation assistance (Claude API or similar)

**Storage**: File-based; static Markdown files in `/i18n/ur/docusaurus-docs/` and locale strings in `/i18n/ur/code.json`
**Testing**:
- Manual verification of formatting (headings, code blocks, tables)
- Link validation (internal + external)
- Language switcher functional testing
- Build verification (`npm run build` output check)
- Static HTML inspection (no JavaScript-required rendering)

**Target Platform**: Web (Docusaurus hosting; static HTML served)
**Project Type**: Documentation website (Docusaurus)
**Performance Goals**:
- Build time: <5 minutes (including all locales)
- Page load: <2s (static HTML, no client-side translation delay)
- SEO: Full static indexing capability

**Constraints**:
- Docusaurus v2 i18n conventions only
- Static generation (no runtime APIs)
- RTL text support (Urdu script)
- Markdown formatting preservation
- No framework redesign; CSS adjustments minimal

**Scale/Scope**:
- Estimated 15-25 English chapters (to verify)
- Single feature/feature suite (not full platform)
- Terminology glossary (10-50 terms)
- One supporting locale (Urdu)

## Constitution Check

**GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.**

### Principles Verification

| Principle | Status | Rationale |
|-----------|--------|-----------|
| **I. Practical & Hands-on** | ✅ PASS | Translated content preserves code examples and practical exercises; Urdu learners get same hands-on experience as English readers |
| **II. Comprehensive Modules** | ✅ PASS | i18n translation covers all modules uniformly; no module excluded from Urdu version |
| **III. Clear & Accessible** | ✅ PASS | Urdu translation must be idiomatic and accessible to Urdu-speaking students; aligns with accessibility goal |
| **IV. Future-Oriented** | ✅ PASS | Static i18n framework is extensible to future languages (Spanish, Chinese, etc.); no technical debt introduced |
| **V. Ethical Robotics** | ✅ PASS | Ethical sections translated; no cultural or ethical content modified |

### Docusaurus Compliance

- ✅ Docusaurus v2 i18n framework required (per spec constraint)
- ✅ Static generation (per constraint; aligns with Docusaurus build model)
- ✅ Markdown/MDX output (native Docusaurus format)
- ✅ No proprietary dependencies (all open-source or Anthropic services)

**Gate Result**: ✅ **PASS** — All principles and constraints aligned. No violations detected.

---

## Project Structure

### Documentation (this feature)

```text
specs/1-urdu-translation/
├── spec.md                      # Feature specification (✅ complete)
├── plan.md                      # This file (Phase 1 output)
├── research.md                  # Phase 0 output (TBD - research tasks)
├── data-model.md                # Phase 1 output (TBD - content entities)
├── quickstart.md                # Phase 1 output (TBD - workflow guide)
├── contracts/                   # Phase 1 output (TBD - API/config contracts)
│   ├── docusaurus-config.md     # Docusaurus i18n config contract
│   ├── translation-metadata.md  # Translation workflow contract
│   └── terminology-glossary.md  # Technical term translations
├── checklists/
│   └── requirements.md          # Specification quality checklist (✅ complete)
└── tasks.md                     # Phase 2 output (/sp.tasks command - NOT YET)
```

### Source Code (repository root - Docusaurus structure)

```text
content/                                    # Docusaurus root (existing)
├── docusaurus.config.js                   # Main config (MODIFY: add Urdu i18n)
├── docs/                                   # English chapters (existing)
│   ├── intro.md
│   ├── chapter-1.md
│   ├── chapter-2.md
│   └── ... (all English chapters)
│
├── i18n/                                   # NEW: i18n root (created by /sp.plan)
│   └── ur/                                 # Urdu locale
│       ├── code.json                       # UI strings in Urdu (nav, labels, etc.)
│       └── docusaurus-docs/                # Translated chapter structure
│           ├── intro.md                    # Urdu translation of intro.md
│           ├── chapter-1.md
│           ├── chapter-2.md
│           └── ... (parallel to docs/)
│
├── sidebars.js                            # Navigation config (MODIFY: Urdu labels)
└── src/
    └── pages/                             # Optional: custom pages if any

build/                                      # Build output (generated)
├── en/                                     # English static output
├── ur/                                     # Urdu static output (NEW)
└── ...
```

**Structure Decision**: **Docusaurus standard i18n layout**. The `i18n/ur/` directory mirrors the `docs/` structure, following Docusaurus v2 conventions. This is the only supported approach for Docusaurus v2 i18n and ensures build-time generation of both English and Urdu routes (`/docs/...` and `/ur/docs/...`).

---

## Phase 0: Research & Unknowns

### Research Tasks (Dispatched)

The following tasks resolve unknowns and establish best practices:

#### Task 1: Docusaurus v2 i18n Configuration Deep Dive
- **Unknown**: Exact Docusaurus version and current i18n configuration
- **Research**: Verify Docusaurus v2.x version in `package.json`; inspect `docusaurus.config.js` for existing i18n setup; identify required i18n config keys (locales, defaultLocale, path, label, htmlLang)
- **Deliverable**: Configuration contract and migration steps

#### Task 2: RTL (Right-to-Left) Text Support for Urdu
- **Unknown**: CSS/styling requirements for RTL in Docusaurus
- **Research**: Identify CSS classes/configuration for RTL; check Docusaurus theme (docusaurus-theme-classic or custom) for existing RTL support; determine minimal CSS changes needed (dir="rtl", text alignment, layout adjustments)
- **Deliverable**: RTL setup guide and CSS modifications checklist

#### Task 3: AI Translation Workflow & Quality Assurance
- **Unknown**: Translation method (Claude API, batch processing, manual steps)
- **Research**: Define AI prompt for chapter translation (preserve Markdown, technical terms, code); establish review checklist for human validation; create glossary template for term consistency
- **Deliverable**: Translation workflow SOP and review template

#### Task 4: Terminology Transliteration Standards
- **Unknown**: Standardized Urdu transliterations for programming terms
- **Research**: Identify common technical terms (API, database, algorithm, variable, function, etc.) and their standard Urdu equivalents or transliterations; establish glossary entries
- **Deliverable**: Terminology glossary (10-50 terms) with Urdu equivalents and usage examples

#### Task 5: Link Resolution in i18n Context
- **Unknown**: How Docusaurus handles internal links across locales
- **Research**: Test link behavior (e.g., does `/docs/chapter-1/` in English automatically become `/ur/docs/chapter-1/` in Urdu?); identify fallback behavior for untranslated chapters
- **Deliverable**: Link resolution guide and fallback strategy

#### Task 6: Build Process & Static Output Verification
- **Unknown**: Build output structure and verification method
- **Research**: Run `npm run build` with i18n config; inspect output directory structure; verify static HTML completeness (no JavaScript-required rendering)
- **Deliverable**: Build verification checklist and test procedure

### Research Output: `research.md`

All research tasks will be consolidated into a single `research.md` file with sections:

```markdown
# Research Summary: Urdu i18n for Docusaurus

## Task 1: Docusaurus v2 i18n Configuration
- **Decision**: [Selected config approach]
- **Rationale**: [Why chosen]
- **Alternatives considered**: [Other options and why rejected]
- **Implementation details**: [Specific config keys and values]

## Task 2: RTL Support for Urdu
- [Similar structure...]

[... all tasks ...]
```

---

## Phase 1: Design & Contracts

### 1.1 Data Model & Entities

Extract and define key entities for translation workflow:

#### Entity: Chapter/Document
- **Fields**: id, title_en, title_ur, slug, path (docs/ or i18n/ur/), content_md, status (pending, translated, reviewed, published)
- **Relationships**: Contains headings, code blocks, links; references terminology glossary
- **Validation**: Markdown format valid, required sections present (intro, content, conclusion if applicable)
- **State transitions**: pending → translated → reviewed → published

#### Entity: Terminology Entry
- **Fields**: id, term_en, term_ur (transliteration or Urdu), category (API, data structures, algorithms, platforms), usage_context, example_sentence
- **Relationships**: Referenced by multiple chapters
- **Validation**: Term must be consistent across all chapters; Urdu text must use Urdu script
- **Lifecycle**: Proposed → approved → used in translations

#### Entity: Translation Metadata
- **Fields**: chapter_id, translator_ai_model, review_status (approved, pending_review, rejected), reviewer_name, review_date, notes
- **Relationships**: Links to chapter
- **Validation**: Review required before publication

#### Entity: Language Configuration
- **Fields**: locale_code (ur), language_name (Urdu), supported_from_date, rtl_enabled, default_fallback_locale (en)
- **Relationships**: Docusaurus config entry
- **Validation**: Must match Docusaurus i18n config

**Output**: `data-model.md` (detailed entity definitions with field specs and relationships)

### 1.2 API & Configuration Contracts

#### Contract 1: Docusaurus i18n Configuration (`docusaurus.config.js`)

```javascript
// Expected shape (contract):
{
  i18n: {
    defaultLocale: "en",
    locales: ["en", "ur"],
    localeConfigs: {
      en: {
        label: "English",
        direction: "ltr",
        htmlLang: "en",
      },
      ur: {
        label: "اردو",  // Urdu label for switcher
        direction: "rtl",
        htmlLang: "ur",
      },
    },
  },
}
```

**Deliverable**: `contracts/docusaurus-config.md`

#### Contract 2: Translation Workflow Input/Output

**Input**:
- English chapter Markdown file (path: `docs/chapter-X.md`)
- Technical glossary (terminology list)
- Translation prompt template

**Output**:
- Translated Markdown file (path: `i18n/ur/docusaurus-docs/chapter-X.md`)
- Translation metadata (translator, date, status)
- Review feedback (if applicable)

**Deliverable**: `contracts/translation-workflow.md`

#### Contract 3: Terminology Glossary

**Format**:
```json
{
  "terms": [
    {
      "id": "api-001",
      "term_en": "API",
      "term_ur": "اے پی آئی",
      "category": "core-concepts",
      "definition_ur": "کمپیوٹر پروگراموں کے درمیان ربط۔",
      "usage_example": "REST API کا استعمال کریں۔"
    }
  ]
}
```

**Deliverable**: `contracts/terminology-glossary.md` + `glossary.json`

### 1.3 Quickstart & Workflow Guide

**Output**: `quickstart.md`

```markdown
# Quickstart: Translate a Chapter to Urdu

## Prerequisites
- [ ] Docusaurus i18n configured (ur locale added)
- [ ] Terminology glossary loaded
- [ ] Access to Claude API or translation service

## Steps

### 1. Extract Chapter
- Copy English chapter: `docs/chapter-X.md`
- Create target: `i18n/ur/docusaurus-docs/chapter-X.md`

### 2. Translate
- Use AI prompt with chapter content + glossary
- Preserve Markdown formatting
- Apply terminology glossary

### 3. Review
- Verify formatting (headings, code, tables)
- Verify links (internal + external)
- Check terminology consistency
- Verify Urdu readability

### 4. Publish
- Commit translated file
- Verify build succeeds
- Test language switcher

## Review Checklist
- [ ] All headings preserved
- [ ] All code blocks intact
- [ ] All links functional
- [ ] Technical terms from glossary
- [ ] Urdu text is natural and readable
- [ ] No English text except code identifiers
- [ ] Build passes without errors
```

### 1.4 Agent Context Update (Conditional)

If external agent context files exist (`.claude/` or similar), they should be updated with:
- Docusaurus v2 i18n patterns
- Urdu language guidelines (RTL, script, terminology)
- Translation workflow SOP
- Technical term glossary

**Note**: This step skipped if no agent context mechanism exists in the project.

---

## Complexity Tracking

| Aspect | Complexity | Justification | Risk Mitigation |
|--------|-----------|---------------|-----------------|
| **Docusaurus i18n Config** | Medium | Requires correct i18n setup; mistakes break routing | Research Task 1 validates current config; testing confirms routing works |
| **RTL Text Support** | Medium | RTL layout can break if CSS not configured | Research Task 2 identifies required CSS changes; manual testing of layout |
| **AI Translation Quality** | High | Translation quality directly impacts user experience | Human review for every chapter (Research Task 3); terminology glossary ensures consistency |
| **Link Resolution Across Locales** | Medium | Links must point to correct Urdu pages | Research Task 5 tests link behavior; fallback strategy for untranslated chapters |
| **Build Verification** | Low | Standard Docusaurus build process | Research Task 6 documents verification; build test before deployment |

---

## Decisions Summary

| Decision | Choice | Rationale | Alternatives Rejected |
|----------|--------|-----------|----------------------|
| **Translation Method** | AI-assisted (Claude) + Human Review | Cost-effective, fast iteration, quality assurance | Full manual (slower), Google Translate (poor terminology), No translation |
| **Terminology Handling** | Glossary-driven (transliteration + English-with-context) | Ensures consistency, balances accessibility with precision | Pure English (less accessible), Pure Urdu (loss of technical precision) |
| **Storage** | Static Markdown files in `i18n/ur/` | Aligns with Docusaurus v2, SEO-friendly, version-controllable | Database-backed (over-engineered), CDN service (external dependency) |
| **Build Strategy** | Docusaurus v2 i18n (static generation at build time) | Native support, no runtime overhead, SEO optimal | Client-side translation (bad UX, not SEO), API-based (introduces latency) |
| **Scope** | All English chapters translated (100%) | Consistent user experience; no confusing mixed-language site | Partial translation (confusing navigation) |

---

## Next Steps (Phase 2)

After Phase 1 design approval:

1. **Run `/sp.tasks`** to generate actionable implementation tasks
2. **Execute Phase 0 research** (concurrent with task generation if needed)
3. **Implement Phase 1 design** (Docusaurus config, folder structure, initial chapter translation)
4. **Execute Phase 2 tasks** (translate all chapters, review, build, test, deploy)

---

## Assumptions

- Docusaurus v2.x is installed and configured in `content/` directory
- English chapters exist in `content/docs/` (exact count TBD in research)
- Node.js + npm available for build process
- Access to AI translation service (Claude API) or willing to use alternative
- Human reviewers available for translation QA
- Urdu script rendering supported by Docusaurus theme (or minimal CSS adjustments acceptable)
- SEO indexing of static HTML acceptable (no dynamic meta tags needed)

---

## Risks & Mitigations

| Risk | Impact | Mitigation |
|------|--------|-----------|
| **Poor translation quality** | High - confuses Urdu readers | Human review for every chapter; terminology glossary; iterative refinement |
| **RTL layout breakage** | Medium - unprofessional appearance | Research Task 2; manual layout testing; CSS adjustments before production |
| **Untranslated chapter fallback unclear** | Medium - navigation confusion | Research Task 5; define and document fallback behavior; test before release |
| **Build time increases significantly** | Low - delays deployment | Document build baseline; optimize i18n config if needed |
| **Link resolution issues** | Medium - broken internal links | Research Task 5; automated link validation; manual spot-check |

