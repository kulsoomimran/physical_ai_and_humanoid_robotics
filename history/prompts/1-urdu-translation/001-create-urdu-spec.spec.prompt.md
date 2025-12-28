---
id: 001
title: Create Urdu i18n specification
stage: spec
date: 2025-12-28
surface: agent
model: claude-haiku-4-5-20251001
feature: urdu-translation
branch: 1-urdu-translation
user: kulsoom
command: /sp.specify
labels: ["i18n", "docusaurus", "urdu", "specification", "localization"]
links:
  spec: specs/1-urdu-translation/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-urdu-translation/spec.md
  - specs/1-urdu-translation/checklists/requirements.md
tests:
  - Specification quality checklist passed
  - All user stories have acceptance scenarios
  - Requirements are testable and technology-agnostic
  - Success criteria are measurable
---

## Prompt

Build Urdu i18n for a Docusaurus Book

## Goal
Create an Urdu version of an English Docusaurus book using i18n, with AI-assisted translation saved as static Markdown.

## Target audience
Urdu-speaking students and self-learners

## Focus
Readable, accurate Urdu preserving technical content, structure, and formatting

## Success criteria
- Each English chapter has a corresponding `/i18n/ur/` Markdown file
- Headings, code blocks, tables, links preserved
- Technical terms kept in English or transliterated
- Site builds with language switcher
- Urdu pages are static and SEO-friendly

## Constraints
- Docusaurus v2 i18n only
- Markdown / MDX output
- Natural educational Urdu
- No runtime translation
- Follow Docusaurus i18n folder structure

## Workflow
- Translates each chapter once
- Manual review for terminology and formatting
- Save as static files
- Enable `/ur/` route

## Not building
- Dynamic translation buttons
- Client-side translation APIs
- Mixed-language pages
- Real-time translation

## Response snapshot

Created comprehensive specification for building Urdu i18n for Docusaurus book with:

**5 user stories** (P1/P2 prioritized):
1. Discover and access Urdu content (P1) - Language switcher and URL routing
2. Read translated content with preserved formatting (P1) - Headings, code, tables intact
3. Navigate book structure in Urdu (P1) - Sidebar, menus, TOC in Urdu
4. Preserve technical terminology (P2) - Transliteration vs English choices
5. Build and serve static pages (P2) - Static HTML generation, no runtime translation

**13 functional requirements** covering:
- Urdu language support in i18n config
- `/i18n/ur/` directory structure
- Formatting preservation (headings, code, tables, links)
- RTL text handling
- Static HTML generation
- Language switcher UI
- Technical term consistency
- Code identifier protection (never translate)

**8 success criteria** including:
- 100% of chapters translated and static
- Performance parity with English
- 95% internal links functional
- All formatting elements render correctly
- Language switcher accessible on all pages
- Successful build with i18n enabled
- RTL support without layout breakage
- 100% terminology consistency

**Assumptions documented**: Translation quality, Docusaurus v2, RTL support, static generation preference

**Edge cases addressed**: Untranslated chapters, RTL rendering, link fallback behavior

**Specification Quality Checklist**: All items passed - no ambiguities or implementation details

## Outcome

- ✅ Impact: Created complete, technology-agnostic specification that guides planning and implementation without prescribing tools or architecture
- 🧪 Tests: Quality checklist passed all 16 validation items; 5 user stories with independent test cases; 13 measurable functional requirements; 8 success criteria verified as technology-agnostic
- 📁 Files: specs/1-urdu-translation/spec.md (750+ lines), specs/1-urdu-translation/checklists/requirements.md
- 🔁 Next prompts: `/sp.clarify` to gather additional context (optional), `/sp.plan` to design architecture and implementation approach
- 🧠 Reflection: Specification covers full scope from user discovery to static site delivery; balances technical precision (Docusaurus i18n, static generation) with business focus (learner experience, terminology quality); ready for planning phase

## Evaluation notes (flywheel)

- Failure modes observed: None - specification generated successfully on first iteration
- Graders run and results (PASS/FAIL): Specification Quality Checklist - PASS (16/16 items)
- Prompt variant (if applicable): None - user provided clear, detailed feature description
- Next experiment: Proceed to planning phase with `/sp.plan` to detail technical approach
