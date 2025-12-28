# Feature Specification: Build Urdu i18n for Docusaurus Book

**Feature Branch**: `1-urdu-translation`
**Created**: 2025-12-28
**Status**: Draft
**Input**: Build Urdu i18n for a Docusaurus Book with AI-assisted translation saved as static Markdown

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Discover and Access Urdu Content (Priority: P1)

Urdu-speaking students and self-learners visit the Docusaurus site, discover that an Urdu language version is available, and can seamlessly switch to reading the book in their native language.

**Why this priority**: This is the foundational user journey. Without the ability to discover and access Urdu content, the entire feature has no value. All other features depend on this working.

**Independent Test**: Can be fully tested by: (1) Loading the site, (2) Finding the language switcher, (3) Selecting Urdu, (4) Verifying the URL changes to `/ur/` route and Urdu content loads. Delivers: Native language access to the book.

**Acceptance Scenarios**:

1. **Given** the site loads in English (default), **When** user clicks the language switcher, **Then** Urdu is listed as an available option
2. **Given** user selects Urdu from the language menu, **When** the page reloads, **Then** the URL changes to `/ur/docs/...` and all content displays in Urdu
3. **Given** user is on an Urdu page and clicks the language switcher, **When** user selects English, **Then** the URL changes back to `/docs/...` with English content
4. **Given** user bookmarks a Urdu page at `/ur/docs/chapter-3/`, **When** they return to the bookmark, **Then** the page loads correctly in Urdu

---

### User Story 2 - Read Translated Content with Preserved Formatting (Priority: P1)

Urdu-speaking readers open a translated chapter and can read the content clearly, with proper Urdu typography and all original formatting (headings, code blocks, tables, links) intact and functional.

**Why this priority**: Once Urdu language is accessible, the core value is reading comprehensible, well-formatted technical content. Poor formatting or broken code examples significantly harm learning experience.

**Independent Test**: Can be fully tested by: (1) Opening a translated chapter with mixed content (headings, paragraphs, code, tables), (2) Verifying all elements are readable and functional, (3) Clicking links to ensure they work. Delivers: High-quality, readable educational content in Urdu.

**Acceptance Scenarios**:

1. **Given** a chapter has headings, paragraphs, and code blocks, **When** it's viewed in Urdu, **Then** all headings remain as headings (H1, H2, H3), paragraphs are properly formatted, and code blocks display correctly with syntax highlighting
2. **Given** a chapter contains a table with data, **When** it's viewed in Urdu, **Then** the table structure is preserved and rows/columns are readable
3. **Given** a chapter includes links (internal or external), **When** the page is in Urdu, **Then** links remain functional and navigate correctly
4. **Given** a paragraph contains inline code or monospace text, **When** it's displayed in Urdu, **Then** code formatting is preserved and distinguishable from regular text

---

### User Story 3 - Navigate Book Structure in Urdu (Priority: P1)

Urdu-speaking readers can navigate the book's sidebar menu, table of contents, and cross-chapter links entirely in Urdu, making it feel like a native Urdu educational resource.

**Why this priority**: Navigation menus and UI labels significantly affect user experience. If these remain in English, the Urdu experience feels incomplete and confusing. This is part of the core P1 experience.

**Independent Test**: Can be fully tested by: (1) Loading a Urdu page, (2) Checking sidebar menu items, (3) Clicking internal links, (4) Verifying TOC is in Urdu. Delivers: Seamless, native-language navigation.

**Acceptance Scenarios**:

1. **Given** a Urdu page is loaded, **When** the sidebar is visible, **Then** all menu item labels and section headings are in Urdu
2. **Given** an internal link points to another chapter, **When** the user hovers over or clicks it, **Then** the link destination page loads in Urdu
3. **Given** a chapter's table of contents is present, **When** the page is in Urdu, **Then** all heading links in the TOC use Urdu text

---

### User Story 4 - Preserve Technical Terminology (Priority: P2)

Urdu readers encounter technical terms that are either clearly transliterated (phonetically rendered in Urdu script) or kept in English where Urdu translation would obscure meaning, ensuring technical clarity without losing educational value.

**Why this priority**: Technical accuracy is important for learning, but overly literal translations of programming concepts can confuse readers. Smart terminology choices (transliteration + context) make content accessible without compromising precision.

**Independent Test**: Can be fully tested by: (1) Reading chapters with technical terms (API, database, algorithm names), (2) Verifying terms are either transliterated consistently or left in English with context, (3) Confirming glossary or notes explain transliteration choices. Delivers: Technically sound, learner-friendly Urdu.

**Acceptance Scenarios**:

1. **Given** a chapter discusses "API", **When** viewed in Urdu, **Then** it appears as transliterated "اے پی آئی" (or similar standard transliteration) consistently throughout the doc
2. **Given** a chapter uses a specific term (e.g., "variable", "loop", "database"), **When** viewed in Urdu, **Then** the term is either: (a) transliterated in Urdu, or (b) kept in English with Urdu explanation in context
3. **Given** a code variable or function name appears in text, **When** the page is in Urdu, **Then** code identifiers remain in English (never translated), only surrounding explanation is in Urdu

---

### User Story 5 - Build and Serve Static Urdu Pages (Priority: P2)

The development team successfully builds the Docusaurus site with Urdu i18n enabled, all Urdu pages are generated as static HTML, and the site serves both English and Urdu routes without runtime translation overhead.

**Why this priority**: This ensures the site is performant and SEO-friendly. Static generation means pages are fast, searchable by crawlers, and don't require server-side translation. Critical for production quality.

**Independent Test**: Can be fully tested by: (1) Running build process, (2) Verifying `/build/i18n/ur/` directory contains static HTML files, (3) Testing site serves both `/docs/` and `/ur/docs/` routes correctly. Delivers: Production-ready, SEO-friendly site.

**Acceptance Scenarios**:

1. **Given** the build process runs, **When** the build completes, **Then** the output includes a `/build/i18n/ur/` directory with static HTML files for all translated chapters
2. **Given** the Docusaurus dev server runs, **When** a request comes for `/ur/docs/chapter-1/`, **Then** the Urdu version of chapter 1 loads without any client-side translation delay
3. **Given** both English and Urdu versions exist, **When** a crawler visits `/ur/docs/`, **Then** the HTML is complete and static (not requiring JavaScript to render content)

---

### Edge Cases

- What happens if a chapter exists in English but hasn't been translated to Urdu yet? (Fallback behavior: show English, or show message that translation is pending)
- How does the site handle special characters and RTL (right-to-left) rendering for Urdu script?
- What happens if a link in the Urdu version points to an untranslated chapter?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST support Urdu (ur) as a secondary language alongside English (en) in Docusaurus i18n configuration
- **FR-002**: System MUST generate a `/i18n/ur/` directory structure mirroring the English `/docs/` structure with translated Markdown files
- **FR-003**: Each translated chapter MUST preserve original heading hierarchy (H1, H2, H3, etc.) and structure
- **FR-004**: System MUST preserve all code blocks, tables, and lists with original formatting and readability
- **FR-005**: System MUST preserve all internal and external links; links MUST remain functional and point to the correct Urdu or English pages
- **FR-006**: All navigation UI elements (sidebar menu, breadcrumbs, language switcher) MUST display labels and text in Urdu when viewing Urdu pages
- **FR-007**: System MUST handle RTL (right-to-left) text rendering for Urdu script without breaking layout or readability
- **FR-008**: System MUST build and generate static HTML files for all Urdu pages (no runtime translation)
- **FR-009**: Urdu translation MUST follow Docusaurus v2 i18n folder structure conventions
- **FR-010**: System MUST provide a language switcher UI element visible on all pages, allowing users to toggle between English and Urdu
- **FR-011**: Technical terms MUST be handled consistently: either transliterated into Urdu script using standard conventions, or kept in English with Urdu context
- **FR-012**: Code identifiers (variable names, function names, class names) MUST NOT be translated; only surrounding explanatory text is translated
- **FR-013**: Translated content MUST maintain SEO-friendly static pages (no JavaScript-based runtime translation)

### Key Entities

- **Chapter/Document**: A single Markdown file representing a section of the book. Has a title, content (paragraphs, code, tables), and internal/external links. Exists in both English and Urdu versions.
- **Terminology Glossary**: A record of technical terms and their Urdu equivalents or transliterations, ensuring consistency across all chapters.
- **Language Configuration**: Docusaurus i18n configuration defining supported languages, default language, locale settings, and routing rules.
- **Translation Asset**: A Markdown file in `/i18n/ur/` containing translated chapter content with formatting and structure preserved.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: All English chapters (100% of existing English content) have corresponding Urdu translations in `/i18n/ur/` that are SEO-indexable static pages
- **SC-002**: Urdu pages load with no noticeable delay compared to English pages (performance parity)
- **SC-003**: 95% of internal links in Urdu content point to the correct Urdu page (or fallback to English if translation pending)
- **SC-004**: All formatting elements (headings, code blocks, tables, lists) render correctly and are readable in Urdu pages
- **SC-005**: Language switcher is accessible and functional on 100% of pages; switching languages maintains user's current page context (e.g., switching on chapter 3 takes user to Urdu chapter 3, not homepage)
- **SC-006**: The site builds successfully with Urdu i18n enabled, and build process completes without errors
- **SC-007**: Site handles RTL text without layout breakage; Urdu text is readable and properly aligned
- **SC-008**: Terminology consistency: 100% of repeated technical terms use the same transliteration or English equivalent across all chapters

## Assumptions

- **Urdu Translation Quality**: Translations are created by fluent Urdu speakers with technical knowledge or with AI assistance followed by human review for accuracy and readability
- **Docusaurus Version**: Project uses Docusaurus v2.x (not v1.x) which has built-in i18n support
- **RTL Support**: The Docusaurus theme and CSS framework used support RTL rendering; minimal custom CSS adjustments may be needed for Urdu
- **No Dynamic Translation**: Static Markdown translation is preferred over runtime translation APIs (e.g., Google Translate, Azure Translator); all Urdu content is pre-translated and saved as static files
- **Terminology Reference**: A basic glossary or reference for consistent technical term transliteration is created and maintained (can be a simple document or external resource)
- **Focus on Content**: UI language strings (sidebar labels, menus) can be translated through Docusaurus i18n config or manually placed in Urdu translation files; assumed to be within scope
- **Link Structure**: Internal links will use locale-aware routing; Docusaurus handles link resolution (e.g., `/docs/chapter/` becomes `/ur/docs/chapter/` on Urdu pages)

## Constraints

- **Docusaurus v2 i18n Only**: Must use Docusaurus's native i18n feature; cannot use third-party translation plugins or middleware
- **Markdown / MDX Output**: Translations must remain as Markdown or MDX files, not converted to other formats
- **Static Generation**: All Urdu content must be pre-translated and built as static HTML; no client-side or runtime translation
- **No Mixed-Language Pages**: Each page is either fully English or fully Urdu; no page contains both languages side-by-side (except code comments/identifiers which remain English)
- **Docusaurus Folder Structure**: Translations must follow Docusaurus's `/i18n/<locale>/` convention
- **Natural Urdu**: Translations must be idiomatic, natural Urdu suitable for self-learners and students; not literal word-for-word translations
- **Manual Review Required**: AI-generated translations must be reviewed by humans to ensure quality, accuracy, and appropriateness
- **Build Performance**: Build times should not increase significantly; static generation is preferred for performance

## Out of Scope

- Dynamic language detection based on browser locale
- Client-side translation switching (all switching is page-reload based)
- Right-to-left (RTL) CSS framework redesign (assumed existing theme supports RTL or requires minimal adjustments)
- Translation of Docusaurus plugin documentation or API references (only main book content)
- Automated translation tools or API integrations (manual translation workflow)
- Multi-language searchability optimization (search can be implemented in a future iteration)
- Urdu voice or audio versions of the book
