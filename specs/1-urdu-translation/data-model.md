# Data Model: Urdu i18n Translation System

**Date**: 2025-12-28
**Feature**: Build Urdu i18n for Docusaurus Book
**Purpose**: Define entities, relationships, and validation rules for the translation workflow

---

## Core Entities

### Entity 1: Chapter / Document

**Purpose**: Represents a single chapter or documentation page in the book, with English source and Urdu translation.

#### Fields

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `id` | UUID or slug | Yes | Unique identifier (e.g., "chapter-1", "intro") |
| `title_en` | String | Yes | English chapter title (e.g., "Introduction to ROS 2") |
| `title_ur` | String | No (initially) | Urdu translation of title (populated during translation) |
| `slug` | String | Yes | URL-friendly identifier (e.g., "introduction-to-ros-2") |
| `path_en` | String | Yes | English file path (e.g., "docs/chapter-1.md") |
| `path_ur` | String | No (initially) | Urdu file path (e.g., "i18n/ur/docusaurus-docs/chapter-1.md") |
| `content_en` | Text (Markdown) | Yes | English chapter content (full Markdown) |
| `content_ur` | Text (Markdown) | No (initially) | Urdu chapter content (full Markdown, populated during translation) |
| `word_count_en` | Integer | Yes | English word count (for effort estimation) |
| `word_count_ur` | Integer | No (initially) | Urdu word count (may differ due to language differences) |
| `status` | Enum | Yes | Current workflow status: `pending` \| `translating` \| `translated` \| `reviewing` \| `reviewed` \| `approved` \| `published` |
| `created_at` | DateTime | Yes | When chapter was added to book |
| `translated_at` | DateTime | No | When AI translation was generated |
| `reviewed_at` | DateTime | No | When human review was completed |
| `published_at` | DateTime | No | When chapter was deployed to production |
| `has_code_blocks` | Boolean | Yes | Whether chapter contains code blocks (for review focus) |
| `has_tables` | Boolean | Yes | Whether chapter contains tables (for formatting verification) |
| `has_links` | Boolean | Yes | Whether chapter contains internal or external links (for link validation) |

#### Relationships

- **References**: `TerminologyEntry` (many-to-many via terms used in content)
- **References**: `TranslationMetadata` (one-to-one; metadata about the translation)
- **Grouped by**: `Section` (hierarchical organization of chapters)

#### Validation Rules

| Rule | Description | Enforcement |
|------|-------------|------------|
| `id` unique | Chapter ID must be globally unique | Database unique constraint |
| `slug` unique | URL slug must be globally unique | Database unique constraint |
| `path_en` exists | English Markdown file must exist in `docs/` | File system check before import |
| `title_en` non-empty | English title required | Length >= 1 |
| `content_en` valid Markdown | English content must be parseable Markdown | Markdown parser validation |
| `status` valid | Status must be one of enum values | Enum constraint |
| `content_ur` valid Markdown | Urdu content must be parseable Markdown (when populated) | Markdown parser validation |
| `Urdu script` | Urdu content must use Urdu script (when populated) | Character set validation (U+0600-U+06FF) |
| `path_ur` mirrors structure | Urdu path must match English structure (chapter name preserved) | Path naming convention check |

#### State Transitions

```
pending → translating → translated → reviewing → reviewed → approved → published

Fallback paths:
- translated → translating (if re-translation needed)
- reviewed → reviewing (if revisions requested)
- approved → reviewing (if issues found)
```

#### Example

```json
{
  "id": "chapter-1",
  "title_en": "Introduction to ROS 2",
  "title_ur": "ROS 2 کا تعارف",
  "slug": "introduction-to-ros-2",
  "path_en": "docs/chapter-1.md",
  "path_ur": "i18n/ur/docusaurus-docs/chapter-1.md",
  "status": "approved",
  "word_count_en": 2500,
  "word_count_ur": 2800,
  "has_code_blocks": true,
  "has_tables": true,
  "has_links": true,
  "created_at": "2025-12-01T00:00:00Z",
  "translated_at": "2025-12-20T10:30:00Z",
  "reviewed_at": "2025-12-25T14:15:00Z",
  "published_at": "2025-12-28T09:00:00Z"
}
```

---

### Entity 2: Terminology Entry

**Purpose**: Represents a technical term with its Urdu equivalent or transliteration, used consistently across all chapters.

#### Fields

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `id` | UUID or slug | Yes | Unique identifier (e.g., "api-001", "variable-001") |
| `term_en` | String | Yes | English technical term (e.g., "API", "variable", "database") |
| `term_ur` | String | Yes | Urdu transliteration or translation (e.g., "اے پی آئی", "متغیر", "ڈیٹا بیس") |
| `category` | Enum | Yes | Term category: `core-concepts` \| `programming-basics` \| `data-structures` \| `oop` \| `robotics` \| `frameworks` \| `math-algorithms` \| `other` |
| `definition_ur` | Text | No | Urdu definition or explanation (1-2 sentences) |
| `usage_example` | String | No | Example sentence using the term in Urdu context |
| `approved_by` | String | No | Name/ID of reviewer who approved this term |
| `approved_at` | DateTime | No | When the term was approved |
| `notes` | String | No | Translation notes (e.g., "Transliterated; widely recognized in tech community") |
| `alternate_forms_ur` | Array[String] | No | Alternative Urdu forms or spellings (if applicable) |
| `priority` | Enum | Yes | Usage frequency: `high` (used in >5 chapters) \| `medium` (used in 2-4 chapters) \| `low` (used in 1 chapter) |
| `status` | Enum | Yes | Approval status: `proposed` \| `under-review` \| `approved` \| `deprecated` |

#### Relationships

- **Referenced by**: `Chapter` (many-to-many; terms used in chapter content)
- **Approved by**: `Reviewer` (person who validated the term)

#### Validation Rules

| Rule | Description | Enforcement |
|------|-------------|------------|
| `id` unique | Term ID must be globally unique | Database unique constraint |
| `term_en` non-empty | English term required | Length >= 1 |
| `term_ur` non-empty | Urdu term required | Length >= 1 |
| `term_ur` valid script | Urdu term must use Urdu script | Character set validation (U+0600-U+06FF) |
| `category` valid | Category must be one of enum values | Enum constraint |
| `definition_ur` optional but consistent | If provided, must be 1-3 sentences | Length <= 500 characters |
| `status` valid | Status must be one of enum values | Enum constraint |
| `approved_terms only` | Terms used in published chapters must have status = `approved` | Workflow constraint |

#### Priority Assignment Logic

```
Priority = HIGH if:
  - term appears in >5 chapters
  OR term is core programming concept (variable, function, loop, etc.)

Priority = MEDIUM if:
  - term appears in 2-4 chapters
  OR term is domain-specific (ROS, Gazebo, etc.)

Priority = LOW if:
  - term appears in 1 chapter
  OR term is niche/specialized
```

#### Example

```json
{
  "id": "api-001",
  "term_en": "API",
  "term_ur": "اے پی آئی",
  "category": "core-concepts",
  "definition_ur": "Application Programming Interface - کمپیوٹر پروگراموں کے درمیان معلومات کا تبادلہ کرنے کا طریقہ",
  "usage_example": "REST API کا استعمال کریں تاکہ سرور سے ڈیٹا حاصل کریں۔",
  "approved_by": "fatima.ali@example.com",
  "approved_at": "2025-12-20T12:00:00Z",
  "notes": "Transliterated; widely recognized in tech community. Used in >10 chapters.",
  "priority": "high",
  "status": "approved"
}
```

---

### Entity 3: Translation Metadata

**Purpose**: Tracks the translation workflow and review status for each chapter.

#### Fields

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `id` | UUID | Yes | Unique metadata ID |
| `chapter_id` | Foreign Key | Yes | Reference to Chapter entity |
| `ai_model` | String | Yes | AI model used (e.g., "claude-3-opus-20250101", "claude-3-sonnet") |
| `translator_ai_version` | String | No | Version/hash of translation prompt template used |
| `glossary_version` | String | Yes | Version of terminology glossary used (e.g., "1.0", "1.1") |
| `translated_at` | DateTime | Yes | When AI translation was generated |
| `word_count_translated` | Integer | Yes | Word count of translated content (for billing/metrics) |
| `translation_cost_tokens` | Integer | No | Estimated token cost (if using API with token-based billing) |
| `review_status` | Enum | Yes | Review workflow: `pending-review` \| `under-review` \| `approved` \| `changes-requested` \| `rejected` |
| `reviewer_name` | String | No | Name/ID of human reviewer |
| `review_comments` | Text | No | Reviewer feedback or change requests |
| `review_completed_at` | DateTime | No | When review was completed |
| `review_iterations` | Integer | Yes (default 0) | Number of review cycles (translation → review → re-translate, etc.) |
| `approved_by` | String | No | Final approver name/ID |
| `approved_at` | DateTime | No | When chapter was approved for publication |
| `issues_found` | Array[String] | No | List of issues found during review (e.g., ["missing headings", "broken links"]) |
| `issues_resolved` | Boolean | No | Whether all issues have been resolved |
| `notes` | Text | No | General notes (quality observations, specific challenges, etc.) |

#### Relationships

- **Belongs to**: `Chapter` (one-to-one)
- **Reviewed by**: `Reviewer` (person entity, if tracked separately)
- **References**: `TerminologyGlossary` (version of glossary used)

#### Validation Rules

| Rule | Description | Enforcement |
|------|-------------|------------|
| `chapter_id` exists | Referenced chapter must exist | Foreign key constraint |
| `review_status` valid | Must be one of enum values | Enum constraint |
| `review_status` consistency | Cannot be "approved" without reviewer and approval date | Conditional constraint |
| `review_completed_at` after translated_at | Review date cannot precede translation date | Timestamp validation |
| `approved_at` after review_completed_at | Approval date cannot precede review completion | Timestamp validation |
| `word_count_translated` > 0 | Translated content must have words | Numeric validation |

#### Status Transitions

```
pending-review → under-review → approved
                           ↓
                    changes-requested → under-review (restart)
                           ↓
                        rejected (restart translation)
```

#### Example

```json
{
  "id": "meta-chapter-1-v1",
  "chapter_id": "chapter-1",
  "ai_model": "claude-3-opus-20250101",
  "translator_ai_version": "v2.1",
  "glossary_version": "1.0",
  "translated_at": "2025-12-20T10:30:00Z",
  "word_count_translated": 2800,
  "translation_cost_tokens": 8500,
  "review_status": "approved",
  "reviewer_name": "Fatima Ali",
  "review_comments": "Excellent translation. Terminology consistent. Minor formatting fix in code blocks.",
  "review_completed_at": "2025-12-25T14:15:00Z",
  "review_iterations": 1,
  "approved_by": "Fatima Ali",
  "approved_at": "2025-12-25T15:00:00Z",
  "issues_found": ["code block formatting"],
  "issues_resolved": true,
  "notes": "Strong Urdu language quality. Idiomatic phrasing throughout. Ready for publication."
}
```

---

### Entity 4: Language Configuration

**Purpose**: Defines the language/locale configuration for the site, used in Docusaurus i18n config.

#### Fields

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `locale_code` | String | Yes | ISO 639-1 language code (e.g., "ur" for Urdu, "en" for English) |
| `language_name` | String | Yes | Full language name in English (e.g., "Urdu") |
| `language_name_native` | String | Yes | Language name in native script (e.g., "اردو") |
| `direction` | Enum | Yes | Text direction: `ltr` (left-to-right, English) \| `rtl` (right-to-left, Urdu) |
| `html_lang` | String | Yes | HTML `lang` attribute value (e.g., "ur", "ur-PK" for Pakistan-specific Urdu) |
| `enabled` | Boolean | Yes | Whether this locale is active on the site |
| `supported_from_date` | DateTime | Yes | When this locale was first made available |
| `translation_complete` | Boolean | Yes | Whether all chapters are translated (for this locale) |
| `translation_coverage_percent` | Integer | No | Percentage of chapters translated (0-100) |
| `default_fallback_locale` | String | Yes | Fallback locale for untranslated content (e.g., "en" for Urdu) |
| `font_family_primary` | String | No | CSS font family for primary text (e.g., "Arial, sans-serif") |
| `font_family_code` | String | No | CSS font family for code blocks (e.g., "Courier New, monospace") |
| `notes` | String | No | Configuration notes (e.g., "Urdu uses Arabic numerals 0-9; Arabic script U+0600-U+06FF") |

#### Relationships

- **References**: `Chapter` (all chapters may be in this locale)

#### Validation Rules

| Rule | Description | Enforcement |
|------|-------------|------------|
| `locale_code` unique | Locale code must be globally unique | Database unique constraint |
| `locale_code` format | Must be valid ISO 639-1 code (2-3 characters) | Regex validation |
| `direction` valid | Must be "ltr" or "rtl" | Enum constraint |
| `html_lang` non-empty | HTML lang attribute required | Length >= 2 |
| `translation_coverage_percent` range | Must be 0-100 | Range validation |
| `enabled` with coverage | If enabled=true, should have translation_coverage_percent >= 80 (convention) | Business rule |

#### Example

```json
{
  "locale_code": "ur",
  "language_name": "Urdu",
  "language_name_native": "اردو",
  "direction": "rtl",
  "html_lang": "ur-PK",
  "enabled": true,
  "supported_from_date": "2025-12-28T00:00:00Z",
  "translation_complete": false,
  "translation_coverage_percent": 45,
  "default_fallback_locale": "en",
  "font_family_primary": "Arial Unicode MS, Arial, sans-serif",
  "font_family_code": "Courier New, monospace",
  "notes": "Urdu script uses U+0600-U+06FF (Arabic/Urdu block). RTL rendering enabled. Fallback to English for untranslated chapters."
}
```

---

## Relationships & Data Flow

### Entity Diagram (Simplified)

```
Chapter (1) ---(has many)--- TerminologyEntry
    |                              |
    |                              |
    +---(has one)--- TranslationMetadata
    |
    +---(belongs to)--- LanguageConfiguration
    |
    +---(uses)--- TerminologyEntry (via terms in content)

Workflow Flow:
Chapter (pending)
  → Translation (Claude API)
  → TranslationMetadata (created, review_status = pending-review)
  → Human Review
  → TranslationMetadata (updated with feedback)
  → Re-translation (if needed)
  → Approval
  → TranslationMetadata (review_status = approved)
  → Build & Publish
  → Chapter (status = published)
```

### Translation Workflow Sequence

```
1. Extract English Chapter (source material)
   ↓
2. AI Translation
   - Input: Chapter content + Terminology Glossary
   - Output: Urdu Markdown content
   ↓
3. Create TranslationMetadata
   - Status: pending-review
   - Glossary version: current
   ↓
4. Human Review
   - Reviewer checks against Review Checklist
   - Validates terminology, formatting, readability
   ↓
5. Decision Point
   if approved:
     → Update TranslationMetadata (approved)
     → Commit file
     → Build & Deploy
   else (changes requested):
     → Update TranslationMetadata (changes-requested)
     → Re-translate with feedback
     → Re-review (iteration)
```

---

## Data Validation & Integrity

### Chapter Content Validation

When translating a chapter, ensure:

```javascript
validation_rules = {
  markdown_syntax: "Valid Markdown (headings, lists, code blocks, tables)",
  heading_structure: "Matches English structure (H1, H2, H3 hierarchy preserved)",
  code_blocks: "All code blocks present and readable (backticks preserved)",
  links: "All links (internal & external) preserved and functional",
  tables: "Table structure preserved; rows and columns intact",
  formatting: "Bold, italics, inline code preserved",
  script: "Urdu content uses Urdu script (U+0600-U+06FF for Arabic/Urdu block)",
  language: "No English text except code identifiers and comments",
  terminology: "All repeated terms use glossary equivalents consistently",
}
```

### Terminology Consistency Validation

```javascript
consistency_check = {
  glossary_terms: "All terms used in chapter must be in glossary (or explicitly marked as new)",
  term_repetition: "Same term always uses same Urdu equivalent within and across chapters",
  code_identifiers: "Code variable/function names never translated; remain in English",
  proper_nouns: "ROS 2, Docusaurus, Unity, etc. remain as-is (transliteration or English as appropriate)",
}
```

---

## Notes for Implementation

1. **Version Control**: Store all entities (chapters, glossary, metadata) in version-controlled files (JSON, Markdown) or database
2. **Audit Trail**: Log all state transitions (status changes, reviews, approvals) for transparency
3. **Metrics**: Track word counts, translation times, review cycles for process improvement
4. **Scalability**: Design for future languages (Spanish, Chinese, etc.) beyond Urdu
5. **Glossary Evolution**: Glossary grows with each chapter translated; plan for version management and rollback if needed

