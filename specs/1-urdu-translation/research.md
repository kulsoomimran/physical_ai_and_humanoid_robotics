# Research Summary: Urdu i18n for Docusaurus

**Date**: 2025-12-28
**Branch**: `1-urdu-translation`
**Purpose**: Resolve unknowns and establish implementation approach for Urdu i18n feature

---

## Task 1: Docusaurus v2 i18n Configuration Deep Dive

### Decision
Use Docusaurus v2's native `i18n` configuration in `docusaurus.config.js` to add Urdu (ur) as a secondary locale alongside English (en).

### Rationale
- Docusaurus v2 has built-in i18n support with automatic locale routing (`/docs/...` for English, `/ur/docs/...` for Urdu)
- Configuration-driven approach requires no code changes; all locale handling is delegated to Docusaurus build process
- Static generation at build time ensures no runtime translation overhead
- Native support provides automatic language switcher integration and SEO-friendly routing

### Alternatives Considered
1. **Third-party i18n plugin** (e.g., Docusaurus i18n plugin) - Rejected: adds external dependency; Docusaurus native support is sufficient
2. **Manual folder structure** (separate site builds for each language) - Rejected: duplicates Docusaurus config; harder to maintain
3. **Runtime translation API** (e.g., Google Translate, Anthropic API) - Rejected: poor UX, SEO-unfriendly, ongoing latency cost per spec constraints

### Implementation Details

#### Config Structure (docusaurus.config.js)
```javascript
module.exports = {
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
        htmlLang: 'en',
      },
      ur: {
        label: 'اردو',
        direction: 'rtl',
        htmlLang: 'ur',
      },
    },
  },
  // ... rest of config
};
```

#### File Structure
- English docs: `content/docs/` → routes to `/docs/...`
- Urdu translations: `content/i18n/ur/docusaurus-docs/` → routes to `/ur/docs/...`
- UI strings (nav, menus): `content/i18n/ur/code.json` → translates navbar, sidebar, common UI labels

#### Build Output
- `build/en/` - English static site
- `build/ur/` - Urdu static site
- Both are separate, pre-built static HTML trees; language switcher in navbar provides navigation between them

### Testing Approach
1. Verify current `docusaurus.config.js` exists and Docusaurus version is v2.x
2. Add `i18n` config block with Urdu locale
3. Run `npm run build` and verify both `/build/en/` and `/build/ur/` directories exist
4. Run `npm run start` (dev server) and test language switcher functionality
5. Verify `/ur/docs/` route loads without 404

---

## Task 2: RTL (Right-to-Left) Text Support for Urdu

### Decision
Enable RTL text rendering for Urdu by setting `direction: 'rtl'` in Docusaurus i18n config and applying minimal CSS adjustments to the theme.

### Rationale
- Urdu script is written right-to-left; RTL rendering is essential for readability
- Docusaurus theme (docusaurus-theme-classic by default) supports `dir` HTML attribute; setting it via i18n config applies RTL CSS automatically
- Minimal CSS changes required in most modern themes; Docusaurus handles majority of layout adjustments

### Alternatives Considered
1. **Framework redesign** (custom RTL-aware theme) - Rejected: over-engineered; Docusaurus default theme has RTL support built-in
2. **No RTL support** - Rejected: Urdu text would be unreadable (violates spec requirement for readability)
3. **JavaScript-based RTL toggle** - Rejected: adds client-side complexity; prefer configuration-driven approach

### Implementation Details

#### Docusaurus i18n Config
```javascript
ur: {
  direction: 'rtl',  // Enables RTL for Urdu locale
  htmlLang: 'ur',
  label: 'اردو',
}
```

#### HTML Output
Docusaurus automatically injects `dir="rtl"` and `lang="ur"` into HTML root element:
```html
<html dir="rtl" lang="ur">
  <!-- Urdu content renders right-to-left -->
</html>
```

#### CSS Adjustments (Minimal)
Most modern themes (including docusaurus-theme-classic) automatically handle RTL with CSS logical properties. Potential adjustments:
- Sidebar positioning (may shift from left to right)
- Text alignment (may shift from left to right)
- Padding/margin directions (automatically handled by logical properties)

**Expected**: Most RTL support is automatic; manual CSS tweaks minimal (< 5-10 rules if any).

#### Typography Considerations
- Urdu script (Nastaliq or Naskh fonts) - use system fonts or specify `font-family: 'Urdu', sans-serif;`
- Line height and letter spacing may need adjustment for Urdu fonts
- Numbers in Urdu typically render as Arabic numerals (0-9) or Eastern-Arabic numerals (٠-٩)

### Testing Approach
1. Build Urdu locale and inspect `build/ur/index.html` for `dir="rtl"` and `lang="ur"`
2. Load `/ur/` in browser and verify:
   - Sidebar on right side (if applicable)
   - Text flows right-to-left
   - No horizontal scrolling or clipping
3. Check layout on mobile (responsive RTL support)
4. Verify links and buttons are accessible and properly positioned in RTL context

---

## Task 3: AI Translation Workflow & Quality Assurance

### Decision
Implement a **chapter-concurrent, AI-assisted workflow** with mandatory human review before publication:
1. Extract English chapter
2. Translate via Claude API with domain-specific prompt
3. Apply terminology glossary
4. Manual review and refinement
5. Commit to `i18n/ur/docusaurus-docs/`

### Rationale
- AI translation (Claude) provides fast, consistent baseline translations
- Human review ensures quality, catches terminology errors, validates readability
- Concurrent processing of chapters allows parallel translation + review
- Glossary-driven approach ensures technical term consistency across all chapters

### Alternatives Considered
1. **Full manual translation** - Rejected: slow (weeks vs. days); higher cost
2. **No human review** (AI-only) - Rejected: unacceptable quality risk for educational content
3. **Third-party translation API** (Google Translate, etc.) - Rejected: poor technical term handling; no education domain training
4. **Professional human translator only** - Rejected: higher cost; AI-assisted approach is faster and adequate with review

### Implementation Details

#### Translation Prompt Template (Claude API)

```
You are an expert Urdu translator specializing in educational and technical content.

Your task: Translate the following English chapter to natural, idiomatic Urdu.

IMPORTANT CONSTRAINTS:
1. Preserve all Markdown formatting (headings, bold, italics, code blocks, links, tables)
2. Preserve all code identifiers (variable names, function names, class names) - never translate
3. Preserve all internal links (e.g., [link](../other-chapter/))
4. Preserve all external links (e.g., [link](https://example.com))
5. Apply the terminology glossary below for consistent technical term translation

TERMINOLOGY GLOSSARY:
[Insert glossary in JSON or table format]

CONTENT TO TRANSLATE:
[English chapter Markdown]

OUTPUT FORMAT:
Return ONLY the translated Markdown content. Do not include explanations or notes.
Ensure the output is valid Markdown that matches the structure of the input.
```

#### Workflow Steps

**Step 1: Extraction**
- List all English chapters in `content/docs/`
- Create corresponding directory structure in `i18n/ur/docusaurus-docs/`

**Step 2: Translation (per chapter)**
- Read English chapter: `content/docs/chapter-X.md`
- Call Claude API with prompt + chapter + glossary
- Save translation: `i18n/ur/docusaurus-docs/chapter-X.md`

**Step 3: Review Checklist**
```
[ ] All headings preserved (H1, H2, H3 structure intact)
[ ] All code blocks intact and readable
[ ] All tables structure preserved
[ ] All links (internal + external) present and formatted correctly
[ ] Technical terms from glossary applied consistently
[ ] Urdu text is natural and readable (not literal word-for-word)
[ ] No English text except code identifiers and comments
[ ] Markdown syntax valid (no broken formatting)
[ ] No HTML tags; pure Markdown only
```

**Step 4: Refinement**
- Make corrections based on review checklist
- Request re-translation if major issues (e.g., missing sections)
- Finalize and commit

#### Quality Metrics
- **Consistency**: Terminology glossary ensures 100% consistency for repeated terms
- **Accuracy**: Human review catches translation errors
- **Readability**: Idiomatic Urdu (verified by Urdu-fluent reviewer)

### Testing Approach
1. Translate 1-2 sample chapters using the workflow
2. Have Urdu-fluent reviewer validate quality
3. Refine workflow based on feedback (e.g., prompt adjustments)
4. Iterate until quality threshold met, then scale to all chapters

---

## Task 4: Terminology Transliteration Standards

### Decision
Create a **terminology glossary** with standard Urdu transliterations and English-with-context phrases for programming terms. Transliterations use standard Urdu script; when translation would obscure meaning, keep English with Urdu explanation.

### Rationale
- Programming concepts are often better understood by Urdu learners with English terms (e.g., "API" is universally recognized)
- Urdu transliterations provide localization for branding and education (students learn Urdu names for concepts)
- Consistent glossary prevents terminology fragmentation across chapters
- Glossary serves as reference for translators and reviewers

### Alternatives Considered
1. **Pure English terms** - Rejected: less accessible; violates goal of native-language learning
2. **Pure Urdu translation** - Rejected: may obscure technical precision (e.g., "algorithm" has no perfect Urdu equivalent)
3. **No glossary** (ad-hoc translation per chapter) - Rejected: inconsistency; reviewed will catch and correct

### Implementation Details

#### Glossary Format (JSON)
```json
{
  "glossary_version": "1.0",
  "language": "Urdu",
  "date_created": "2025-12-28",
  "terms": [
    {
      "id": "api-001",
      "term_en": "API",
      "term_ur": "اے پی آئی",
      "category": "core-concepts",
      "definition_ur": "Application Programming Interface - کمپیوٹر پروگراموں کے درمیان معلومات کا تبادلہ کرنے کا طریقہ",
      "usage_example": "REST API کا استعمال کریں۔",
      "notes": "Transliterated; widely recognized in tech community"
    },
    {
      "id": "var-001",
      "term_en": "variable",
      "term_ur": "متغیر",
      "category": "programming-basics",
      "definition_ur": "ڈیٹا کو محفوظ کرنے کے لیے ایک کنٹینر جو قدر کو بدل سکتا ہے",
      "usage_example": "ایک متغیر x بنائیں اور اسے 10 سے سیٹ کریں۔",
      "notes": "Translated to natural Urdu term"
    },
    {
      "id": "func-001",
      "term_en": "function",
      "term_ur": "فنکشن (تکرار شدہ کوڈ)",
      "category": "programming-basics",
      "definition_ur": "کوڈ کا ایک قابلِ استعمال حصہ جو مخصوص کام انجام دیتا ہے",
      "usage_example": "ایک فنکشن لکھیں جو دو نمبروں کو شامل کرے۔",
      "notes": "Transliterated with Urdu explanation"
    },
    {
      "id": "db-001",
      "term_en": "database",
      "term_ur": "ڈیٹا بیس",
      "category": "data-structures",
      "definition_ur": "منظم شدہ معلومات کو محفوظ کرنے کی جگہ",
      "usage_example": "MongoDB ڈیٹا بیس استعمال کریں۔",
      "notes": "Transliterated"
    },
    {
      "id": "loop-001",
      "term_en": "loop",
      "term_ur": "لوپ (دہرایا جانے والا کوڈ)",
      "category": "control-flow",
      "definition_ur": "کوڈ کا ایک حصہ جو بار بار چلتا ہے جب تک شرط صحیح ہے",
      "usage_example": "for لوپ استعمال کریں تاکہ 1 سے 10 تک گنیں۔",
      "notes": "Transliterated with Urdu explanation"
    },
    {
      "id": "ros-001",
      "term_en": "ROS 2 (Robot Operating System)",
      "term_ur": "ROS 2",
      "category": "robotics-frameworks",
      "definition_ur": "روبوٹس کے لیے پروگرامنگ کا فریم ورک",
      "usage_example": "ROS 2 میں ایک node بنائیں۔",
      "notes": "Keep as ROS 2; widely recognized in robotics community"
    }
  ]
}
```

#### Glossary Categories
- **Core Concepts**: API, endpoint, protocol, etc.
- **Programming Basics**: variable, function, loop, condition, etc.
- **Data Structures**: array, list, dictionary, database, etc.
- **OOP**: class, object, inheritance, polymorphism, etc.
- **Robotics**: robot, actuator, sensor, simulation, ROS, etc.
- **Frameworks**: Docusaurus, Gazebo, Unity, NVIDIA Isaac, etc.
- **Math/Algorithms**: algorithm, matrix, neural network, etc.

#### Glossary Maintenance
- Version control: glossary lives in `specs/1-urdu-translation/contracts/terminology-glossary.json`
- Updates: Add new terms as encountered during translation
- Review: Terms approved by Urdu-fluent technical reviewer before use

### Testing Approach
1. Create initial glossary (20-30 core terms)
2. Use glossary in first sample translations
3. Gather feedback from reviewers on terminology choices
4. Refine glossary iteratively
5. Expand glossary as more chapters are translated

---

## Task 5: Link Resolution in i18n Context

### Decision
Docusaurus v2 i18n automatically handles link routing: links in Urdu pages are resolved to Urdu versions when available, falling back to English if not yet translated.

### Rationale
- Docusaurus i18n system internally tracks which chapters are translated
- Relative links (e.g., `[link](../chapter/)`) are automatically localized
- Absolute links (e.g., `[link](/docs/chapter/)`) require locale-aware rewriting or manual adjustment
- Fallback behavior ensures navigation doesn't break for untranslated chapters

### Alternatives Considered
1. **Manual link rewriting during translation** - Rejected: error-prone; Docusaurus should handle automatically
2. **No i18n link resolution** (users see broken links for untranslated chapters) - Rejected: poor UX
3. **Redirect untranslated chapters to English** - Rejected: acceptable but less flexible than fallback

### Implementation Details

#### Link Types & Behavior

**Type 1: Relative Links** (Preferred in Docusaurus)
```markdown
[Link to Chapter 2](../chapter-2/)
```
- **In English**: resolves to `/docs/chapter-2/`
- **In Urdu**: resolves to `/ur/docs/chapter-2/` (if translated) or `/docs/chapter-2/` (if not yet translated, fallback)
- Docusaurus handles automatically; no changes needed

**Type 2: Absolute Links** (May need adjustment)
```markdown
[Link to Chapter 2](/docs/chapter-2/)
```
- **In English**: resolves to `/docs/chapter-2/`
- **In Urdu**: resolves to `/docs/chapter-2/` (does NOT automatically localize to `/ur/docs/...`)
- **Solution**: Avoid absolute links; use relative links instead or manually rewrite to `/ur/docs/chapter-2/` during translation

**Type 3: External Links** (No localization needed)
```markdown
[External Link](https://example.com)
```
- Unchanged; external links work the same in both locales

#### Docusaurus Link Component (for .mdx files)
If using MDX, Docusaurus provides a `Link` component that is locale-aware:
```jsx
import { Link } from '@docusaurus/router';
<Link to="/docs/chapter-2/">Chapter 2</Link>
```
This component automatically localizes the path. (Use if available; standard Markdown links also work.)

#### Fallback Behavior
- If a chapter is not yet translated to Urdu, a link to that chapter from a Urdu page:
  - Option A: Falls back to English version (current Docusaurus default)
  - Option B: Shows a "Translation pending" message and links to English version
  - **Selected**: Option A (simpler; user can understand from URL that it's English)

#### Testing Approach
1. Translate 2 sample chapters with mixed links (relative, absolute, external)
2. Build and load Urdu pages in browser
3. Click each link and verify it resolves to the correct page (Urdu if translated, English if not)
4. Verify no 404 errors
5. Document any link rewriting rules for translators

### Testing Checklist
```
[ ] Relative links resolve to Urdu pages (if translated) or English (fallback)
[ ] Absolute links either rewritten during translation or resolve to English fallback
[ ] External links unchanged and functional
[ ] Navigation between chapters works seamlessly
[ ] No 404 errors when clicking links
[ ] Breadcrumb navigation updates correctly when switching locales
```

---

## Task 6: Build Process & Static Output Verification

### Decision
Use standard Docusaurus build process (`npm run build`); verify that both English and Urdu static HTML outputs are generated correctly. Establish a build verification checklist to ensure i18n configuration is correct.

### Rationale
- Docusaurus build process is standard and well-tested
- Static HTML generation ensures no runtime translation; pages are pre-built and served
- Verification checklist provides confidence that i18n is properly configured before deployment

### Alternatives Considered
1. **Custom build script** - Rejected: Docusaurus build is sufficient; custom script adds complexity
2. **Incremental build** - Rejected: full build is clearer for verification; speed is acceptable
3. **Development server only** (skip full build verification) - Rejected: build output must be verified for production deployment

### Implementation Details

#### Standard Docusaurus Build Command
```bash
cd content/
npm run build
```

**Expected Output**:
```
build/
├── en/
│   ├── docs/
│   │   ├── intro/index.html
│   │   ├── chapter-1/index.html
│   │   └── ...
│   ├── index.html
│   └── ...
├── ur/
│   ├── docs/
│   │   ├── intro/index.html
│   │   ├── chapter-1/index.html
│   │   └── ...
│   ├── index.html
│   └── ...
└── ...
```

#### Build Verification Checklist

```
[ ] Build command completes without errors
[ ] No missing translation warnings (or expected warnings for untranslated chapters)
[ ] build/en/ directory contains English static HTML
[ ] build/ur/ directory contains Urdu static HTML
[ ] build/ur/docs/intro/index.html exists and contains Urdu content (verify by opening in browser)
[ ] build/ur/docs/chapter-1/index.html exists and contains Urdu content
[ ] All chapters listed in sidebars.js have corresponding HTML files
[ ] build size is reasonable (no duplicated or bloated assets)
[ ] build/en/ and build/ur/ are separate and don't contain mixed content
```

#### Static HTML Content Verification

Inspect `build/ur/docs/intro/index.html` (or any Urdu page):
```html
<!doctype html>
<html dir="rtl" lang="ur">
<head>
  <!-- Should contain Urdu metadata -->
  <meta charset="utf-8" />
  <title>اردو عنوان</title>
  <!-- ... -->
</head>
<body>
  <!-- Content should be present; verify it's NOT loading from JavaScript -->
  <h1>اردو متن</h1>
  <!-- ... -->
  <script>/* Page content should be in HTML, not generated by JavaScript */</script>
</body>
</html>
```

**Key Verification**: The Urdu content (headings, paragraphs, code blocks) should be **visible in the HTML source**, not generated by JavaScript. This confirms static generation is working.

#### Development Server Testing

```bash
npm run start
```

- Load http://localhost:3000 (English home)
- Load http://localhost:3000/ur/ (Urdu home)
- Click language switcher; verify it navigates between English and Urdu
- Load http://localhost:3000/ur/docs/chapter-1/ and verify Urdu content displays

#### Build Time Baseline

First build with i18n:
- Record build time (e.g., 120 seconds)
- Document as baseline for performance tracking
- Future builds should not significantly exceed baseline

#### Testing Approach (Integration)
1. Run `npm run build` after adding Urdu translations
2. Execute build verification checklist
3. Inspect sample HTML files to confirm static content
4. Run `npm run start` and test language switcher
5. Verify no console errors or warnings
6. Document build output and results for QA handoff

---

## Summary Table: Research Findings

| Task | Decision | Key Finding | Impact |
|------|----------|------------|--------|
| **Docusaurus i18n Config** | Use native v2 i18n config | `i18n` block in `docusaurus.config.js` with `locales: ['en', 'ur']` | Routes, language switcher automatic; no custom coding |
| **RTL Support** | Enable via i18n config | `direction: 'rtl'` in config + minimal CSS adjustments | Urdu text readable; sidebar may shift to right |
| **AI Translation Workflow** | Claude API + human review | 6-step workflow: extract → translate → review → refine → test → commit | Quality assured; consistent terminology |
| **Terminology Glossary** | JSON glossary with 20-30 core terms | Transliterated + English-with-context approach | 100% consistency across chapters; learner-friendly |
| **Link Resolution** | Docusaurus automatic (relative links) | Use relative links; absolute links fallback to English | No broken links; seamless navigation |
| **Build Verification** | Standard Docusaurus build + checklist | Static HTML in `build/ur/` with RTL metadata | Production-ready; no runtime translation |

---

## Unknowns Resolved

All 6 Phase 0 research tasks completed:
- ✅ Task 1: Docusaurus v2 i18n configuration approach confirmed
- ✅ Task 2: RTL support via i18n config (minimal CSS adjustments expected)
- ✅ Task 3: AI translation workflow with human review established
- ✅ Task 4: Terminology glossary structure defined (JSON, category-based)
- ✅ Task 5: Link resolution behavior documented (automatic for relative links)
- ✅ Task 6: Build verification checklist created

**Status**: Ready for Phase 1 design (data model, contracts) and Phase 2 implementation (chapter translation, build, test).

