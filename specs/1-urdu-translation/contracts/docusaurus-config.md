# Contract: Docusaurus i18n Configuration

**Date**: 2025-12-28
**Feature**: Build Urdu i18n for Docusaurus Book
**Purpose**: Define the required Docusaurus configuration for Urdu language support

---

## Overview

This contract specifies the `i18n` configuration block required in `content/docusaurus.config.js` to enable Urdu (ur) as a secondary locale alongside English (en).

---

## Configuration Contract

### Input/Output

**Input**:
- Current `docusaurus.config.js` (with or without i18n block)
- Desired locales: English (en) and Urdu (ur)
- RTL support requirement: Yes (for Urdu script)

**Output**:
- Updated `docusaurus.config.js` with `i18n` block configured
- Locale-specific directories created: `i18n/ur/`
- Language switcher automatically available in navbar

---

## Required Configuration Block

### JavaScript Configuration

```javascript
// In content/docusaurus.config.js

module.exports = {
  // ... other config ...

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

  // ... rest of config ...
};
```

### Configuration Fields Explained

| Field | Value | Description |
|-------|-------|-------------|
| `defaultLocale` | `'en'` | Default language when user first visits (English) |
| `locales` | `['en', 'ur']` | Supported languages; order determines navbar priority |
| `en.label` | `'English'` | Display label for English in language switcher |
| `en.direction` | `'ltr'` | Left-to-right for English; applied as HTML `dir` attribute |
| `en.htmlLang` | `'en'` | HTML `lang` attribute value for English pages |
| `ur.label` | `'اردو'` | Display label for Urdu in language switcher (in Urdu script) |
| `ur.direction` | `'rtl'` | Right-to-left for Urdu; applied as HTML `dir` attribute |
| `ur.htmlLang` | `'ur'` | HTML `lang` attribute value for Urdu pages (or `'ur-PK'` for Pakistan-specific) |

---

## Directory Structure Requirements

After adding the i18n configuration, the following directory structure is required:

```
content/
├── docusaurus.config.js           # Updated with i18n block (this contract)
├── docs/                          # English chapters (existing)
│   ├── intro.md
│   ├── chapter-1.md
│   └── ... (all English chapters)
│
├── i18n/                          # NEW: Create this directory
│   └── ur/                        # Urdu locale (created during initial setup)
│       ├── code.json              # UI strings in Urdu (see contract below)
│       └── docusaurus-docs/       # Translated chapters (mirrors docs/ structure)
│           ├── intro.md
│           ├── chapter-1.md
│           └── ... (parallel to docs/)
│
├── sidebars.js                    # May need Urdu labels (see below)
└── ... (other Docusaurus files)
```

### Initial Setup

Docusaurus provides a command to initialize i18n:

```bash
cd content/
npx docusaurus write-translations --locale ur
```

This command creates:
- `i18n/ur/` directory
- `i18n/ur/code.json` with UI strings (to be translated to Urdu)
- `i18n/ur/docusaurus-docs/` structure

---

## UI Strings Configuration (`code.json`)

After running `write-translations`, update `content/i18n/ur/code.json` with Urdu translations for navbar, sidebar, and common labels:

```json
{
  "theme.navbar.label.Documentation": "دستاویزات",
  "theme.navbar.mobileLanguageSelector.label": "زبانیں",
  "theme.navbar.mobileLanguageSelector.ariaLabel": "متبادل موجودہ زبان {language}",
  "theme.common.editThisPage": "اس صفحہ میں ترمیم کریں",
  "theme.common.headingsMenu": "عنوانات",
  "theme.docs.sidebar.navAriaLabel": "سائڈ بار نیویگیشن",
  "theme.docs.paginator.previous": "پچھلا",
  "theme.docs.paginator.next": "اگلا",
  "theme.docs.breadcrumbs.home": "گھر",
  "theme.last_update.lastUpdatedBy": "آخری تبدیلی اثر: {author}",
  "theme.last_update.lastUpdatedAt": "آخری اپڈیٹ: {date}",
  "theme.searchBar.sidebarTitle": "تلاش",
  "theme.searchBar.placeholder": "تلاش کریں",
  "blog.sidebar.title": "تمام پوسٹس",
  "blog.sidebar.navAriaLabel": "حالیہ بلاگ پوسٹس",
  "docusaurus.meta.description": "فزیکل AI اور Humanoid Robotics پر تفصیلی رہنمائی۔ Urdu میں دستاویزات۔"
}
```

---

## Sidebar Navigation Configuration

If `sidebars.js` has hardcoded labels, it may need Urdu translations. Example:

### English Sidebar (existing)

```javascript
// sidebars.js
const sidebars = {
  docs: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Introduction',
    },
    {
      type: 'category',
      label: 'ROS 2 Fundamentals',
      items: [
        'ros2-intro',
        'ros2-nodes',
        'ros2-topics',
      ],
    },
    // ... more items
  ],
};

module.exports = sidebars;
```

### Urdu Sidebar (in `code.json`)

Instead of hardcoding, use localization keys in `code.json`:

```json
{
  "sidebar.docs.intro": "تعارف",
  "sidebar.docs.ros2_fundamentals": "ROS 2 بنیادی باتیں",
  "sidebar.docs.ros2_intro": "ROS 2 کا تعارف",
  "sidebar.docs.ros2_nodes": "Nodes",
  "sidebar.docs.ros2_topics": "Topics"
}
```

Then in `sidebars.js`, use `i18n` keys (if Docusaurus supports it) or rely on Docusaurus auto-detection of `code.json` entries.

**Note**: For simplicity, hardcoded English labels in `sidebars.js` are acceptable; full navbar localization can be handled via `code.json`.

---

## Build Behavior

When the i18n configuration is in place, Docusaurus build process:

1. **Compiles English site** → `build/en/` (routes: `/docs/...`)
2. **Compiles Urdu site** → `build/ur/` (routes: `/ur/docs/...`)
3. **Injects language switcher** in navbar (automatically)
4. **Sets HTML attributes** on each page:
   - English pages: `<html dir="ltr" lang="en">`
   - Urdu pages: `<html dir="rtl" lang="ur">`

---

## Verification Checklist

After updating `docusaurus.config.js`:

```
[ ] i18n block added to docusaurus.config.js
[ ] defaultLocale set to 'en'
[ ] locales array includes both 'en' and 'ur'
[ ] English config has direction: 'ltr' and htmlLang: 'en'
[ ] Urdu config has direction: 'rtl' and htmlLang: 'ur'
[ ] Urdu label set to 'اردو' (in Urdu script)
[ ] i18n/ur/ directory exists
[ ] i18n/ur/code.json exists and contains UI translations
[ ] i18n/ur/docusaurus-docs/ directory exists
[ ] npm run build completes without errors
[ ] build/en/ directory contains English static output
[ ] build/ur/ directory contains Urdu static output
[ ] npm run start (dev server) shows language switcher in navbar
[ ] Clicking language switcher toggles between English and Urdu
[ ] /docs/ route loads English content
[ ] /ur/docs/ route loads Urdu content
[ ] /ur/docs/intro/ loads Urdu version of intro page
```

---

## RTL CSS Handling

Docusaurus automatically applies RTL CSS when `direction: 'rtl'` is set. Expected CSS changes:

### Automatic (Docusaurus handles)
- Text alignment (right for RTL, left for LTR)
- Flex direction (reversed for RTL)
- Sidebar positioning (right for RTL, left for LTR)
- Padding/margin direction (reversed for RTL)

### Possible Manual Adjustments (if needed)
If custom CSS exists, ensure it uses CSS logical properties:

```css
/* Instead of */
padding-left: 1rem;

/* Use */
padding-inline-start: 1rem;  /* Auto-reverses for RTL */
```

---

## Language Switcher Behavior

After i18n is configured, the navbar includes a language switcher dropdown:

**English page** (`/docs/chapter-1/`):
- Dropdown shows: English (current), اردو
- Clicking اردو → navigates to `/ur/docs/chapter-1/`

**Urdu page** (`/ur/docs/chapter-1/`):
- Dropdown shows: English, اردو (current)
- Clicking English → navigates to `/docs/chapter-1/`

**Homepage**:
- English: `/` (or `/docs/`)
- Urdu: `/ur/` (or `/ur/docs/`)

---

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| Build fails with "i18n error" | Malformed i18n config syntax | Validate JSON syntax in docusaurus.config.js |
| `/ur/` route returns 404 | i18n config missing or incorrect | Verify i18n block is present and locales includes 'ur' |
| Urdu pages load with English content | Translations missing or wrong path | Verify `i18n/ur/docusaurus-docs/` exists and contains .md files |
| Language switcher not visible | Theme doesn't support i18n switcher | Ensure docusaurus-theme-classic is installed and i18n config is present |
| RTL layout broken | CSS not handling RTL properly | Check for hardcoded directions in custom CSS; use logical properties |
| Urdu text displays as garbled | Font issue or character encoding | Ensure HTML meta charset is UTF-8; install Urdu fonts on system |

---

## Notes

- This configuration is **non-breaking**: Adding i18n does not affect existing English content or routes
- **Build time** may increase slightly due to dual-locale compilation (acceptable overhead)
- **SEO**: Both English and Urdu pages are static HTML and SEO-indexable
- **Future languages**: To add another language (e.g., Spanish), add to `locales` array and create `i18n/es/` directory

