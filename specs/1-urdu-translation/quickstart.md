# Quickstart: Translate a Chapter to Urdu

**Date**: 2025-12-28
**Feature**: Build Urdu i18n for Docusaurus Book
**Purpose**: Step-by-step workflow for translating a single English chapter to Urdu

---

## Overview

This quickstart guide walks you through translating one chapter from English to Urdu using AI assistance (Claude) and human review. Once you've completed one chapter, the process scales to all chapters.

**Estimated time per chapter** (depending on length):
- Extraction: 5 minutes
- AI translation: 5-10 minutes (API call)
- Manual review: 20-30 minutes
- Refinement & approval: 10-15 minutes
- **Total: ~45-70 minutes per chapter**

---

## Prerequisites

Before starting, ensure:

- [ ] Docusaurus v2 is installed in `content/` directory
- [ ] Node.js and npm are available (`node --version`, `npm --version`)
- [ ] `docusaurus.config.js` has i18n configuration for Urdu (see [docusaurus-config.md](contracts/docusaurus-config.md))
- [ ] Directory structure exists:
  - `content/docs/` (English chapters)
  - `content/i18n/ur/docusaurus-docs/` (for Urdu translations)
  - `content/i18n/ur/code.json` (UI strings)
- [ ] Terminology glossary is available at `specs/1-urdu-translation/contracts/terminology-glossary.json`
- [ ] Access to Claude API or similar AI translation service
- [ ] A Urdu-fluent technical reviewer (human) for quality assurance
- [ ] Git repository set up with branch `1-urdu-translation` checked out

---

## Step 1: Extract the English Chapter

### 1.1 Choose a chapter to translate

Start with a shorter chapter (500-1000 words) for your first translation. Example: "Introduction" or "Getting Started".

### 1.2 Inspect the source file

```bash
cd content/
cat docs/intro.md
```

### 1.3 Note the structure

Identify:
- Chapter title (H1 heading)
- Sections (H2, H3 headings)
- Code blocks (triple backticks)
- Tables (if any)
- Links (internal and external)
- Lists (bullet points, numbered)

**Example structure**:

```markdown
# Introduction to ROS 2

This chapter introduces ROS 2 basics.

## What is ROS 2?

ROS 2 is a middleware framework...

## Installation

Follow these steps:

1. Install dependencies
2. Clone the repository
3. Build the package

### Example code

```cpp
#include <rclcpp/rclcpp.hpp>

int main() {
  auto node = std::make_shared<rclcpp::Node>("hello_world");
  return 0;
}
```

[Read more](https://docs.ros2.org)
```

### 1.4 Save chapter metadata

Create a simple metadata file for tracking:

```json
{
  "chapter_id": "intro",
  "title_en": "Introduction to ROS 2",
  "path_en": "docs/intro.md",
  "word_count_en": 850,
  "has_code_blocks": true,
  "has_tables": false,
  "has_links": true,
  "status": "extracted"
}
```

---

## Step 2: Prepare for AI Translation

### 2.1 Load the terminology glossary

Read the approved glossary:

```bash
cat specs/1-urdu-translation/contracts/terminology-glossary.json
```

Or view the Markdown version:

```bash
cat specs/1-urdu-translation/contracts/terminology-glossary.md
```

### 2.2 Create a translation prompt

Use the following Claude API prompt template:

```
# Urdu Translation Task: [CHAPTER_TITLE]

You are an expert Urdu translator specializing in technical and educational content.

## Your Task

Translate the following English chapter to natural, fluent Urdu.

### CRITICAL CONSTRAINTS

1. **Preserve Markdown Structure**: All headings (# ## ###), bold (**), italics (*), inline code (`...`), code blocks (```...```), and links ([...](...)​) must remain exactly as in the original.

2. **Preserve Code Identifiers**: Never translate code variable names, function names, class names, or code comments. These remain in English. Only translate explanatory text around code.

3. **Preserve Links**: All links (internal and external) must remain unchanged. Do NOT translate link text for internal documentation links.

4. **Use Glossary Terms**: Apply the terminology glossary strictly. For each English term in the list below, use ONLY the Urdu form provided.

5. **Natural Urdu**: Translation must be idiomatic Urdu suitable for self-learners. Not word-for-word; prioritize readability and clarity.

6. **No English Except Code**: All explanatory text, headings, and descriptions should be in Urdu. Only code and code identifiers remain in English.

### TERMINOLOGY GLOSSARY

[INSERT RELEVANT TERMS FROM GLOSSARY HERE]

Example entries (use these consistently):
- "API" → "اے پی آئی"
- "variable" → "متغیر"
- "function" → "فنکشن"
- "ROS 2" → "ROS 2" (keep as-is)
- (... more terms ...)

### CHAPTER CONTENT TO TRANSLATE

[INSERT ENGLISH CHAPTER HERE]

### OUTPUT REQUIREMENTS

Return ONLY the translated Markdown. No explanations, no notes. The output must be:
- Valid Markdown (no syntax errors)
- Same structure as input
- Same length approximately (Urdu may be slightly longer/shorter)
- All formatting preserved
- All links intact
- Code blocks unchanged (except comments may be translated)

---

**DO NOT output anything except the translated Markdown. Begin translation now.**
```

### 2.3 Prepare the API call

If using Claude API via Python or JavaScript:

```python
import anthropic

client = anthropic.Anthropic(api_key="YOUR_API_KEY")

with open("content/docs/intro.md", "r", encoding="utf-8") as f:
    english_chapter = f.read()

with open("specs/1-urdu-translation/contracts/terminology-glossary.json", "r", encoding="utf-8") as f:
    import json
    glossary = json.load(f)

# Build glossary string for prompt
glossary_entries = "\n".join([
    f"- \"{term['term_en']}\" → \"{term['term_ur']}\" ({term.get('definition_ur', '')})"
    for term in glossary["terms"]
    if term.get("approved", False)
])

prompt = f"""[PROMPT TEMPLATE ABOVE]

TERMINOLOGY GLOSSARY:
{glossary_entries}

CHAPTER CONTENT:
{english_chapter}

OUTPUT: Provide only the translated Markdown below."""

response = client.messages.create(
    model="claude-3-opus-20250101",
    max_tokens=4000,
    messages=[{"role": "user", "content": prompt}]
)

translated_content = response.content[0].text

# Save translation
with open("content/i18n/ur/docusaurus-docs/intro.md", "w", encoding="utf-8") as f:
    f.write(translated_content)

print("Translation saved to content/i18n/ur/docusaurus-docs/intro.md")
```

---

## Step 3: AI Translation

### 3.1 Call Claude API

Run the script above (adjust model and parameters as needed).

### 3.2 Verify output

Check that the output file exists and contains Urdu content:

```bash
cat content/i18n/ur/docusaurus-docs/intro.md
```

Look for:
- Urdu script (U+0600-U+06FF range)
- Markdown structure preserved (# ## ### headings, code blocks, links)
- No broken formatting

### 3.3 Record translation metadata

Update metadata file:

```json
{
  "chapter_id": "intro",
  "status": "translated",
  "ai_model": "claude-3-opus-20250101",
  "translated_at": "2025-12-28T10:30:00Z",
  "word_count_ur": 890,
  "glossary_version": "1.0",
  "review_status": "pending-review"
}
```

---

## Step 4: Manual Review

### 4.1 Read the translation

Open the translated file in an editor with UTF-8 support:

```bash
cat content/i18n/ur/docusaurus-docs/intro.md | less
```

### 4.2 Review checklist

Go through each item:

#### Formatting Checklist

- [ ] **Headings preserved**: All # ## ### headings are present and formatted correctly
- [ ] **Code blocks intact**: All \`\`\` ... \`\`\` code blocks are present and readable
- [ ] **Code identifiers in English**: Variable names, function names, class names remain in English (e.g., `setVariable`, `class Robot`, `function init()`)
- [ ] **Links preserved**: All links `[text](url)` are intact; internal links point to correct pages
- [ ] **Lists preserved**: Bullet lists and numbered lists have correct structure
- [ ] **Tables preserved** (if any): Table rows and columns intact
- [ ] **Bold/italics preserved**: **bold** and *italic* formatting intact

#### Language Quality Checklist

- [ ] **Urdu script**: All text uses Urdu script (Nastaliq or Naskh); no Latin characters except in code
- [ ] **Natural Urdu**: Translation reads naturally; not literal word-for-word
- [ ] **Terminology consistent**: All repeated terms use glossary forms (e.g., "اے پی آئی" for "API" every time)
- [ ] **No English except code**: Explanatory text is fully in Urdu; only code remains English
- [ ] **Clarity**: Sentences are clear and understandable for Urdu-speaking learners
- [ ] **Context preservation**: Code examples and explanations match the original meaning

#### Technical Accuracy Checklist

- [ ] **Code examples unchanged**: If code blocks have explanatory comments, those can be translated, but code logic is unchanged
- [ ] **Links functional**: Internal links would navigate to correct Urdu pages; external links are unchanged
- [ ] **Technical terms accurate**: Terms from glossary are applied correctly; no mistranslations of programming concepts

### 4.3 Document issues

If you find issues, list them:

```
Issues Found:
1. Heading "چیزیں سیکھیں" should use "سیکھنے کی چیزیں" (correct grammar)
2. Link [سرور](../server.md) should be [سرور](/ur/docs/server/) (locale-aware)
3. Code block comment: "// یہ کمنٹ ہے" - check grammar
4. Term "ڈیٹا بیس" appears sometimes as "ڈیٹا بیس" and sometimes as "ڈیٹابیس" (inconsistent)
```

### 4.4 Request revisions (if needed)

If significant issues found:

```
Translation has issues that need revision:

1. Terminology inconsistency (see issues above)
2. Grammatical errors in 3 sentences
3. One link needs locale adjustment

Re-translate with corrections, or wait for manual refinement in step 5.
```

---

## Step 5: Refinement & Approval

### 5.1 Manual fixes

Make small corrections directly in the translated file:

```bash
# Edit the file
nano content/i18n/ur/docusaurus-docs/intro.md
```

Fix:
- Typos in Urdu
- Formatting inconsistencies
- Terminology corrections
- Link URLs (if needed)

### 5.2 Re-verify formatting

After manual edits, verify the file still has valid Markdown:

```bash
# Check for Markdown syntax errors
npm run build --dry-run
# or just build to verify
```

### 5.3 Final review

Have another Urdu speaker review if possible. Use a checklist:

```
Final Review Checklist:

[ ] I have read the entire translation
[ ] I understand the technical content
[ ] The Urdu language quality is good
[ ] All formatting is correct
[ ] I would be comfortable learning from this content in Urdu

Reviewer signature: ________________  Date: ___________
```

### 5.4 Mark as approved

Update metadata:

```json
{
  "chapter_id": "intro",
  "status": "approved",
  "review_completed_at": "2025-12-28T14:15:00Z",
  "reviewer_name": "Fatima Ali",
  "review_comments": "Excellent translation. Minor terminology correction made (ڈیٹا بیس consistency). Ready for publication.",
  "issues_resolved": true
}
```

---

## Step 6: Build & Test

### 6.1 Run Docusaurus build

```bash
cd content/
npm run build
```

Expected output:
```
✔️ Successfully compiled after 120s.
✔️ Generated static files in build directory.
✔️ Created both en/ and ur/ locale outputs.
```

### 6.2 Verify Urdu output

Check that the Urdu page was generated:

```bash
ls -la build/ur/docs/intro/
# Expected: index.html exists
```

### 6.3 Inspect HTML

View the generated HTML to verify Urdu content is present:

```bash
head -50 build/ur/docs/intro/index.html
```

Look for:
- `<html dir="rtl" lang="ur">` (RTL and Urdu lang)
- Urdu title in `<title>` tag
- Urdu content in `<h1>`, `<p>` tags
- No JavaScript-based loading (content should be in HTML)

### 6.4 Test locally (optional)

Start the dev server and test in browser:

```bash
cd content/
npm run start

# Visit http://localhost:3000/ur/docs/intro/
# Verify page displays correctly in Urdu
# Check language switcher (toggle between English and Urdu)
```

---

## Step 7: Commit & Document

### 7.1 Stage files

```bash
cd /path/to/repo
git add content/i18n/ur/docusaurus-docs/intro.md
git add specs/1-urdu-translation/contracts/terminology-glossary.json  # if updated
```

### 7.2 Commit

```bash
git commit -m "Translate intro chapter to Urdu

- Translated 'Introduction to ROS 2' chapter to Urdu
- Applied terminology glossary (40 core terms)
- Preserved all code blocks, links, and formatting
- Reviewed and approved by Fatima Ali
- Status: Ready for production

Related files:
- content/i18n/ur/docusaurus-docs/intro.md
- specs/1-urdu-translation/contracts/terminology-glossary.json
"
```

### 7.3 Log completion

Update tracking sheet or progress file:

```markdown
# Translation Progress

| Chapter | Status | Translator | Reviewer | Translated | Reviewed |
|---------|--------|-----------|----------|-----------|----------|
| intro | ✅ Published | Claude API | Fatima Ali | 2025-12-28 | 2025-12-28 |
| chapter-1 | ⏳ Translating | Claude API | - | - | - |
| chapter-2 | pending | - | - | - | - |
```

---

## Troubleshooting

### Issue: AI Translation Cuts Off or Returns Incomplete Output

**Cause**: Token limit or API timeout
**Solution**:
- Split long chapters into sections
- Increase `max_tokens` in API call
- Retry with adjusted prompt

### Issue: Urdu Text Shows as Garbled (??????)

**Cause**: Encoding issue or font problem
**Solution**:
- Ensure file is saved as UTF-8 (not ASCII)
- Check terminal encoding: `locale` or set `export LANG=en_US.UTF-8`
- Verify editor supports UTF-8 (VS Code, Sublime, etc.)

### Issue: Build Fails with "Invalid Markdown"

**Cause**: Translation broke Markdown syntax
**Solution**:
- Check for unmatched backticks in code blocks
- Verify links are in format `[text](url)`
- Check for broken table syntax (pipes `|` aligned)
- Run `npm run build` with `--debug` flag for details

### Issue: Links Point to English (Not Urdu)

**Cause**: Absolute links not rewritten for locale
**Solution**:
- Use relative links in Markdown: `[link](../chapter-1/)`
- If using absolute links, manually rewrite to `/ur/docs/...`
- Test language switcher; should auto-navigate to Urdu version

### Issue: Terminology Inconsistency

**Cause**: AI didn't apply glossary consistently
**Solution**:
- Re-translate with more prominent glossary placement in prompt
- Manually search & replace inconsistent terms
- Add to reviewer checklist to catch before approval

---

## Tips for Success

1. **Start small**: Translate your first chapter carefully; use it to refine your process
2. **Glossary first**: Build and approve glossary before translating many chapters
3. **Review quality matters**: Allocate time for thorough human review; this ensures reader satisfaction
4. **Iterative refinement**: First translation won't be perfect; plan for 1-2 review cycles
5. **Consistency tracking**: Log which glossary version was used for each chapter
6. **Community**: Share translated chapters with Urdu-speaking test readers for feedback

---

## Next Steps

After successfully translating one chapter:

1. **Translate remaining chapters** using the same workflow (Steps 1-7)
2. **Expand glossary** as new terms are encountered
3. **Run full build** with all Urdu translations
4. **Final QA**: Browser testing of all pages
5. **Deployment**: Push to production

---

## Resources

- [Docusaurus i18n Configuration](contracts/docusaurus-config.md)
- [Terminology Glossary](contracts/terminology-glossary.md)
- [Data Model & Entities](data-model.md)
- [Feature Specification](spec.md)
- [Implementation Plan](plan.md)

