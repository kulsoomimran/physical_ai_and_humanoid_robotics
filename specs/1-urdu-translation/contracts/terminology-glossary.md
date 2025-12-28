# Contract: Terminology Glossary

**Date**: 2025-12-28
**Feature**: Build Urdu i18n for Docusaurus Book
**Purpose**: Define the technical terminology glossary used consistently across all Urdu translations

---

## Overview

This contract specifies the structure, format, and initial entries for the Urdu terminology glossary. The glossary ensures consistent translation of technical terms across all chapters and serves as a reference for AI translators and human reviewers.

---

## Glossary Format

### Primary Format: JSON

The glossary is stored as `specs/1-urdu-translation/contracts/terminology-glossary.json`

```json
{
  "metadata": {
    "version": "1.0",
    "language": "Urdu",
    "date_created": "2025-12-28",
    "date_updated": "2025-12-28",
    "maintained_by": "Physical AI & Humanoid Robotics Team",
    "total_terms": 0
  },
  "terms": [
    {
      "id": "core-001",
      "term_en": "API",
      "term_ur": "اے پی آئی",
      "category": "core-concepts",
      "definition_ur": "Application Programming Interface - کمپیوٹر پروگراموں کے درمیان معلومات کا تبادلہ کرنے کا طریقہ",
      "usage_example": "REST API کا استعمال کریں تاکہ سرور سے ڈیٹا حاصل کریں۔",
      "approved": true,
      "approved_by": "Fatima Ali",
      "approved_at": "2025-12-20T12:00:00Z",
      "notes": "Transliterated; widely recognized in tech community.",
      "frequency": "high"
    }
  ]
}
```

### Secondary Format: Markdown (for reference)

A human-readable Markdown version is maintained at `specs/1-urdu-translation/contracts/terminology-glossary.md` (this file includes a table for easy reference).

---

## Glossary Structure

### Metadata Section

| Field | Type | Description |
|-------|------|-------------|
| `version` | String | Glossary version (semantic versioning: "1.0", "1.1", etc.) |
| `language` | String | Target language ("Urdu") |
| `date_created` | ISO 8601 | Initial glossary creation date |
| `date_updated` | ISO 8601 | Last update date |
| `maintained_by` | String | Team or person responsible for glossary |
| `total_terms` | Integer | Count of terms (updated as entries are added) |

### Term Entry Structure

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `id` | String | Yes | Unique identifier (format: `category-###`, e.g., "core-001", "ros-010") |
| `term_en` | String | Yes | English term (source language) |
| `term_ur` | String | Yes | Urdu transliteration or translation |
| `category` | Enum | Yes | Term category (see categories below) |
| `definition_ur` | String | No | Urdu definition (1-3 sentences) |
| `usage_example` | String | No | Example sentence using the term in Urdu context |
| `approved` | Boolean | Yes | Whether term is approved for use in translations |
| `approved_by` | String | No | Name of approver (required if `approved: true`) |
| `approved_at` | ISO 8601 | No | Approval timestamp (required if `approved: true`) |
| `notes` | String | No | Additional notes (transliteration strategy, context, etc.) |
| `frequency` | Enum | Yes | Usage frequency: `high` \| `medium` \| `low` |
| `alternate_forms` | Array[String] | No | Alternative Urdu spellings or translations |

### Term Categories

| Category | Description | Examples |
|----------|-------------|----------|
| `core-concepts` | Fundamental programming/robotics concepts | API, endpoint, protocol, algorithm |
| `programming-basics` | Basic programming constructs | variable, function, loop, condition, array |
| `data-structures` | Data organization patterns | list, dictionary, database, tree, graph |
| `oop` | Object-oriented programming | class, object, inheritance, polymorphism, method |
| `robotics` | Robotics-specific terms | robot, actuator, sensor, gripper, kinematics |
| `ros` | ROS 2 and ROS-specific terms | node, topic, service, publisher, subscriber |
| `simulation` | Simulation frameworks | Gazebo, Unity, Webots, physics simulation |
| `frameworks` | Development frameworks and tools | Docusaurus, NVIDIA Isaac, TensorFlow, PyTorch |
| `math-algorithms` | Mathematical and algorithmic concepts | matrix, neural network, optimization, gradient |
| `ai-ml` | AI and machine learning terms | machine learning, deep learning, model, training |
| `other` | Miscellaneous terms | N/A |

### Frequency Categories

- **`high`**: Used in >5 chapters or core concept (e.g., "variable", "function")
- **`medium`**: Used in 2-4 chapters or domain-specific (e.g., "ROS", "Gazebo")
- **`low`**: Used in 1 chapter or niche term (e.g., "Denavit-Hartenberg")

---

## Initial Glossary (Core Terms)

Below is the initial set of terminology entries for the Urdu translation project. This list will grow as chapters are translated and new terms are encountered.

### Core Concepts (core-0XX)

| ID | English | Urdu | Definition (Urdu) | Frequency |
|----|---------|------|-------------------|-----------|
| core-001 | API | اے پی آئی | Application Programming Interface - پروگراموں کے درمیان رابطہ | high |
| core-002 | endpoint | اینڈ پوائنٹ | API کے ذریعے رسائی کا مقام | high |
| core-003 | protocol | پروٹوکول | مواصلت کے اصول | high |
| core-004 | library | لائبریری | دوبارہ استعمال کے لیے کوڈ کا مجموعہ | high |
| core-005 | framework | فریم ورک | پروگرام لکھنے کا ڈھانچہ | high |

### Programming Basics (prog-0XX)

| ID | English | Urdu | Definition (Urdu) | Frequency |
|----|---------|------|-------------------|-----------|
| prog-001 | variable | متغیر | ڈیٹا کو رکھنے والا کنٹینر | high |
| prog-002 | function | فنکشن | دوبارہ استعمال کے لیے کوڈ | high |
| prog-003 | loop | لوپ | بار بار چلنے والا کوڈ | high |
| prog-004 | condition | شرط | اگر...تو کوڈ | high |
| prog-005 | array | سرنی | ترتیب سے ڈیٹا کا ذخیرہ | high |
| prog-006 | parameter | پیرامیٹر | فنکشن میں ڈیٹا داخل کرنا | medium |
| prog-007 | return value | بازگشت قدر | فنکشن سے نتیجہ | medium |

### Data Structures (data-0XX)

| ID | English | Urdu | Definition (Urdu) | Frequency |
|----|---------|------|-------------------|-----------|
| data-001 | database | ڈیٹا بیس | منظم ڈیٹا کو ریکارڈ کرنے کی جگہ | high |
| data-002 | table | ٹیبل | ڈیٹا بیس میں قطار اور ستون | high |
| data-003 | query | سوال | ڈیٹا بیس سے معلومات حاصل کرنا | medium |
| data-004 | index | انڈیکس | ڈیٹا بیس میں تیز رفتاری | medium |

### Object-Oriented Programming (oop-0XX)

| ID | English | Urdu | Definition (Urdu) | Frequency |
|----|---------|------|-------------------|-----------|
| oop-001 | class | کلاس | چیزوں کا نقشہ | high |
| oop-002 | object | آبجیکٹ | کلاس کی مثال | high |
| oop-003 | method | طریقہ | کلاس میں فنکشن | high |
| oop-004 | inheritance | وراثت | ایک کلاس دوسری سے خصوصیات لیتی ہے | medium |
| oop-005 | polymorphism | کثیر الاشکالیت | ایک نام کے متعدد معنی | low |

### Robotics (robot-0XX)

| ID | English | Urdu | Definition (Urdu) | Frequency |
|----|---------|------|-------------------|-----------|
| robot-001 | robot | روبوٹ | خود کار مشین | high |
| robot-002 | actuator | actuator | حرکت کرنے والا حصہ | high |
| robot-003 | sensor | سینسر | ماحول کو محسوس کرنا | high |
| robot-004 | gripper | gripper | چیزوں کو پکڑنے والا | medium |
| robot-005 | kinematics | Kinematics | حرکت کا حساب | low |
| robot-006 | dynamics | Dynamics | قوت اور حرکت | low |

### ROS 2 (ros-0XX)

| ID | English | Urdu | Definition (Urdu) | Frequency |
|----|---------|------|-------------------|-----------|
| ros-001 | ROS 2 | ROS 2 | روبوٹ آپریٹنگ سسٹم | high |
| ros-002 | node | نوڈ | ROS میں ایک پروگرام | high |
| ros-003 | topic | ٹاپک | پیغامات بھیجنے کا راستہ | high |
| ros-004 | service | سروس | درخواست اور جواب | high |
| ros-005 | publisher | ناشر | پیغام بھیجنے والا | high |
| ros-006 | subscriber | سبسکرائبر | پیغام حاصل کرنے والا | high |
| ros-007 | message | پیغام | ROS میں ڈیٹا کی شکل | medium |
| ros-008 | launch file | لانچ فائل | متعدد nodes شروع کرنے کی فائل | medium |

### Simulation Frameworks (sim-0XX)

| ID | English | Urdu | Definition (Urdu) | Frequency |
|----|---------|------|-------------------|-----------|
| sim-001 | Gazebo | Gazebo | روبوٹ کی نقل کا سافٹ ویئر | high |
| sim-002 | Unity | Unity | گیم بنانے کا پلیٹ فارم | medium |
| sim-003 | physics simulation | فزکس نقل | اصل دنیا کی نقل | medium |
| sim-004 | NVIDIA Isaac | NVIDIA Isaac | روبوٹکس کے لیے سافٹ ویئر | low |

### AI & Machine Learning (ai-0XX)

| ID | English | Urdu | Definition (Urdu) | Frequency |
|----|---------|------|-------------------|-----------|
| ai-001 | machine learning | مشین لرننگ | کمپیوٹر کو سیکھنا | high |
| ai-002 | deep learning | ڈیپ لرننگ | اعصابی نیٹ ورک سے سیکھنا | high |
| ai-003 | neural network | اعصابی نیٹ ورک | دماغ جیسی ڈھانچہ | high |
| ai-004 | model | ماڈل | سیکھا ہوا نتیجہ | high |
| ai-005 | training | تربیت | ڈیٹا سے سیکھنا | high |
| ai-006 | inference | اندازہ | سیکھے ہوئے ماڈل سے نتیجہ | medium |
| ai-007 | tensor | ٹینسر | ڈیٹا کی بہی خانہ شکل | low |
| ai-008 | gradient | میلان | تبدیلی کی سمت | low |

---

## Glossary Maintenance

### Adding New Terms

When a new term is encountered during translation:

1. **Propose entry** with:
   - English term
   - Suggested Urdu translation/transliteration
   - Category
   - Definition and usage example
   - Frequency estimate

2. **Review and approve** by Urdu-fluent technical reviewer

3. **Add to glossary** with approval metadata

4. **Update version** (e.g., 1.0 → 1.1)

### Update Workflow

```
Propose → Review → Approve → Add to glossary → Increment version
```

### Version Control

Glossary changes are tracked in Git:
- Each approved term addition increments glossary version
- Version comments document what was added (e.g., "Added 5 AI/ML terms for Chapter 4")

---

## Usage Guidelines for Translators

### Rule 1: Always Use Glossary Terms

If a term appears in the glossary, use the **exact** Urdu form from the glossary:

```
❌ Wrong: "API کو اے پی آئی کے طور پر استعمال کریں"
✅ Correct: "اے پی آئی کو استعمال کریں"
```

### Rule 2: Consistency Across Chapters

Same English term = same Urdu form throughout:

```
Chapter 1: "متغیر کو 10 سے سیٹ کریں"
Chapter 3: "متغیر کو 20 سے سیٹ کریں"
❌ NOT: "متغیر" in Chapter 1 and "تبدیل شے" in Chapter 3
```

### Rule 3: Code Identifiers Never Translated

Code variable/function names remain in English:

```
❌ Wrong: "ہمیں setVariable کے لیے setVariable_ur لکھنا ہے"
✅ Correct: "ہمیں setVariable میں متغیر سیٹ کرنی ہے"
```

### Rule 4: Context Matters

Use usage examples from glossary to understand correct context:

```
Glossary entry shows: "REST API کا استعمال کریں تاکہ سرور سے ڈیٹا حاصل کریں۔"
Chapter context: "ہمیں ڈیٹا حاصل کرنے کے لیے اے پی آئی استعمال کرنا ہے"
```

### Rule 5: Untranslated Terms

If a term is NOT in the glossary:

1. Check if it's a code identifier (keep English)
2. Propose a Urdu form with definition
3. Wait for approval before using
4. Document in chapter notes

---

## Glossary Versioning

### Version History Example

| Version | Date | Changes | Added By |
|---------|------|---------|----------|
| 1.0 | 2025-12-28 | Initial glossary: 40 core terms (core, prog, data, oop, robot, ros, sim, ai) | Fatima Ali |
| 1.1 | 2025-12-29 | Added 8 Vision & Language terms (vision-0XX) for Chapter 5 | Hassan Khan |
| 1.2 | 2026-01-05 | Added 5 Hardware terms (hw-0XX) for Gazebo simulation chapter | Amina Malik |

---

## Validation & Quality Checks

### Before Approval

- [ ] Urdu spelling and grammar verified
- [ ] Transliteration (if used) follows standard conventions
- [ ] Definition is clear and concise (1-3 sentences max)
- [ ] Usage example is realistic and relevant
- [ ] Category correctly assigned
- [ ] Frequency estimate justified

### After Addition

- [ ] Term added to JSON glossary
- [ ] Term added to Markdown reference
- [ ] Version incremented
- [ ] All translators notified of new term

---

## Example: Adding a New Term

### Proposal

```
New Term: "Vision-Language-Action" (VLA)

Proposed Urdu: "بصری زبان عمل" or keep as "VLA"
Category: ai-ml (or new category vision-0XX)
Definition: AI نظام جو تصاویر دیکھتا ہے، ہدایات سمجھتا ہے، اور عمل کرتا ہے
Usage: "VLA ماڈل کا استعمال کریں تاکہ روبوٹ کو تصریح دے سکیں"
Frequency: high (will be used in multiple chapters)
Suggested ID: vision-001
Notes: New for Advanced AI chapter; niche term but growing importance
```

### Approval

Reviewer validates and approves:

```json
{
  "id": "vision-001",
  "term_en": "Vision-Language-Action",
  "term_ur": "بصری زبان عمل",
  "category": "ai-ml",
  "definition_ur": "مصنوعی ذہانت کا نظام جو تصاویر دیکھتا ہے، ہدایات سمجھتا ہے، اور روبوٹ کو کام کرنے کا حکم دیتا ہے",
  "usage_example": "VLA ماڈل کا استعمال کریں تاکہ روبوٹ کو قدرتی زبان میں ہدایات دے سکیں",
  "approved": true,
  "approved_by": "Fatima Ali",
  "approved_at": "2025-12-30T14:30:00Z",
  "notes": "Niche term for Advanced AI chapter; future-oriented per constitution principle IV",
  "frequency": "high"
}
```

---

## Integration with Translation Workflow

### Translator Tool

Glossary is provided to AI translation prompt:

```
Use this glossary for consistent terminology:

[Generated JSON dump of all approved terms with definitions]

Example:
- API → اے پی آئی (used for technical interfaces)
- variable → متغیر (used for data storage)
- function → فنکشن (used for reusable code)
...

Ensure ALL repeated terms use the glossary forms.
```

### Reviewer Checklist

Reviewer verifies:

```
[ ] All technical terms from glossary used consistently
[ ] No ad-hoc translations of glossary terms
[ ] Code identifiers left in English
[ ] New terms encountered documented for next glossary version
[ ] Terminology quality aligned with Urdu standards
```

