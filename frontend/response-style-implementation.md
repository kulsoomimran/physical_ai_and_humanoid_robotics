# Frontend Implementation Guide: Response Style Selection

## Overview
This document outlines the frontend implementation required to support User Story 3: Interactive Learning with Contextual Explanations.

## Required Frontend Features

### 1. Response Style Selector
- UI controls to allow users to select their preferred response style:
  - `simplified`: Simple, easy-to-understand explanations
  - `detailed`: Comprehensive, in-depth explanations with technical details
  - `examples`: Practical examples that illustrate concepts
  - `standard`: Default response style

### 2. API Integration
The frontend needs to integrate with the following backend endpoints:

#### For Queries:
- **POST** `/chat/{session_token}/query`
  - Include `response_style` parameter in request
  - Example: `{"session_token": "token", "query": "question", "response_style": "detailed"}`

### 3. UI Components to Implement

#### Response Style Selector Component
```jsx
// Example implementation approach
const ResponseStyleSelector = ({ onStyleChange }) => {
  const [selectedStyle, setSelectedStyle] = useState('standard');

  const styles = [
    { value: 'standard', label: 'Standard', description: 'Balanced explanation' },
    { value: 'simplified', label: 'Simple', description: 'Easy to understand' },
    { value: 'detailed', label: 'Detailed', description: 'In-depth technical info' },
    { value: 'examples', label: 'Examples', description: 'Practical illustrations' }
  ];

  return (
    <div className="response-style-selector">
      <label>Select Response Style:</label>
      <select
        value={selectedStyle}
        onChange={(e) => {
          setSelectedStyle(e.target.value);
          onStyleChange(e.target.value);
        }}
      >
        {styles.map(style => (
          <option key={style.value} value={style.value}>
            {style.label} - {style.description}
          </option>
        ))}
      </select>
    </div>
  );
};
```

#### Integration with Chat Interface
- Store user's preferred style as default
- Allow per-query style selection
- Visual indication of current style
- Style persistence across sessions

## Implementation Notes

1. **User Experience**: Provide clear descriptions of each response style
2. **Accessibility**: Ensure all components are accessible
3. **Performance**: Style selection should not impact response time significantly
4. **Feedback**: Consider showing an example of each style type

## Integration Points

- Connect to the backend RAG API with response_style parameter
- Maintain user preferences across sessions
- Handle API responses appropriately based on selected style
- Update UI to reflect selected style

## Example Usage Flow

1. User selects a response style (e.g., "Examples")
2. User submits a query about a complex robotics concept
3. Backend processes the query with the selected style preference
4. Response includes practical examples from the book content
5. UI displays the response with appropriate formatting for examples

## Additional Considerations

- Allow users to switch styles mid-conversation
- Consider adaptive responses based on user interaction patterns
- Implement tooltips explaining each style option
- Provide quick-select buttons for common styles