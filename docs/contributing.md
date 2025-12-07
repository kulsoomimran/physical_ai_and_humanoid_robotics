# Contributing Guide

## Getting Started

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/amazing-feature`
3. Make your changes
4. Test thoroughly
5. Commit with conventional commits: `git commit -m "feat: add amazing feature"`
6. Push to your fork: `git push origin feature/amazing-feature`
7. Open a pull request

## Development Workflow

### Backend Development
- Follow FastAPI best practices
- Write type hints for all functions
- Use Pydantic models for request/response validation
- Add unit tests for new functionality
- Follow PEP 8 style guidelines

### Frontend Development
- Use React best practices
- Follow Docusaurus theming conventions
- Write component documentation
- Test responsiveness
- Follow accessibility guidelines

## Code Standards

### Python
- Follow PEP 8
- Use type hints
- Write docstrings for all functions
- Use meaningful variable names
- Keep functions small and focused

### JavaScript/React
- Use functional components with hooks
- Follow React best practices
- Use TypeScript for new components
- Write accessible components
- Use CSS modules or styled-components

## Testing

### Backend Tests
- Unit tests for service layer functions
- Integration tests for API endpoints
- Contract tests for external APIs

### Frontend Tests
- Unit tests for React components
- Integration tests for API interactions
- End-to-end tests for critical user flows

## Documentation

- Update API documentation when adding endpoints
- Add user guides for new features
- Update architecture documentation for major changes
- Write clear commit messages

## Pull Request Guidelines

- Keep PRs small and focused
- Include tests for new functionality
- Update documentation as needed
- Link to related issues
- Get code review before merging

## Issue Tracking

- Use appropriate labels
- Write clear, actionable descriptions
- Include reproduction steps for bugs
- Estimate complexity when possible