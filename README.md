# Physical AI & Humanoid Robotics Book

A documentation website for the Physical AI & Humanoid Robotics book content.

## Features

- Interactive learning platform
- Book content delivery
- User-friendly interface

## Prerequisites

- Node.js (for running the documentation site locally)

## Running the Documentation Site

1. Navigate to the content directory:
   ```bash
   cd content
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Start the development server:
   ```bash
   npm start
   ```

4. The site will be available at `http://localhost:3000`

## Architecture

- **Docusaurus**: Static site generator for documentation
- **React**: Frontend framework

## Deployment

### Using Docker Compose

1. Ensure you have Docker and Docker Compose installed
2. Run the application:
   ```bash
   docker-compose up --build
   ```

### Manual Deployment

1. Navigate to the content directory:
   ```bash
   cd content
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Build the static site:
   ```bash
   npm run build
   ```

4. Serve the built site:
   ```bash
   npm run serve
   ```

## License

[Specify your license here]