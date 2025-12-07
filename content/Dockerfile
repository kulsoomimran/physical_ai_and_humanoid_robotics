# Use Node 18 as the base image
FROM node:18-alpine

# Set working directory
WORKDIR /app

# Copy package files
COPY package*.json ./

# Install dependencies
RUN npm install

# Copy the rest of the application
COPY . .

# Build the Docusaurus site
RUN npm run build

# Expose port
EXPOSE 3000

# Serve the built site
CMD ["npm", "run", "serve"]