# ---------- Build stage ----------
FROM node:18-alpine AS builder

WORKDIR /app

# Install dependencies
COPY package*.json ./
RUN npm install

# Copy source code
COPY . .

# Build the app
RUN npm run build


# ---------- Production stage ----------
FROM node:18-alpine

WORKDIR /app

# Install only production deps
COPY package*.json ./
RUN npm install --production

# Copy built app from builder
COPY --from=builder /app/public ./public

# Expose correct port
EXPOSE 5000

# IMPORTANT: bind to 0.0.0.0 so Docker can access it
CMD ["npx", "sirv", "public", "--host", "0.0.0.0", "--port", "5000"]