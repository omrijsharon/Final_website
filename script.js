const horizontalLineLeft = document.getElementById('horizontalLineLeft');
const horizontalLineRight = document.getElementById('horizontalLineRight');
const verticalLineTop = document.getElementById('verticalLineTop');
const verticalLineBottom = document.getElementById('verticalLineBottom');
const cursorSquare = document.getElementById('cursorSquare');
const mouseSquare = document.getElementById('mouseSquare');
const mouseSquareInner = document.getElementById('mouseSquareInner');

// ====== CURSOR CONFIGURATION ======
// Alpha parameter for exponential moving average (0 to 1)
// Higher alpha = faster tracking, Lower alpha = smoother/slower tracking
const ALPHA = 0.1;
// Maximum distance for color interpolation (pixels)
const MAX_DISTANCE = 200;
// Square dimensions
const SQUARE_SIZE = 51;
const HALF_SQUARE = SQUARE_SIZE / 2;

// Current smoothed position
let smoothX = window.innerWidth / 2;
let smoothY = window.innerHeight / 2;

// Target position (actual mouse position)
let targetX = smoothX;
let targetY = smoothY;

// Navigation hover state
let isHoveringNav = false;
let targetNavWidth = 0;
let targetNavHeight = 0;
let currentNavWidth = 0;
let currentNavHeight = 0;
let targetNavX = 0;
let targetNavY = 0;
let smoothNavX = 0;
let smoothNavY = 0;
// ===========================

// ====== BOID SIMULATION BACKGROUND ======
const canvas = document.getElementById('matrixCanvas');
const ctx = canvas.getContext('2d');

canvas.width = window.innerWidth;
canvas.height = window.innerHeight;

// Boid configuration
const NUM_BOIDS = 85;
const BOID_SIZE = 2.5;
const MAX_SPEED = 6;
const MAX_FORCE = 0.35;

// Boid behavior weights
const SEPARATION_WEIGHT = 5.2;
const ALIGNMENT_WEIGHT = 3.0;
const COHESION_WEIGHT = 4.0;
const ATTRACTION_WEIGHT = 1.0;

// Perception radii
const SEPARATION_RADIUS = 80;
const ALIGNMENT_RADIUS = 50;
const COHESION_RADIUS = 120;
const ATTRACTION_RADIUS = 300;

class Boid {
    constructor() {
        this.position = {
            x: Math.random() * canvas.width,
            y: Math.random() * canvas.height
        };
        this.velocity = {
            x: (Math.random() - 0.5) * MAX_SPEED,
            y: (Math.random() - 0.5) * MAX_SPEED
        };
        this.acceleration = { x: 0, y: 0 };
    }

    edges() {
        // Wrap around screen edges
        if (this.position.x > canvas.width) this.position.x = 0;
        if (this.position.x < 0) this.position.x = canvas.width;
        if (this.position.y > canvas.height) this.position.y = 0;
        if (this.position.y < 0) this.position.y = canvas.height;
    }

    align(boids) {
        let steering = { x: 0, y: 0 };
        let total = 0;
        for (let other of boids) {
            let d = this.distance(other);
            if (other !== this && d < ALIGNMENT_RADIUS) {
                steering.x += other.velocity.x;
                steering.y += other.velocity.y;
                total++;
            }
        }
        if (total > 0) {
            steering.x /= total;
            steering.y /= total;
            steering = this.setMag(steering, MAX_SPEED);
            steering.x -= this.velocity.x;
            steering.y -= this.velocity.y;
            steering = this.limit(steering, MAX_FORCE);
        }
        return steering;
    }

    cohesion(boids) {
        let steering = { x: 0, y: 0 };
        let total = 0;
        for (let other of boids) {
            let d = this.distance(other);
            if (other !== this && d < COHESION_RADIUS) {
                steering.x += other.position.x;
                steering.y += other.position.y;
                total++;
            }
        }
        if (total > 0) {
            steering.x /= total;
            steering.y /= total;
            steering.x -= this.position.x;
            steering.y -= this.position.y;
            steering = this.setMag(steering, MAX_SPEED);
            steering.x -= this.velocity.x;
            steering.y -= this.velocity.y;
            steering = this.limit(steering, MAX_FORCE);
        }
        return steering;
    }    separation(boids) {
        let steering = { x: 0, y: 0 };
        let total = 0;
        for (let other of boids) {
            let d = this.distance(other);
            if (other !== this && d < SEPARATION_RADIUS) {
                let diff = {
                    x: this.position.x - other.position.x,
                    y: this.position.y - other.position.y
                };
                diff.x /= d * d;
                diff.y /= d * d;
                steering.x += diff.x;
                steering.y += diff.y;
                total++;
            }
        }
        if (total > 0) {
            steering.x /= total;
            steering.y /= total;
            steering = this.setMag(steering, MAX_SPEED);
            steering.x -= this.velocity.x;
            steering.y -= this.velocity.y;
            steering = this.limit(steering, MAX_FORCE);
        }
        return steering;
    }

    attract(targetX, targetY) {
        let desired = {
            x: targetX - this.position.x,
            y: targetY - this.position.y
        };
        let d = Math.sqrt(desired.x * desired.x + desired.y * desired.y);
        
        // Only attract if within radius
        if (d < ATTRACTION_RADIUS && d > 0) {
            desired = this.setMag(desired, MAX_SPEED);
            let steering = {
                x: desired.x - this.velocity.x,
                y: desired.y - this.velocity.y
            };
            steering = this.limit(steering, MAX_FORCE);
            return steering;
        }
        return { x: 0, y: 0 };
    }

    flock(boids, targetX, targetY) {
        let alignment = this.align(boids);
        let cohesion = this.cohesion(boids);
        let separation = this.separation(boids);
        let attraction = this.attract(targetX, targetY);

        alignment.x *= ALIGNMENT_WEIGHT;
        alignment.y *= ALIGNMENT_WEIGHT;
        cohesion.x *= COHESION_WEIGHT;
        cohesion.y *= COHESION_WEIGHT;
        separation.x *= SEPARATION_WEIGHT;
        separation.y *= SEPARATION_WEIGHT;
        attraction.x *= ATTRACTION_WEIGHT;
        attraction.y *= ATTRACTION_WEIGHT;

        this.acceleration.x += alignment.x;
        this.acceleration.y += alignment.y;
        this.acceleration.x += cohesion.x;
        this.acceleration.y += cohesion.y;
        this.acceleration.x += separation.x;
        this.acceleration.y += separation.y;
        this.acceleration.x += attraction.x;
        this.acceleration.y += attraction.y;
    }

    update() {
        this.position.x += this.velocity.x;
        this.position.y += this.velocity.y;
        this.velocity.x += this.acceleration.x;
        this.velocity.y += this.acceleration.y;
        this.velocity = this.limit(this.velocity, MAX_SPEED);
        this.acceleration.x = 0;
        this.acceleration.y = 0;
    }    show() {
        ctx.fillStyle = '#00ff00';
        ctx.shadowBlur = 2;
        ctx.shadowColor = '#00ff00';
        ctx.beginPath();
        ctx.arc(this.position.x, this.position.y, BOID_SIZE, 0, Math.PI * 2);
        ctx.fill();
        ctx.shadowBlur = 0;
    }

    distance(other) {
        let dx = this.position.x - other.position.x;
        let dy = this.position.y - other.position.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    setMag(vec, mag) {
        let current = Math.sqrt(vec.x * vec.x + vec.y * vec.y);
        if (current > 0) {
            vec.x = (vec.x / current) * mag;
            vec.y = (vec.y / current) * mag;
        }
        return vec;
    }

    limit(vec, max) {
        let mag = Math.sqrt(vec.x * vec.x + vec.y * vec.y);
        if (mag > max) {
            vec.x = (vec.x / mag) * max;
            vec.y = (vec.y / mag) * max;
        }
        return vec;
    }
}

// Create boids
const boids = [];
for (let i = 0; i < NUM_BOIDS; i++) {
    boids.push(new Boid());
}

function animateBoids() {
    // Clear canvas with fade effect (lower opacity = longer trails)
    ctx.fillStyle = 'rgba(0, 0, 0, 0.03)';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    // Draw connections between boids and their 3 closest neighbors
    drawConnections();

    // Update and draw boids
    for (let boid of boids) {
        boid.edges();
        boid.flock(boids, smoothX, smoothY);
        boid.update();
        boid.show();
    }

    requestAnimationFrame(animateBoids);
}

// Function to draw lines connecting each boid to its 3 closest neighbors
function drawConnections() {
    ctx.strokeStyle = '#000000';
    ctx.lineWidth = 1;
    ctx.shadowBlur = 8;
    ctx.shadowColor = '#00ff00';

    for (let i = 0; i < boids.length; i++) {
        let boid = boids[i];
        
        // Calculate distances to all other boids
        let distances = [];
        for (let j = 0; j < boids.length; j++) {
            if (i !== j) {
                let other = boids[j];
                let dx = other.position.x - boid.position.x;
                let dy = other.position.y - boid.position.y;
                let dist = Math.sqrt(dx * dx + dy * dy);
                distances.push({ index: j, distance: dist });
            }
        }
        
        // Sort by distance and get 3 closest
        distances.sort((a, b) => a.distance - b.distance);
        let closest3 = distances.slice(0, 3);
        
        // Draw lines to 3 closest neighbors
        for (let neighbor of closest3) {
            let other = boids[neighbor.index];
            ctx.beginPath();
            ctx.moveTo(boid.position.x, boid.position.y);
            ctx.lineTo(other.position.x, other.position.y);
            ctx.stroke();
        }
    }
    
    // Reset shadow
    ctx.shadowBlur = 0;
}

// Start boid animation
animateBoids();

// Resize canvas on window resize
window.addEventListener('resize', () => {
    canvas.width = window.innerWidth;
    canvas.height = window.innerHeight;
});

// ====== CURSOR TRACKING ======

document.addEventListener('mousemove', (e) => {
    // Update target position with actual mouse position
    targetX = e.clientX;
    targetY = e.clientY;
    
    // Update mouse square position instantly (only when not hovering nav)
    if (!isHoveringNav) {
        mouseSquare.style.left = targetX + 'px';
        mouseSquare.style.top = targetY + 'px';
    }
});

// Animation loop for smooth tracking
function animate() {    // Exponential moving average formula: smoothed = alpha * target + (1 - alpha) * smoothed
    smoothX = ALPHA * targetX + (1 - ALPHA) * smoothX;
    smoothY = ALPHA * targetY + (1 - ALPHA) * smoothY;
    
    // Update line positions to stop at the edges of the setpoint square
    // Horizontal lines
    horizontalLineLeft.style.top = smoothY + 'px';
    horizontalLineLeft.style.width = (smoothX - HALF_SQUARE) + 'px';
    
    horizontalLineRight.style.top = smoothY + 'px';
    horizontalLineRight.style.width = (window.innerWidth - smoothX - HALF_SQUARE) + 'px';
    
    // Vertical lines
    verticalLineTop.style.left = smoothX + 'px';
    verticalLineTop.style.height = (smoothY - HALF_SQUARE) + 'px';
    
    verticalLineBottom.style.left = smoothX + 'px';
    verticalLineBottom.style.height = (window.innerHeight - smoothY - HALF_SQUARE) + 'px';
    
    // Update setpoint square position
    cursorSquare.style.left = smoothX + 'px';
    cursorSquare.style.top = smoothY + 'px';
      // Calculate distance between setpoint and mouse
    const dx = targetX - smoothX;
    const dy = targetY - smoothY;
    const distance = Math.sqrt(dx * dx + dy * dy);    // Normalize distance (0 = same position, 1 = max distance or more)
    const normalizedDistance = Math.min(distance / MAX_DISTANCE, 1);
    
    let currentSize, innerSize;    if (isHoveringNav) {
        // Smoothly transition to navigation bbox size
        const navAlpha = 0.08; // Slower transition for smooth morphing effect
        currentNavWidth = navAlpha * targetNavWidth + (1 - navAlpha) * currentNavWidth;
        currentNavHeight = navAlpha * targetNavHeight + (1 - navAlpha) * currentNavHeight;
        
        // Smooth position transition to nav item center
        smoothNavX = navAlpha * targetNavX + (1 - navAlpha) * smoothNavX;
        smoothNavY = navAlpha * targetNavY + (1 - navAlpha) * smoothNavY;
        
        // Position mouse square at nav item location
        mouseSquare.style.left = smoothNavX + 'px';
        mouseSquare.style.top = smoothNavY + 'px';
        
        currentSize = Math.max(currentNavWidth, currentNavHeight);
        innerSize = currentSize - 2;
        
        // Make the square rectangular to match nav item
        mouseSquare.style.width = currentNavWidth + 'px';
        mouseSquare.style.height = currentNavHeight + 'px';
        mouseSquareInner.style.width = (currentNavWidth - 2) + 'px';
        mouseSquareInner.style.height = (currentNavHeight - 2) + 'px';
    } else {
        // Interpolate size from 51px (close) to 11px (far)
        // When distance is 0: 51px
        // When distance is max: 11px
        const minSize = 11;
        const maxSize = 51;
        currentSize = maxSize - (normalizedDistance * (maxSize - minSize));
        innerSize = currentSize - 2; // Account for 1px border on each side
        
        // Update mouse square size (square shape)
        mouseSquare.style.width = currentSize + 'px';
        mouseSquare.style.height = currentSize + 'px';
        mouseSquareInner.style.width = innerSize + 'px';
        mouseSquareInner.style.height = innerSize + 'px';
        
        // Position follows actual mouse (handled in mousemove event)
    }
    
    // Continue animation loop
    requestAnimationFrame(animate);
}

// Start animation loop
animate();

// Optional: Show lines only when mouse is in the window
document.addEventListener('mouseenter', () => {
    horizontalLineLeft.style.opacity = '0.25';
    horizontalLineRight.style.opacity = '0.25';
    verticalLineTop.style.opacity = '0.25';
    verticalLineBottom.style.opacity = '0.25';
    cursorSquare.style.opacity = '0.25';
    mouseSquare.style.opacity = '0.5';
});

document.addEventListener('mouseleave', () => {
    horizontalLineLeft.style.opacity = '0';
    horizontalLineRight.style.opacity = '0';
    verticalLineTop.style.opacity = '0';
    verticalLineBottom.style.opacity = '0';
    cursorSquare.style.opacity = '0';
    mouseSquare.style.opacity = '0';
});

// ====== NAVIGATION HOVER DETECTION ======
const navLinks = document.querySelectorAll('.nav-link');

navLinks.forEach(link => {
    link.addEventListener('mouseenter', (e) => {
        isHoveringNav = true;
        const rect = e.target.getBoundingClientRect();
        targetNavWidth = rect.width + 20; // Add padding
        targetNavHeight = rect.height + 10; // Add padding
        
        // Set target position to center of nav item
        targetNavX = rect.left + rect.width / 2;
        targetNavY = rect.top + rect.height / 2;
        
        // Initialize smooth position if first hover
        if (smoothNavX === 0 && smoothNavY === 0) {
            smoothNavX = targetNavX;
            smoothNavY = targetNavY;
        }
    });
    
    link.addEventListener('mouseleave', () => {
        isHoveringNav = false;
    });
});
