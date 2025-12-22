const horizontalLineLeft = document.getElementById('horizontalLineLeft');
const horizontalLineRight = document.getElementById('horizontalLineRight');
const verticalLineTop = document.getElementById('verticalLineTop');
const verticalLineBottom = document.getElementById('verticalLineBottom');
const cursorSquare = document.getElementById('cursorSquare');
const mouseSquare = document.getElementById('mouseSquare');
const mouseSquareInner = document.getElementById('mouseSquareInner');
const productSection = document.querySelector('.product-section');

// ====== CURSOR CONFIGURATION ======
// Alpha parameter for exponential moving average (0 to 1)
// Higher alpha = faster tracking, Lower alpha = smoother/slower tracking
const ALPHA = 0.1;
// Blend weight between actual (0.0) and predicted (1.0) bad particle position
// 0.0 = track actual position, 1.0 = track predicted position, 0.5 = blend equally
const PREDICTION_BLEND_WEIGHT = 0.5;
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
const BOID_SIZE = 3.5;
const MAX_SPEED = 2.0;
const MAX_FORCE = 0.15;
const ROTATION_RATE = (360 * Math.PI) / 180; // 720 degrees per second in radians/second
const FRAME_RATE = 60; // Assuming 60 FPS
const MAX_ROTATION_PER_FRAME = ROTATION_RATE / FRAME_RATE; // Radians per frame

// Boid behavior weights
const SEPARATION_WEIGHT = 17.2;
const ALIGNMENT_WEIGHT = 12.0;
const COHESION_WEIGHT = 8.0;
const ATTRACTION_WEIGHT = 6.0;

// Perception radii
const SEPARATION_RADIUS = 90;
const ALIGNMENT_RADIUS = 100;
const COHESION_RADIUS = 150;
const ATTRACTION_RADIUS = 500;

// Bad particle configuration
const BAD_PARTICLE_SIZE = BOID_SIZE * 2; // Twice as big as boids
const MUTUAL_DESTRUCTION_RADIUS = 10; // Both boid and bad particle destroyed within 10px
const BAD_PARTICLE_ATTRACTION_RADIUS = 400; // Boids attracted from 400px away
const BAD_PARTICLE_ATTRACTION_FORCE = 22; // Strong attraction force (increased from 6 to 12)
const BAD_PARTICLE_DETECTION_RADIUS = 120; // Distance to detect bad particles
const BAD_PARTICLE_MAX_SPEED = 4; // Bad particles move faster than boids
const BAD_PARTICLE_ROTATION_RATE = (720 * Math.PI) / 180; // 720 degrees per second
const BAD_PARTICLE_MAX_ROTATION_PER_FRAME = BAD_PARTICLE_ROTATION_RATE / FRAME_RATE; // Radians per frame
const BAD_PARTICLE_ESCAPE_RADIUS = 80; // Flee from boids within this radius
const BAD_PARTICLE_WALL_REPEL_DISTANCE = 100; // Start repelling when within this distance from walls
const SIGNAL_DURATION = 120; // Duration of green signal in ms
const SIGNAL_COOLDOWN = 1200; // Cooldown before boid can signal again in ms
const SIGNAL_PROPAGATION_DELAY = 50; // Delay before propagating signal to neighbors in ms
const badParticles = [];
const explosionParticles = [];
const signalQueue = []; // Queue of pending signal propagations
let longPressTimer = null;
const LONG_PRESS_DURATION = 250; // 250ms for long press

// Explosion configuration
const EXPLOSION_PARTICLE_COUNT = 101;
const EXPLOSION_PARTICLE_SIZE = 2.0;
const EXPLOSION_SPEED_MIN = 1;
const EXPLOSION_SPEED_MAX = 10;
const EXPLOSION_LIFETIME = 400; // 400ms

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
        this.signalEndTime = 0; // When the current signal ends
        this.lastSignalTime = 0; // Last time this boid signaled (for cooldown)
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
    }    attract(targetX, targetY) {
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
    }    attractToBadParticles(badParticles) {
        let steering = { x: 0, y: 0 };
        let closestDist = Infinity;
        let closestParticle = null;
        
        // Find the closest bad particle
        for (let particle of badParticles) {
            let dx = particle.position.x - this.position.x;
            let dy = particle.position.y - this.position.y;
            let d = Math.sqrt(dx * dx + dy * dy);
            
            if (d < BAD_PARTICLE_ATTRACTION_RADIUS && d < closestDist) {
                closestDist = d;
                closestParticle = particle;
            }
        }
        
        // Attract to the expected position of the closest bad particle (4 frames ahead)
        if (closestParticle && closestDist > 0) {
            let expectedPos = closestParticle.getExpectedPosition();
            let desired = {
                x: expectedPos.x - this.position.x,
                y: expectedPos.y - this.position.y
            };
            desired = this.setMag(desired, MAX_SPEED);
            steering = {
                x: desired.x - this.velocity.x,
                y: desired.y - this.velocity.y
            };
            steering = this.limit(steering, MAX_FORCE * BAD_PARTICLE_ATTRACTION_FORCE);
        }
        
        return steering;
    }flock(boids, targetX, targetY, badParticles) {
        let alignment = this.align(boids);
        let cohesion = this.cohesion(boids);
        let separation = this.separation(boids);
        
        // If bad particles exist, attract to them instead of cursor
        let attraction;
        if (badParticles.length > 0) {
            attraction = this.attractToBadParticles(badParticles);
        } else {
            attraction = this.attract(targetX, targetY);
        }

        alignment.x *= ALIGNMENT_WEIGHT;
        alignment.y *= ALIGNMENT_WEIGHT;
        cohesion.x *= COHESION_WEIGHT;
        cohesion.y *= COHESION_WEIGHT;
        separation.x *= SEPARATION_WEIGHT;
        separation.y *= SEPARATION_WEIGHT;
        
        // When attracted to bad particles, use their stronger force directly
        // When attracted to cursor, use the normal weight
        if (badParticles.length === 0) {
            attraction.x *= ATTRACTION_WEIGHT;
            attraction.y *= ATTRACTION_WEIGHT;
        }

        this.acceleration.x += alignment.x;
        this.acceleration.y += alignment.y;
        this.acceleration.x += cohesion.x;
        this.acceleration.y += cohesion.y;
        this.acceleration.x += separation.x;
        this.acceleration.y += separation.y;
        this.acceleration.x += attraction.x;
        this.acceleration.y += attraction.y;
    }    update() {
        // Calculate desired direction from acceleration (steering force)
        if (this.acceleration.x !== 0 || this.acceleration.y !== 0) {
            // Get current velocity direction (heading)
            const currentSpeed = Math.sqrt(this.velocity.x * this.velocity.x + this.velocity.y * this.velocity.y);
            
            if (currentSpeed > 0.01) {
                // Normalize current velocity to get current heading
                const currentHeading = {
                    x: this.velocity.x / currentSpeed,
                    y: this.velocity.y / currentSpeed
                };
                
                // Get desired direction from acceleration
                const desiredDir = {
                    x: this.acceleration.x,
                    y: this.acceleration.y
                };
                const desiredMag = Math.sqrt(desiredDir.x * desiredDir.x + desiredDir.y * desiredDir.y);
                
                if (desiredMag > 0.01) {
                    // Normalize desired direction
                    desiredDir.x /= desiredMag;
                    desiredDir.y /= desiredMag;
                    
                    // Calculate angle between current heading and desired direction
                    const dotProduct = currentHeading.x * desiredDir.x + currentHeading.y * desiredDir.y;
                    const crossProduct = currentHeading.x * desiredDir.y - currentHeading.y * desiredDir.x;
                    const angleDiff = Math.atan2(crossProduct, dotProduct);
                    
                    // Limit rotation to MAX_ROTATION_PER_FRAME
                    const rotationAmount = Math.max(-MAX_ROTATION_PER_FRAME, Math.min(MAX_ROTATION_PER_FRAME, angleDiff));
                    
                    // Calculate new heading by rotating current heading
                    const cosRot = Math.cos(rotationAmount);
                    const sinRot = Math.sin(rotationAmount);
                    const newHeading = {
                        x: currentHeading.x * cosRot - currentHeading.y * sinRot,
                        y: currentHeading.x * sinRot + currentHeading.y * cosRot
                    };
                    
                    // Calculate projection of desired direction onto new heading
                    const projection = desiredDir.x * newHeading.x + desiredDir.y * newHeading.y;
                    
                    // Only increase speed if projection is positive (moving in the right direction)
                    let newSpeed = currentSpeed;
                    if (projection > 0) {
                        // Increase speed based on projection
                        newSpeed = Math.min(MAX_SPEED, currentSpeed + desiredMag * projection * 0.5);
                    }
                    
                    // Set new velocity
                    this.velocity.x = newHeading.x * newSpeed;
                    this.velocity.y = newHeading.y * newSpeed;
                }
            } else {
                // If nearly stationary, apply acceleration directly but limit speed
                this.velocity.x += this.acceleration.x;
                this.velocity.y += this.acceleration.y;
                this.velocity = this.limit(this.velocity, MAX_SPEED);
            }
        }
        
        // Update position
        this.position.x += this.velocity.x;
        this.position.y += this.velocity.y;
        
        // Reset acceleration
        this.acceleration.x = 0;
        this.acceleration.y = 0;
    }show() {
        ctx.globalAlpha = 0.05; // Make boids more subtle (20% opacity)
        ctx.fillStyle = '#00ff00';
        ctx.shadowBlur = 0.01;
        ctx.shadowColor = '#00ff00';
        ctx.beginPath();
        ctx.arc(this.position.x, this.position.y, BOID_SIZE, 0, Math.PI * 2);
        ctx.fill();
        ctx.shadowBlur = 0;
        ctx.globalAlpha = 1.0; // Reset alpha
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
    }    limit(vec, max) {
        let mag = Math.sqrt(vec.x * vec.x + vec.y * vec.y);
        if (mag > max) {
            vec.x = (vec.x / mag) * max;
            vec.y = (vec.y / mag) * max;
        }
        return vec;    }

    // Check if this boid is currently signaling
    isSignaling(currentTime) {
        return currentTime < this.signalEndTime;
    }

    // Check if this boid can start a new signal (not on cooldown)
    canSignal(currentTime) {
        return currentTime - this.lastSignalTime >= SIGNAL_COOLDOWN;
    }

    // Start signaling (called when detecting bad particle or receiving signal from neighbor)
    startSignal(currentTime) {
        if (this.canSignal(currentTime)) {
            this.signalEndTime = currentTime + SIGNAL_DURATION;
            this.lastSignalTime = currentTime;
            return true; // Signal was started
        }
        return false; // On cooldown, signal not started
    }

    // Check if this boid detects a bad particle
    detectsBadParticle(badParticles) {
        for (let particle of badParticles) {
            let dx = particle.position.x - this.position.x;
            let dy = particle.position.y - this.position.y;
            let dist = Math.sqrt(dx * dx + dy * dy);
            if (dist < BAD_PARTICLE_DETECTION_RADIUS) {
                return true;
            }
        }
        return false;
    }
}

// Bad Particle class
class BadParticle {
    constructor(x, y) {
        this.position = { x, y };
        this.velocity = {
            x: (Math.random() - 0.5) * BAD_PARTICLE_MAX_SPEED,
            y: (Math.random() - 0.5) * BAD_PARTICLE_MAX_SPEED
        };
        this.acceleration = { x: 0, y: 0 };
        this.pulsePhase = Math.random() * Math.PI * 2; // Random starting phase
    }

    // Calculate expected position 4 frames in the future
    getExpectedPosition() {
        const framesAhead = 8;
        return {
            x: this.position.x + this.velocity.x * framesAhead,
            y: this.position.y + this.velocity.y * framesAhead
        };
    }

    edges() {
        // Wrap around screen edges
        if (this.position.x > canvas.width) this.position.x = 0;
        if (this.position.x < 0) this.position.x = canvas.width;
        if (this.position.y > canvas.height) this.position.y = 0;
        if (this.position.y < 0) this.position.y = canvas.height;
    }    flee(boids) {
        // Calculate average position of nearby boids and flee from it
        let steering = { x: 0, y: 0 };
        let total = 0;
        
        for (let boid of boids) {
            let dx = this.position.x - boid.position.x;
            let dy = this.position.y - boid.position.y;
            let d = Math.sqrt(dx * dx + dy * dy);
            
            if (d < BAD_PARTICLE_ESCAPE_RADIUS && d > 0) {
                // Flee away from boid, weighted by distance (closer = stronger flee)
                let diff = { x: dx, y: dy };
                diff.x /= d * d; // Stronger effect when closer
                diff.y /= d * d;
                steering.x += diff.x;
                steering.y += diff.y;
                total++;
            }
        }
        
        if (total > 0) {
            steering.x /= total;
            steering.y /= total;
            
            // Normalize and scale to max speed
            let mag = Math.sqrt(steering.x * steering.x + steering.y * steering.y);
            if (mag > 0) {
                steering.x = (steering.x / mag) * BAD_PARTICLE_MAX_SPEED;
                steering.y = (steering.y / mag) * BAD_PARTICLE_MAX_SPEED;
            }
        }
          return steering;
    }    avoidWalls() {
        // Repel from walls when getting too close
        let steering = { x: 0, y: 0 };
        
        // Left wall
        if (this.position.x < BAD_PARTICLE_WALL_REPEL_DISTANCE) {
            let force = (BAD_PARTICLE_WALL_REPEL_DISTANCE - this.position.x) / BAD_PARTICLE_WALL_REPEL_DISTANCE;
            steering.x += force * force * force * BAD_PARTICLE_MAX_SPEED * 3; // Cubic force, tripled strength
        }
        
        // Right wall
        if (this.position.x > canvas.width - BAD_PARTICLE_WALL_REPEL_DISTANCE) {
            let force = (this.position.x - (canvas.width - BAD_PARTICLE_WALL_REPEL_DISTANCE)) / BAD_PARTICLE_WALL_REPEL_DISTANCE;
            steering.x -= force * force * force * BAD_PARTICLE_MAX_SPEED * 3;
        }
        
        // Top wall
        if (this.position.y < BAD_PARTICLE_WALL_REPEL_DISTANCE) {
            let force = (BAD_PARTICLE_WALL_REPEL_DISTANCE - this.position.y) / BAD_PARTICLE_WALL_REPEL_DISTANCE;
            steering.y += force * force * force * BAD_PARTICLE_MAX_SPEED * 3;
        }
        
        // Bottom wall
        if (this.position.y > canvas.height - BAD_PARTICLE_WALL_REPEL_DISTANCE) {
            let force = (this.position.y - (canvas.height - BAD_PARTICLE_WALL_REPEL_DISTANCE)) / BAD_PARTICLE_WALL_REPEL_DISTANCE;
            steering.y -= force * force * force * BAD_PARTICLE_MAX_SPEED * 3;
        }
        
        return steering;
    }    update(boids) {
        // Get flee direction from boids
        let fleeForce = this.flee(boids);
        
        // Get wall avoidance force
        let wallForce = this.avoidWalls();
        
        // Combine forces
        this.acceleration.x = fleeForce.x + wallForce.x;
        this.acceleration.y = fleeForce.y + wallForce.y;
        
        // Apply rotation constraints similar to boids
        if (this.acceleration.x !== 0 || this.acceleration.y !== 0) {
            const currentSpeed = Math.sqrt(this.velocity.x * this.velocity.x + this.velocity.y * this.velocity.y);
            
            if (currentSpeed > 0.01) {
                // Normalize current velocity to get current heading
                const currentHeading = {
                    x: this.velocity.x / currentSpeed,
                    y: this.velocity.y / currentSpeed
                };
                
                // Get desired direction from acceleration
                const desiredDir = {
                    x: this.acceleration.x,
                    y: this.acceleration.y
                };
                const desiredMag = Math.sqrt(desiredDir.x * desiredDir.x + desiredDir.y * desiredDir.y);
                
                if (desiredMag > 0.01) {
                    // Normalize desired direction
                    desiredDir.x /= desiredMag;
                    desiredDir.y /= desiredMag;
                    
                    // Calculate angle between current heading and desired direction
                    const dotProduct = currentHeading.x * desiredDir.x + currentHeading.y * desiredDir.y;
                    const crossProduct = currentHeading.x * desiredDir.y - currentHeading.y * desiredDir.x;
                    const angleDiff = Math.atan2(crossProduct, dotProduct);
                    
                    // Limit rotation to BAD_PARTICLE_MAX_ROTATION_PER_FRAME
                    const rotationAmount = Math.max(-BAD_PARTICLE_MAX_ROTATION_PER_FRAME, Math.min(BAD_PARTICLE_MAX_ROTATION_PER_FRAME, angleDiff));
                    
                    // Calculate new heading by rotating current heading
                    const cosRot = Math.cos(rotationAmount);
                    const sinRot = Math.sin(rotationAmount);
                    const newHeading = {
                        x: currentHeading.x * cosRot - currentHeading.y * sinRot,
                        y: currentHeading.x * sinRot + currentHeading.y * cosRot
                    };
                    
                    // Calculate projection of desired direction onto new heading
                    const projection = desiredDir.x * newHeading.x + desiredDir.y * newHeading.y;
                    
                    // Only increase speed if projection is positive (moving in the right direction)
                    let newSpeed = currentSpeed;
                    if (projection > 0) {
                        // Increase speed based on projection
                        newSpeed = Math.min(BAD_PARTICLE_MAX_SPEED, currentSpeed + desiredMag * projection * 0.5);
                    }
                    
                    // Set new velocity
                    this.velocity.x = newHeading.x * newSpeed;
                    this.velocity.y = newHeading.y * newSpeed;
                }
            } else {
                // If nearly stationary, apply acceleration directly but limit speed
                this.velocity.x += this.acceleration.x;
                this.velocity.y += this.acceleration.y;
                let mag = Math.sqrt(this.velocity.x * this.velocity.x + this.velocity.y * this.velocity.y);
                if (mag > BAD_PARTICLE_MAX_SPEED) {
                    this.velocity.x = (this.velocity.x / mag) * BAD_PARTICLE_MAX_SPEED;
                    this.velocity.y = (this.velocity.y / mag) * BAD_PARTICLE_MAX_SPEED;
                }
            }
        }
        
        // Update position
        this.position.x += this.velocity.x;
        this.position.y += this.velocity.y;
        
        // Reset acceleration
        this.acceleration.x = 0;
        this.acceleration.y = 0;
    }show(time) {
        // Draw prediction arrow first (before particle so it's behind)
        // this.drawPredictionArrow(); // Arrow is now invisible
        
        // Really bright green glow - much more visible
        // Draw the bad particle with intense green glow
        ctx.globalAlpha = 1.0; // Full opacity - not transparent at all
        ctx.fillStyle = '#00ff00'; // Bright green fill
        ctx.shadowBlur = 60; // Much larger glow (was 20, now 60)
        ctx.shadowColor = '#00ff00';
        
        // Draw multiple layers for extra brightness
        ctx.beginPath();
        ctx.arc(this.position.x, this.position.y, BAD_PARTICLE_SIZE, 0, Math.PI * 2);
        ctx.fill();
        
        // Add second glow layer for extra brightness
        ctx.shadowBlur = 40;
        ctx.beginPath();
        ctx.arc(this.position.x, this.position.y, BAD_PARTICLE_SIZE, 0, Math.PI * 2);
        ctx.fill();
        
        // Add third inner bright core
        ctx.shadowBlur = 20;
        ctx.beginPath();
        ctx.arc(this.position.x, this.position.y, BAD_PARTICLE_SIZE, 0, Math.PI * 2);
        ctx.fill();
        
        ctx.shadowBlur = 0;
        ctx.globalAlpha = 1.0;
    }

    drawPredictionArrow() {
        const expectedPos = this.getExpectedPosition();
        
        // Draw arrow from current position to expected position
        ctx.globalAlpha = 0.4; // Semi-transparent arrow
        ctx.strokeStyle = '#00ff00';
        ctx.lineWidth = 2;
        ctx.shadowBlur = 5;
        ctx.shadowColor = '#00ff00';
        
        // Draw line
        ctx.beginPath();
        ctx.moveTo(this.position.x, this.position.y);
        ctx.lineTo(expectedPos.x, expectedPos.y);
        ctx.stroke();
        
        // Draw arrowhead
        const angle = Math.atan2(expectedPos.y - this.position.y, expectedPos.x - this.position.x);
        const arrowLength = 10;
        const arrowWidth = Math.PI / 6; // 30 degrees
        
        ctx.beginPath();
        ctx.moveTo(expectedPos.x, expectedPos.y);
        ctx.lineTo(
            expectedPos.x - arrowLength * Math.cos(angle - arrowWidth),
            expectedPos.y - arrowLength * Math.sin(angle - arrowWidth)
        );
        ctx.moveTo(expectedPos.x, expectedPos.y);
        ctx.lineTo(
            expectedPos.x - arrowLength * Math.cos(angle + arrowWidth),
            expectedPos.y - arrowLength * Math.sin(angle + arrowWidth)
        );
        ctx.stroke();
        
        ctx.shadowBlur = 0;
        ctx.globalAlpha = 1.0;
    }

    checkCollision(boid) {
        const dx = this.position.x - boid.position.x;
        const dy = this.position.y - boid.position.y;
        const distance = Math.sqrt(dx * dx + dy * dy);
        return distance < MUTUAL_DESTRUCTION_RADIUS;
    }
}

// Explosion Particle class
class ExplosionParticle {
    constructor(x, y, badParticleVelocity = null) {
        this.position = { x, y };
        // Random velocity in any direction
        const angle = Math.random() * Math.PI * 2;
        const speed = EXPLOSION_SPEED_MIN + Math.random() * (EXPLOSION_SPEED_MAX - EXPLOSION_SPEED_MIN);
        this.velocity = {
            x: Math.cos(angle) * speed,
            y: Math.sin(angle) * speed
        };
        
        // Store bad particle velocity for later adjustment
        this.badParticleVelocity = badParticleVelocity;
        
        this.birthTime = Date.now();
        this.lifetime = EXPLOSION_LIFETIME;
    }

    update() {
        this.position.x += this.velocity.x;
        this.position.y += this.velocity.y;
    }    show(currentTime) {
        const age = currentTime - this.birthTime;
        const lifeRatio = 1 - (age / this.lifetime); // 1.0 at birth, 0.0 at death
        
        if (lifeRatio <= 0) return false; // Signal particle should be removed
        
        // Fade opacity from 0.5 to 0 (less transparent, more visible)
        const opacity = lifeRatio * 0.5;
        
        // Draw explosion particle with trail effect
        ctx.globalAlpha = opacity;
        ctx.fillStyle = '#00ff00';
        ctx.shadowBlur = 3 * lifeRatio; // Glow fades with life
        ctx.shadowColor = '#00ff00';
        ctx.beginPath();
        ctx.arc(this.position.x, this.position.y, EXPLOSION_PARTICLE_SIZE, 0, Math.PI * 2);
        ctx.fill();
        ctx.shadowBlur = 0;
        ctx.globalAlpha = 1.0;
        
        return true; // Particle still alive
    }

    isDead(currentTime) {
        return (currentTime - this.birthTime) > this.lifetime;
    }
}

// Create boids
const boids = [];
for (let i = 0; i < NUM_BOIDS; i++) {
    boids.push(new Boid());
}

function animateBoids() {
    // Clear canvas with fade effect (higher opacity = shorter trails)
    ctx.fillStyle = 'rgba(0, 0, 0, 0.3)'; // Increased from 0.03 to 0.3 for much shorter trails
    ctx.fillRect(0, 0, canvas.width, canvas.height);    // Get current time for pulsing effect and signaling
    const currentTime = Date.now();

    // Hide/show product section based on bad particles
    if (badParticles.length > 0) {
        productSection.style.opacity = '0';
        productSection.style.pointerEvents = 'none';
    } else {
        productSection.style.opacity = '1';
        productSection.style.pointerEvents = 'none';
    }

    // Process signal queue - propagate signals that have waited long enough
    for (let i = signalQueue.length - 1; i >= 0; i--) {
        let queuedSignal = signalQueue[i];
        if (currentTime >= queuedSignal.triggerTime) {
            // Time to propagate this signal
            propagateSignalToNeighbors(queuedSignal.boidIndex, currentTime);
            signalQueue.splice(i, 1); // Remove from queue
        }
    }

    // First pass: Check for bad particle detection and start signaling
    for (let i = 0; i < boids.length; i++) {
        let boid = boids[i];
        
        // If boid detects a bad particle and can signal, start signaling
        if (boid.detectsBadParticle(badParticles) && boid.canSignal(currentTime)) {
            if (boid.startSignal(currentTime)) {
                // Queue signal propagation with delay
                signalQueue.push({
                    boidIndex: i,
                    triggerTime: currentTime + SIGNAL_PROPAGATION_DELAY
                });
            }
        }
    }

    // Draw connections between boids and their 3 closest neighbors
    drawConnections(currentTime);

    // Update and draw explosion particles
    for (let i = explosionParticles.length - 1; i >= 0; i--) {
        let particle = explosionParticles[i];
        particle.update();
        const stillAlive = particle.show(currentTime);
        
        // Remove dead particles
        if (!stillAlive || particle.isDead(currentTime)) {
            explosionParticles.splice(i, 1);
        }
    }

    // Update and draw boids, check for collisions with bad particles
    for (let i = boids.length - 1; i >= 0; i--) {
        let boid = boids[i];
          // Check collision with bad particles
        let destroyed = false;
        for (let j = badParticles.length - 1; j >= 0; j--) {
            let particle = badParticles[j];
            if (particle.checkCollision(boid)) {
                // Create explosion at bad particle position with conservation of momentum
                let newExplosionParticles = [];
                
                // Step 1: Generate all particles with random velocities
                for (let k = 0; k < EXPLOSION_PARTICLE_COUNT; k++) {
                    newExplosionParticles.push(new ExplosionParticle(particle.position.x, particle.position.y, particle.velocity));
                }
                
                // Step 2: Calculate mean velocity of all explosion particles
                let meanVx = 0, meanVy = 0;
                for (let expParticle of newExplosionParticles) {
                    meanVx += expParticle.velocity.x;
                    meanVy += expParticle.velocity.y;
                }
                meanVx /= EXPLOSION_PARTICLE_COUNT;
                meanVy /= EXPLOSION_PARTICLE_COUNT;
                
                // Step 3: Adjust each particle's velocity to conserve momentum
                // We want the mean to equal the bad particle's velocity
                const velocityAdjustmentX = particle.velocity.x - meanVx;
                const velocityAdjustmentY = particle.velocity.y - meanVy;
                
                for (let expParticle of newExplosionParticles) {
                    expParticle.velocity.x += velocityAdjustmentX;
                    expParticle.velocity.y += velocityAdjustmentY;
                }
                
                // Add adjusted particles to main array
                explosionParticles.push(...newExplosionParticles);
                
                boids.splice(i, 1); // Remove the boid
                badParticles.splice(j, 1); // Remove the bad particle (mutual destruction)
                destroyed = true;
                break;
            }
        }
          // Only update and show if not destroyed
        if (!destroyed) {
            boid.edges();
            boid.flock(boids, smoothX, smoothY, badParticles);
            boid.update();
            boid.show();        }
    }

    // Draw bad particles LAST so they appear on top without being faded
    for (let particle of badParticles) {
        particle.edges();
        particle.update(boids);
        particle.show(currentTime);
    }    requestAnimationFrame(animateBoids);
}

// Function to propagate signal to 3 nearest neighbors (called after delay)
function propagateSignalToNeighbors(boidIndex, currentTime) {
    // Check if boid still exists (might have been destroyed)
    if (boidIndex >= boids.length) return;
    
    let boid = boids[boidIndex];
    
    // Calculate distances to all other boids
    let distances = [];
    for (let j = 0; j < boids.length; j++) {
        if (boidIndex !== j) {
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
    
    // Signal the 3 closest neighbors
    for (let neighbor of closest3) {
        let neighborBoid = boids[neighbor.index];
        if (neighborBoid.canSignal(currentTime)) {
            if (neighborBoid.startSignal(currentTime)) {
                // Queue propagation to their neighbors with delay
                signalQueue.push({
                    boidIndex: neighbor.index,
                    triggerTime: currentTime + SIGNAL_PROPAGATION_DELAY
                });
            }
        }
    }
}

// Function to draw lines connecting each boid to its 3 closest neighbors
function drawConnections(currentTime) {
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
            
            // Check if this boid is currently signaling
            if (boid.isSignaling(currentTime)) {
                // Draw solid green line
                ctx.globalAlpha = 1.0; // Full opacity
                ctx.strokeStyle = '#00ff00'; // Bright green
                ctx.lineWidth = 2;
                ctx.shadowBlur = 15;
                ctx.shadowColor = '#00ff00';
            } else {
                // Draw normal black line with subtle green glow
                ctx.globalAlpha = 0.15; // Make connections subtle (15% opacity)
                ctx.strokeStyle = '#000000';
                ctx.lineWidth = 2;
                ctx.shadowBlur = 13;
                ctx.shadowColor = '#00ff00';
            }
            
            ctx.beginPath();
            ctx.moveTo(boid.position.x, boid.position.y);
            ctx.lineTo(other.position.x, other.position.y);
            ctx.stroke();
        }
    }
    
    // Reset shadow and alpha
    ctx.shadowBlur = 0;
    ctx.globalAlpha = 1.0;
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
function animate() {
    // If bad particles exist, track the one closest to any boid
    let trackingBadParticle = false;
    if (badParticles.length > 0) {
        let closestDistance = Infinity;
        let closestBadParticle = null;
        
        // Find the bad particle with minimum distance to any boid
        for (let particle of badParticles) {
            for (let boid of boids) {
                let dx = particle.position.x - boid.position.x;
                let dy = particle.position.y - boid.position.y;
                let dist = Math.sqrt(dx * dx + dy * dy);
                
                if (dist < closestDistance) {
                    closestDistance = dist;
                    closestBadParticle = particle;
                }
            }        }
          // Set target to blend between actual and predicted position
        if (closestBadParticle) {
            // Get actual position
            const actualX = closestBadParticle.position.x;
            const actualY = closestBadParticle.position.y;
            
            // Get predicted position (8 frames ahead)
            const predicted = closestBadParticle.getExpectedPosition(8);
            const predictedX = predicted.x;
            const predictedY = predicted.y;
            
            // Blend between actual and predicted based on PREDICTION_BLEND_WEIGHT
            // Weight 0.0 = actual, Weight 1.0 = predicted, Weight 0.5 = 50/50 blend
            targetX = actualX * (1 - PREDICTION_BLEND_WEIGHT) + predictedX * PREDICTION_BLEND_WEIGHT;
            targetY = actualY * (1 - PREDICTION_BLEND_WEIGHT) + predictedY * PREDICTION_BLEND_WEIGHT;
            trackingBadParticle = true;
        }}
    // Otherwise, targetX and targetY follow the mouse (set by mousemove event)
    
    // Update cursor line opacity based on whether we're tracking a bad particle
    const cursorOpacity = trackingBadParticle ? 0.45 : 0.25; // 3x brighter when tracking
    horizontalLineLeft.style.opacity = cursorOpacity;
    horizontalLineRight.style.opacity = cursorOpacity;
    verticalLineTop.style.opacity = cursorOpacity;
    verticalLineBottom.style.opacity = cursorOpacity;
    cursorSquare.style.opacity = cursorOpacity;
    
    // Use 3x faster tracking (higher alpha) when following bad particle for more responsive targeting
    const trackingAlpha = trackingBadParticle ? (ALPHA * 3) : ALPHA;
    
    // Exponential moving average formula: smoothed = alpha * target + (1 - alpha) * smoothed
    smoothX = trackingAlpha * targetX + (1 - trackingAlpha) * smoothX;
    smoothY = trackingAlpha * targetY + (1 - trackingAlpha) * smoothY;
    
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

// Note: Cursor line opacity is now controlled dynamically in the animate() function
// based on whether tracking a bad particle (0.75) or following mouse (0.25)

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

// ====== BAD PARTICLE CREATION ======

// Right-click to create bad particle
document.addEventListener('contextmenu', (e) => {
    e.preventDefault(); // Prevent context menu
    
    // Only create if we haven't reached the limit (1/4 of boids)
    const maxBadParticles = Math.floor(boids.length / 4);
    if (badParticles.length < maxBadParticles) {
        badParticles.push(new BadParticle(e.clientX, e.clientY));
    }
});

// Long-press for mobile (touchstart/touchend)
document.addEventListener('touchstart', (e) => {
    const touch = e.touches[0];
    const touchX = touch.clientX;
    const touchY = touch.clientY;
    
    longPressTimer = setTimeout(() => {
        // Only create if we haven't reached the limit (1/4 of boids)
        const maxBadParticles = Math.floor(boids.length / 4);
        if (badParticles.length < maxBadParticles) {
            badParticles.push(new BadParticle(touchX, touchY));
        }
        longPressTimer = null;
    }, LONG_PRESS_DURATION);
});

document.addEventListener('touchend', () => {
    if (longPressTimer) {
        clearTimeout(longPressTimer);
        longPressTimer = null;
    }
});

document.addEventListener('touchmove', () => {
    if (longPressTimer) {
        clearTimeout(longPressTimer);
        longPressTimer = null;
    }
});
