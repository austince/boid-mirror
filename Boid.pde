// The Boid class

public class Boid {

  PVector position;
  PVector tintColor;

  PVector velocity;
  PVector colorVel;

  PVector acceleration;
  PVector colorAcc;

  PVector avgColor;
  float maxForce;    // Maximum steering force
  float maxSpeed;    // Maximum speed
  float scale = 0.75;
  float mass = 1;

  float desiredSeparation = 25.0f; // magnitude apart
  float desiredColorSep = 25.0f;
  
  // For image
  int chunkWidth, chunkHeight;
  int imgSrcX, imgSrcY;
  PImage img;

  float separationWeight = 1.0;
  float alignmentWeight = 1.5;
  float cohesionWeight = 1.5;

  Boid(float x, float y, int imgX, int imgY, int w, int h) {
    acceleration = new PVector(0, 0);

    velocity = PVector.random2D();
    colorVel = new PVector(0, 0, 0);

    avgColor = new PVector(0, 0, 0); // start with black
    position = new PVector(x, y);
    tintColor = new PVector(0, 0, 0);
    maxSpeed = (w + h) / 2;
    //maxSpeed = 0;
    maxForce = 0.03;

    imgSrcX = imgX;
    imgSrcY = imgY;
    chunkWidth = w;
    chunkHeight = h;

    desiredSeparation = (chunkWidth + chunkHeight) / 2;
  }

  public void run(ArrayList<Boid> boids, PImage img) {
    this.img = img;
    aggregateColor();
    flock(boids);
    update();
    borders();
    render();
  }

  public void applyForce(PVector to, PVector force) {
    // We could add mass here if we want A = F / M
    to.add(force).div(mass);
  }

  // We accumulate a new acceleration each time based on three rules
  public void flock(ArrayList<Boid> boids) {
    PVector sep = separate(boids);   // Separation
    PVector ali = align(boids);      // Alignment
    PVector coh = cohesion(boids);   // Cohesion
    // Arbitrarily weight these forces
    sep.mult(separationWeight);
    ali.mult(alignmentWeight);
    coh.mult(cohesionWeight);
    // Add the force vectors to acceleration
    applyForce(acceleration, sep);
    applyForce(acceleration, ali);
    applyForce(acceleration, coh);
  }

  // Method to update position
  public void update() {
    // Update velocity
    velocity.add(acceleration);
    // Limit speed
    velocity.limit(maxSpeed);
    //position.add(velocity);
    tintColor.add(velocity);
    // Reset accelertion to 0 each cycle
    acceleration.mult(0);
  }

  // A method that calculates and applies a steering force towards a target
  // STEER = DESIRED MINUS VELOCITY
  public PVector seek(PVector target) {
    PVector desired = PVector.sub(target, position);  // A vector pointing from the position to the target
    // Scale to maximum speed
    desired.setMag(maxSpeed);

    // Steering = Desired minus Velocity
    PVector steer = PVector.sub(desired, velocity);
    steer.limit(maxForce);  // Limit to maximum steering force
    return steer;
  }

  public void render() {
    // Draw a triangle rotated in the direction of velocity
    float theta = velocity.heading() + radians(90);
    PImage imgChunk = img.get(imgSrcX, imgSrcY, chunkWidth, chunkHeight);

    pushMatrix();
    translate(position.x, position.y);
    rotate(theta);
    scale(scale);

    // Draw the image chunk
    imageMode(CENTER);
    // tint(colorFromVector(avgColor));
    tint(colorFromVector(tintColor));
    image(imgChunk, 0, 0);

    popMatrix();
  }

  // Wraparound
  public void borders() {
    if (position.x < -chunkWidth) position.x = width+chunkWidth;
    if (position.y < -chunkHeight) position.y = height+chunkHeight;
    if (position.x > width+chunkWidth) position.x = -chunkWidth;
    if (position.y > height+chunkHeight) position.y = -chunkHeight;
  }

  // Separation
  // Method checks for nearby boids and steers away
  public PVector separate (ArrayList<Boid> boids) {
    PVector steer = new PVector(0, 0, 0);
    PVector colorSteer = steer.copy();
    int count = 0;
    int colorCount = 0;
    // For every boid in the system, check if it's too close
    for (Boid other : boids) {
      float d = PVector.dist(position, other.position);
      float dColor = PVector.dist(avgColor, other.avgColor);
      //println("dColor:", dColor);
      //d = (d + dColor) / 2;
      //d += PVector.dist(avgColor, other.avgColor);
      //d += Math.abs(avgColor.x - other.avgColor.x); // red
      // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
      if ((d > 0) && (d < desiredSeparation)) {
        // Calculate vector pointing away from neighbor
        PVector diff = PVector.sub(position, other.position);
        diff.normalize();
        diff.div(d);        // Weight by distance
        steer.add(diff);
        count++;            // Keep track of how many
      }
      
      // Do the same with color
      if ((dColor > 0) && (dColor < desiredColorSep)) {
           PVector diff = PVector.sub(position, other.position);
          diff.normalize();
          diff.div(d);        // Weight by distance
          colorSteer.add(diff);
          colorCount++;            // Keep track of how many
      }
    }
    // Average -- divide by how many
    if (count > 0) {
      steer.div((float)count);
    }

    if (colorCount > 0) {
      colorSteer.div((float) colorCount);
    }

    // As long as the vector is greater than 0
    if (steer.mag() > 0) {
      // Implement Reynolds: Steering = Desired - Velocity
      steer.setMag(maxSpeed);
      steer.sub(velocity);
      steer.limit(maxForce);
    }

  if (colorSteer.mag() > 0) {
      colorSteer.setMag(maxSpeed);
      colorSteer.sub(colorVel);
      colorSteer.limit(maxForce);
  }

    return steer;
  }

  // Alignment
  // For every nearby boid in the system, calculate the average velocity
  public PVector align (ArrayList<Boid> boids) {
    float neighbordist = 50;
    PVector sum = new PVector(0, 0);
    int count = 0;
    for (Boid other : boids) {
      float d = PVector.dist(position, other.position);
      if ((d > 0) && (d < neighbordist)) {
        sum.add(other.velocity);
        count++;
      }
    }
    if (count > 0) {
      sum.div((float)count);
      // Implement Reynolds: Steering = Desired - Velocity
      sum.setMag(maxSpeed);
      PVector steer = PVector.sub(sum, velocity);
      steer.limit(maxForce);
      return steer;
    } else {
      return new PVector(0, 0);
    }
  }

  // Cohesion
  // For the average position (i.e. center) of all nearby boids, calculate steering vector towards that position
  public PVector cohesion (ArrayList<Boid> boids) {
    float neighbordist = 50;
    PVector sum = new PVector(0, 0);   // Start with empty vector to accumulate all positions
    int count = 0;
    for (Boid other : boids) {
      float d = PVector.dist(position, other.position);
      if ((d > 0) && (d < neighbordist)) {
        sum.add(other.position); // Add position
        count++;
      }
    }
    if (count > 0) {
      sum.div(count);
      return (sum);  // Steer towards the position
    } else {
      return new PVector(0, 0);
    }
  }

  public void aggregateColor() {
    PVector agg = new PVector(0, 0, 0);
    int xMax = constrain(imgSrcX + chunkWidth, 0, img.width);
    int yMax = constrain(imgSrcY + chunkHeight, 0, img.height);
    int sampleWidth = 10;
    int sampleHeight = 10;
    int total = 0;
    for (int i = imgSrcX; i < xMax; i+=sampleWidth) {
      for (int j = imgSrcY; j < yMax; j+=sampleHeight) {
        //int index = (img.width-i-1) + j*img.width;
        int index = (img.width-i-1) + j*img.width;
        //println(index, img.pixels.length);
        agg.x += img.pixels[index] >> 16 & 0xFF;
        agg.y += img.pixels[index] >> 8 & 0xFF;
        agg.z += img.pixels[index] & 0xFF;
        total++;
      }
    }
    agg.x /= total;
    agg.y /= total;
    agg.z /= total;
    this.avgColor = agg;
  }
}