
// The Flock (a list of Boid objects)

class Flock {
  ArrayList<Boid> boids; // An ArrayList for all the boids

  public Flock() {
    boids = new ArrayList<Boid>(); // Initialize the ArrayList
  }

  public void run(PImage img) {
    for (Boid b : boids) {
      b.run(boids, img);  // Passing the entire list of boids to each boid individually
    }
  }

  public void addBoid(Boid b) {
    boids.add(b);
  }
}