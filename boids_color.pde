import processing.video.*;  
Flock flock;
PImage img;

Capture camera;      // instance of the Capture class, used 
// to get frames from the camera

final int chunkWidth = 50;
final int chunkHeight = 50;

void setup() {
  size(640, 360);
  img = loadImage("Test.jpg");
  flock = new Flock();

  println(img.width);

  // Add an initial set of boids into the system
  for (int i = 0; i < img.width; i+= chunkWidth) {
    for (int j = 0; j < img.height; j+= chunkHeight) {
      flock.addBoid(new Boid(i, j, i, j, chunkWidth, chunkHeight));
    }
  }
}

void draw() {
  background(50);
  image(img, 0, 0);
  flock.run();
}