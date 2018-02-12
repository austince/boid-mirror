import processing.video.*;  
Flock flock;

Capture camera;      // instance of the Capture class, subclass of PImage

final int chunkWidth = 50;
final int chunkHeight = 50;

void setup() {
  size(1280, 720);
  flock = new Flock();

  String cam = getCameraBySpecs(width, height, 30);
  if (cam == null) {
    println("Can't find our camera.");
    exit();
  }
  camera = new Capture(this, cam);
  camera.start();

  // Add an initial set of boids into the system
  for (int i = 0; i < width; i+= chunkWidth) {
    for (int j = 0; j < height; j+= chunkHeight) {
      flock.addBoid(new Boid(i, j, i, j, chunkWidth, chunkHeight));
    }
  }
}


String getCameraBySpecs(int w, int h, int fps) {
  String camFound = null;

  String[] cameras = Capture.list();
  for (String cam : cameras) {
    if (cam.indexOf("size=" + w + "x" + h + ",fps=" + fps) >= 0) {
      camFound = cam;
      break;
    }
  }
  return camFound;
}

void draw() {
  if (camera.available()) {
    camera.read(); 
    background(50);
    //image(camera, 0, 0);
    flock.run(camera);
  }
}