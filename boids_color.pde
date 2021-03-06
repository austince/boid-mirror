import processing.video.*;  
Flock flock;

Capture camera;      // instance of the Capture class, subclass of PImage

final int chunkWidth = 50;
final int chunkHeight = 50;

void setup() {
  size(1280, 720);
  flock = new Flock();

  String camId = getCameraBySpecs(width, height, 30);
  if (camId == null) {
    println("Can't find our camera.");
    // exit();
    // Just get first camera so it doesn't break
    camId = getFirstCamera();
  }
  camera = new Capture(this, camId);
  camera.start();

  // Add an initial set of boids into the system
  for (int i = 0; i < width; i+= chunkWidth) {
    for (int j = 0; j < height; j+= chunkHeight) {
      flock.addBoid(new Boid(i, j, i, j, chunkWidth, chunkHeight));
    }
  }
}

String getFirstCamera() {
  String[] cameras = Capture.list();
  return cameras[0];
}

String getCameraBySpecs(int w, int h, int fps) {
  String camFound = null;

  String[] cameras = Capture.list();
  for (String camId : cameras) {
    if (camId.indexOf("size=" + w + "x" + h + ",fps=" + fps) >= 0) {
      camFound = camId;
      break;
    }
  }
  return camFound;
}

void draw() {
  if (camera.available()) {
    camera.read(); 

    PImage flipped = flipImageOverX(camera);
    
    background(50);
    //flipped.loadPixels();
    flock.run(flipped);
    //flipped.updatePixels();
  }
}


PImage flipImageOverX(PImage img) {
  PImage flipped = createImage(img.width, img.height, RGB);//create a new image with the same dimensions
  for (int i = 0; i < flipped.pixels.length; i++) {       //loop through each pixel
    int srcX = i % flipped.width;                        //calculate source(original) x position
    int dstX = flipped.width-srcX-1;                     //calculate destination(flipped) x position = (maximum-x-1)
    int y    = i / flipped.width;                        //calculate y coordinate
    flipped.pixels[y*flipped.width+dstX] = img.pixels[i];//write the destination(x flipped) pixel based on the current pixel
  }
  return flipped;
}