#  Boid Mosaic Mirror
The final product is a modification of the Boid Flocking algorithm applied to the webcam. Or that's what the initial idea was. 
After experimenting with motion using chunks of the image as "Boids", I decided that the best motion is no motion. 
I also tried applying the Boid Algorithm to the <R,G,B> colorspace but was also unhappy with the result. 

The final mirror abstracts what is sees into smaller chunks and tries to mirror an average color. The effect is something of a shadow.
From afar, the "shadow" and the object will be indistinguishable. You can only see the granular differences and the rotation of the individual chunks upon closer
inspection. The final result really doesn't need the Boid Algorithm at all but it is an interesting relic to how we got here. 

## Boid Algorithm
I used [Processing's Example](https://processing.org/examples/flocking.html) of the Boid algorithm to start. 

## Sketches
My sketches were kind of all over the place with this project. I went through many ideas before settling on the mosaic Boid movement, which ended up where I did most of my sketching.

[Sketches Folder](./sketches)  
![](./sketches/initial-xsmall.jpg)
![](./sketches/floating-xsmall.jpg)
![](./sketches/chunks-xsmall.jpg)


## Screenshots

### From Close Up
![Up Close Screenshot](./screenshots/up-close.png)

### From Further away
![Far Out Screenshot](./screenshots/far-out.png)

## Video
[Watch it here on Vimeo](https://vimeo.com/255647357)

### Other Screenshots
While developing this, I did a bunch of experiments. I saved the decent ones do their own branches. Here are what they looked like:

Colored Quadrants:
![](./screenshots/colored-quadrants.png)

Rotated Squares:
![](./screenshots/rotated-squares.png)