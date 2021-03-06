ArrayList<Walker> walkers;
//Walker walker;
Path path;

int NoP = 30; // number of paths
float t; // for Perlin's noise

void setup()
{
  size(1000, 360);

  float w = width/NoP; // lenght of part of path
  float h; // y position of path
  t = 0.1;

  path = new Path();
  walkers = new ArrayList<Walker>();
  
  for (int i = 0; i < 80; i++)
  {
    walkers.add(new Walker());
  }

  for (int i = 0; i <= NoP; i++)
  {
    h = noise(t);
    h = map(h, 0, 1, 50, height-50);
    path.addPoint(i*w, h);
    t += 0.1;
  }
}

void draw()
{
  background(255);

  path.display();

  for (Walker w : walkers)
  {
    w.separation(walkers);
    w.display(path);
  }
}
