class Path
{
  ArrayList<PVector> points;

  PVector pos;
  float r;
  float size;

  Path()
  {
    points = new ArrayList<PVector>();
    pos = new PVector(0, 0);
    r = 45;
  }

  void addPoint(float x, float y)
  {
    points.add(new PVector(x, y));
    size = points.size();
  }

  void display()
  {
    stroke(0, 100);
    strokeWeight(r);
    noFill();
    beginShape();
    for (PVector p : points)
    {
      vertex(p.x, p.y);
    }
    endShape();

    stroke(0);
    strokeWeight(1);
    beginShape();
    for (int i = 0; i < size; i++)
    {
      pos = points.get(i);
      vertex(pos.x, pos.y);
    }
    endShape();
  }
}
