class Walker
{
  PVector position;  // position of Walker
  PVector futurePosition; // future position of Walker
  PVector velocity; // velocity of Walker
  PVector tempVel; // temporary velocity for caclulations. DO NOT USE IT WITHOUT CLEARING THIS BEFORE
  PVector acceleration; // acceleration of Walker
  PVector normalPoint; // normal point of two vectors
  PVector unitPathVector; // for getting the angle of path Vector;
  PVector tempNormalPoint; // temporary normal point of two vectors;
  PVector target; // for chasing, some time in future from normal point

  float size; // size of Walker
  float maxSpeed; // maximum speed of Walker
  float maxForce; // maximym force that can act upon Walker
  float steps; // how for it can see in the future
  float minDistanceFromOthers; // minimum distance from others

  color colour;

  // Constructor
  Walker()
  {
    maxForce = 0.005;
    maxSpeed = random(2, 4);

    size = random(4, 10);
    colour = 172;
    steps = 25;

    minDistanceFromOthers = size*1.5;

    position = new PVector(random(width), random(height));
    velocity = new PVector(random(-maxSpeed, maxSpeed), random(-maxSpeed, maxSpeed));
    //velocity = new PVector(2, 0);
    //position = new PVector(320, 70);

    tempVel = new PVector(0, 0);
    acceleration = new PVector(0, 0);
    normalPoint = new PVector(0, 0);
    tempNormalPoint = new PVector(0, 0);
    target = new PVector(0, 0);
    unitPathVector = new PVector(0, 0);
  }

  // for separating individuals
  void separation(ArrayList<Walker> walkers)
  {
    float dist;
    PVector sum = new PVector();
    int count = 0;

    for (Walker w : walkers)
    {
      dist = PVector.dist(position, w.position);

      if ((dist > 0) && (dist < minDistanceFromOthers))
      {
        sum.add(PVector.sub(position, w.position).normalize().div(dist));
        count++;
      }

      if (count > 0)
      {
        sum.div(count);
        sum.normalize();
        sum.mult(maxSpeed);
        PVector steer = PVector.sub(sum, velocity);
        applyForce(steer);
      }
    }
  }

  boolean checkAction(float r)
  {
    float a = PVector.sub(normalPoint, futurePosition).mag();
    if (a > r/2)
    {
      return true;
    } else
    {
      return false;
    }
  }

  void chaseTarget()
  {
    PVector desire = PVector.sub(target, position);
    PVector steer = PVector.sub(desire, velocity);

    applyForce(steer);
  }

  void setTarget()
  {
    target = normalPoint.copy();
    tempVel = unitPathVector.copy();
    tempVel.normalize();
    tempVel.mult(steps);
    target.add(tempVel);
  }

  void chooseClosestPath(ArrayList<PVector> points, PVector pos)
  {
    float Distance = 1000000;
    float dist = 1000000;
    PVector vector = null;
    PVector a = null;
    PVector b = null;

    for (int i = 0; i < points.size() -1; i++)
    {
      a = points.get(i);
      b = points.get(i+1);
      vector = getNormalPosition(pos, a, b);

      if (vector.x < a.x) vector = a.copy();
      if (vector.x > b.x) vector = b.copy();

      dist = PVector.sub(vector, pos).mag();
      if (Distance > dist)
      {
        unitPathVector = PVector.sub(b, a).normalize();
        normalPoint = vector.copy();
        Distance = dist;
      }
    }
  }

  // origin - origin of both vector, a - vector of Walker, b - second vector of path
  PVector getNormalPosition(PVector a, PVector origin, PVector b)
  {
    PVector oa = PVector.sub(a, origin);
    PVector ob = PVector.sub(b, origin);

    ob.normalize();

    float dot = oa.dot(ob);
    ob.mult(dot);
    tempNormalPoint = origin.copy();
    tempNormalPoint.add(ob);

    return tempNormalPoint;
  }

  void setFuturePosition()
  {
    futurePosition = position.copy();
    tempVel = velocity.copy();
    tempVel.normalize();
    tempVel.mult(steps);
    futurePosition.add(tempVel);
  }

  // act on the Walker
  void applyForce(PVector force)
  {
    force.mult(maxForce);
    acceleration.add(force);
  }

  // update the position of Walker 
  void update()
  {
    velocity.add(acceleration);
    velocity.limit(maxSpeed);
    position.add(velocity);

    acceleration.mult(0);
  }

  //Scrolling Walkers beyond the screen
  void edges()
  {
    if (position.x + size/2 < 0) position.x = width + size/2;
    if (position.x - size/2 > width) position.x = 0 - size/2;
    if (position.y + size/2 < 0) position.y = height + size/2;
    if (position.y - size/2 > height) position.y = 0 - size/2;
  }

  // draw circle
  void drawCircle(PVector pos, float s, color col)
  {
    stroke(1);
    strokeWeight(1);
    fill(col);
    ellipse(pos.x, pos.y, s, s);
  }

  // display Walker
  void display(Path p)
  {
    update();
    edges();
    setFuturePosition();
    //normalPoint = getNormalPosition(futurePosition, p.points.get(0), p.points.get(1));
    chooseClosestPath(p.points, futurePosition);

    setTarget();
    if (checkAction(p.r))chaseTarget();

    drawCircle(position, size, colour);
    //drawCircle(futurePosition, 4, color(0, 0, 255));
    //drawCircle(normalPoint, 3, color(0, 0, 255));
    //drawCircle(target, 5, color(255, 0, 255));
  }
}
