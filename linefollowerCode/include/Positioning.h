#ifndef POSITIONING_H_
#define POSITIONING_H_

class Positioning {
 public:
  Positioning(int i);
  void init();
  void update();

  float headingRad, angVelRad, posX, posY;
};

#endif /* POSITIONING_H_ */