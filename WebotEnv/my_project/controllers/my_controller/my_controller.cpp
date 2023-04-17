#include <webots/Supervisor.hpp>
#include <webots/Marker.hpp>
#include <cmath>

using namespace webots;

int main(int argc, char** argv) {
  Supervisor* supervisor = new Supervisor();
  Node* semicircle = supervisor->getFromDef("plane");

  double radius = semicircle->getField("radius")->getSFFloat();
  const int num_divisions = 6;
  const double angle_step = M_PI / num_divisions;

  for (int i = 0; i < num_divisions; i++) {
    double angle = (i + 1) * angle_step;
    double x = radius * cos(angle);
    double z = radius * sin(angle);

    Marker* marker = supervisor->getWorld()->createMarker("marker");
    marker->setPosition(Vector3d(x, 0.01, z));
    marker->setAppearance(0xff0000); // set color to red
  }

  supervisor->step(0);
  delete supervisor;
  return 0;
}
