// Processing 4 sketch for the ESP32/BNO055 IMU tester.
import processing.serial.*;

Serial myPort;
PShape helmet;

final String SERIAL_PORT = "COM66";
final int SERIAL_BAUD = 115200;

String data = "";
float roll, pitch, yaw;

void setup() {
  size(2560, 1440, P3D);
  smooth(8);

  // This must match Serial.begin(115200) in the ESP32 firmware.
  myPort = new Serial(this, SERIAL_PORT, SERIAL_BAUD);
  myPort.clear();
  myPort.bufferUntil('\n');

  helmet = createHelmet();
}

void draw() {
  background(233);
  lights();
  ambientLight(90, 90, 90);
  directionalLight(190, 190, 180, -0.35, 0.55, -1.0);

  // Keep the readout fixed while the helmet follows the IMU.
  fill(25);
  textSize(22);
  textAlign(LEFT, TOP);
  text("Roll: " + nf(roll, 0, 1) +
       "     Pitch: " + nf(pitch, 0, 1) +
       "     Yaw: " + nf(yaw, 0, 1), 30, 30);

  pushMatrix();
  translate(width/2, height/2, 0);
  rotateX(radians(-pitch));
  rotateZ(radians(roll));
  rotateY(radians(yaw));
  shape(helmet);
  popMatrix();

  fill(60);
  textAlign(CENTER, BOTTOM);
  textSize(20);
  text("ACH / MICH-style IMU orientation", width/2, height - 30);
}

PShape createHelmet() {
  PShape assembly = createShape(GROUP);
  assembly.addChild(createHelmetShell());
  assembly.addChild(createHelmetRim());
  assembly.addChild(createSideRail(-1));
  assembly.addChild(createSideRail(1));
  assembly.addChild(createFrontShroud());
  assembly.addChild(createChinStraps());
  return assembly;
}

// Rounded ballistic shell with a lower side cut and a shorter front edge.
PShape createHelmetShell() {
  final int aroundSegments = 64;
  final int verticalSegments = 24;

  PShape shell = createShape();
  shell.beginShape(QUADS);
  shell.noStroke();
  shell.fill(92, 105, 67);

  for (int ring = 0; ring < verticalSegments; ring++) {
    float v0 = ring / float(verticalSegments);
    float v1 = (ring + 1) / float(verticalSegments);

    for (int segment = 0; segment < aroundSegments; segment++) {
      float theta0 = TWO_PI * segment / aroundSegments;
      float theta1 = TWO_PI * (segment + 1) / aroundSegments;

      addShellVertex(shell, v0, theta0);
      addShellVertex(shell, v0, theta1);
      addShellVertex(shell, v1, theta1);
      addShellVertex(shell, v1, theta0);
    }
  }

  shell.endShape();
  return shell;
}

void addShellVertex(PShape shell, float verticalPosition, float theta) {
  float phi = verticalPosition * lowerShellAngle(theta);
  PVector point = shellPoint(phi, theta);

  // Ellipsoid normal for smooth lighting.
  float zRadius = sin(theta) >= 0 ? 158 : 170;
  PVector normal = new PVector(
    point.x / sq(188),
    (point.y + 15) / sq(150),
    point.z / sq(zRadius)
  );
  normal.normalize();

  shell.normal(normal.x, normal.y, normal.z);
  shell.vertex(point.x, point.y, point.z);
}

float lowerShellAngle(float theta) {
  // The sides extend farther down than the brow; the rear is slightly deeper.
  return radians(101) +
         radians(15) * abs(cos(theta)) +
         radians(4) * max(0, -sin(theta));
}

PVector shellPoint(float phi, float theta) {
  float zRadius = sin(theta) >= 0 ? 158 : 170;
  return new PVector(
    188 * sin(phi) * cos(theta),
    -150 * cos(phi) - 15,
    zRadius * sin(phi) * sin(theta)
  );
}

// A dark edge band makes the open lower edge read as a padded helmet rim.
PShape createHelmetRim() {
  final int segments = 64;
  PShape rim = createShape();
  rim.beginShape(QUAD_STRIP);
  rim.noStroke();
  rim.fill(48, 55, 40);

  for (int segment = 0; segment <= segments; segment++) {
    float theta = TWO_PI * segment / segments;
    PVector outer = shellPoint(lowerShellAngle(theta), theta);
    PVector inner = new PVector(outer.x * 0.955, outer.y - 7, outer.z * 0.955);
    rim.vertex(outer.x, outer.y, outer.z);
    rim.vertex(inner.x, inner.y, inner.z);
  }

  rim.endShape();
  return rim;
}

// Low-profile accessory rail on either side of the shell.
PShape createSideRail(int side) {
  PShape railGroup = createShape(GROUP);

  PShape rail = createShape(BOX, 10, 50, 100);
  rail.setFill(color(50, 55, 42));
  rail.setStroke(false);
  rail.translate(side * 181, 8, 2);
  rail.rotateX(radians(-8));
  railGroup.addChild(rail);

  PShape slotFront = createShape(BOX, 12, 12, 25);
  slotFront.setFill(color(25, 28, 23));
  slotFront.setStroke(false);
  slotFront.translate(side * 187, 7, 29);
  railGroup.addChild(slotFront);

  PShape slotRear = createShape(BOX, 12, 12, 25);
  slotRear.setFill(color(25, 28, 23));
  slotRear.setStroke(false);
  slotRear.translate(side * 187, 19, -23);
  railGroup.addChild(slotRear);

  return railGroup;
}

// Front plate suggesting the common ACH/MICH NVG mounting shroud.
PShape createFrontShroud() {
  PShape shroudGroup = createShape(GROUP);

  PShape plate = createShape(BOX, 86, 66, 12);
  plate.setFill(color(42, 46, 37));
  plate.setStroke(false);
  plate.translate(0, -46, 153);
  shroudGroup.addChild(plate);

  PShape socket = createShape(BOX, 28, 30, 10);
  socket.setFill(color(18, 20, 18));
  socket.setStroke(false);
  socket.translate(0, -39, 163);
  shroudGroup.addChild(socket);

  PShape topScrew = createShape(SPHERE, 6);
  topScrew.setFill(color(18));
  topScrew.setStroke(false);
  topScrew.translate(0, -70, 162);
  shroudGroup.addChild(topScrew);

  PShape leftScrew = createShape(SPHERE, 6);
  leftScrew.setFill(color(18));
  leftScrew.setStroke(false);
  leftScrew.translate(-31, -29, 162);
  shroudGroup.addChild(leftScrew);

  PShape rightScrew = createShape(SPHERE, 6);
  rightScrew.setFill(color(18));
  rightScrew.setStroke(false);
  rightScrew.translate(31, -29, 162);
  shroudGroup.addChild(rightScrew);

  return shroudGroup;
}

PShape createChinStraps() {
  PShape straps = createShape();
  straps.beginShape(LINES);
  straps.stroke(46, 48, 37);
  straps.strokeWeight(8);
  straps.noFill();

  straps.vertex(-157, 45, 45);
  straps.vertex(-62, 167, 64);
  straps.vertex(-62, 167, 64);
  straps.vertex(0, 187, 74);

  straps.vertex(157, 45, 45);
  straps.vertex(62, 167, 64);
  straps.vertex(62, 167, 64);
  straps.vertex(0, 187, 74);

  straps.endShape();
  return straps;
}

// Read one firmware packet in the form: roll/pitch/yaw\n
void serialEvent(Serial port) {
  String line = port.readStringUntil('\n');
  if (line == null) {
    return;
  }

  data = trim(line);
  String[] items = split(data, '/');

  // Ignore firmware status/error messages and incomplete packets.
  if (items.length != 3) {
    println("Ignored serial line: " + data);
    return;
  }

  float nextRoll = parseFloat(trim(items[0]));
  float nextPitch = parseFloat(trim(items[1]));
  float nextYaw = parseFloat(trim(items[2]));

  // Only replace the displayed orientation with a complete numeric packet.
  if (!Float.isNaN(nextRoll) &&
      !Float.isNaN(nextPitch) &&
      !Float.isNaN(nextYaw)) {
    roll = nextRoll;
    pitch = nextPitch;
    yaw = nextYaw;
  } else {
    println("Ignored malformed AHRS packet: " + data);
  }
}
