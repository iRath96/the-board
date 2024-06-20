constexpr uint32_t STEPS_PER_MM = 40;

uint32_t g_jerk         =   40; // mm/s
uint32_t g_maxVelocity  =  400; // mm/s
uint32_t g_acceleration = 4000; // mm/(s^2)

constexpr uint8_t PIN_X_STEP = 2;
constexpr uint8_t PIN_Y_STEP = 3;
constexpr uint8_t PIN_Z_STEP = 4;
constexpr uint8_t PIN_X_DIR = 5;
constexpr uint8_t PIN_Y_DIR = 6;
constexpr uint8_t PIN_Z_DIR = 7;
constexpr uint8_t PIN_STEPPER_DISABLE = 8;

int32_t g_positionX = 0; // 2x steps
int32_t g_positionY = 0; // 2x steps
int32_t g_maxPositionX = 1000 * STEPS_PER_MM;
int32_t g_maxPositionY = 700 * STEPS_PER_MM;

char buffer[128];

// #define DEBUG

template<uint8_t pin>
inline void digitalWriteFast(uint8_t x) {
  if constexpr (pin / 8) {
    PORTB ^= (-x ^ PORTB) & (1 << (pin % 8));
  } else {
    PORTD ^= (-x ^ PORTD) & (1 << (pin % 8));
  }
}

void linearSegment(int32_t xSteps, int32_t ySteps) {
  constexpr uint32_t TIME_PER_STEP = 85899; // 20 usec * 2^32

#ifdef DEBUG
  Serial.println();
  Serial.print("linearSegment(");
  Serial.print(xSteps); Serial.print(", ");
  Serial.print(ySteps); Serial.println(")");
  Serial.println();
#endif

  const bool xDir = xSteps < 0; // true if negative direction
  const bool yDir = ySteps < 0; // true if negative direction
  const uint32_t xpTarget = xDir ? -xSteps : xSteps;
  const uint32_t ypTarget = yDir ? -ySteps : ySteps;

  digitalWriteFast<PIN_X_DIR>(xDir);
  digitalWriteFast<PIN_Y_DIR>(yDir);

  const bool yDominant = ypTarget > xpTarget;

  const uint32_t helper = TIME_PER_STEP * STEPS_PER_MM / (yDominant ? ypTarget : xpTarget);
  const uint32_t baseV = helper * g_jerk;
  const uint32_t baseM = helper * g_maxVelocity;
  const uint32_t baseA = (uint64_t(helper * g_acceleration) * TIME_PER_STEP) >> 32;

  uint32_t xv = baseV * xpTarget;
  uint32_t xm = baseM * xpTarget;
  uint32_t xa = baseA * xpTarget;

  uint32_t yv = baseV * ypTarget;
  uint32_t ym = baseM * ypTarget;
  uint32_t ya = baseA * ypTarget;

  uint32_t xFrac = 0;
  uint32_t yFrac = 0;

  uint32_t xp = 0;
  uint32_t yp = 0;

  const auto segment = [&](int32_t xpTarget, int32_t ypTarget) {
#ifdef DEBUG
    Serial.print("xv "); Serial.print(xv);
    Serial.print(" xm "); Serial.print(xm);
    Serial.print(" xa "); Serial.print(xa);
    Serial.print(" xt "); Serial.print(xpTarget);
    Serial.println();
#endif

    uint32_t iter = 0;
#ifdef DEBUG
    auto mPrev = micros();
#endif
    while (xp < xpTarget || yp < ypTarget) {
      iter++;
      if (xv > xm || yv > ym) goto maxVelocity;

      digitalWriteFast<PIN_X_STEP>(0);
      xv += xa;
      const uint32_t xFrac0 = xFrac;
      xFrac += xv;
      uint8_t xStep = xFrac < xFrac0;
      xp += xStep;
      digitalWriteFast<PIN_X_STEP>(xStep);
      
      digitalWriteFast<PIN_Y_STEP>(0);
      yv += ya;
      const uint32_t yFrac0 = yFrac;
      yFrac += yv;
      uint8_t yStep = yFrac < yFrac0;
      yp += yStep;
      digitalWriteFast<PIN_Y_STEP>(yStep);
    }

#ifdef DEBUG
    auto timeTaken = micros() - mPrev;
    Serial.println("done updating");
    Serial.print("  time "); Serial.println(timeTaken);
    Serial.print("  iter "); Serial.println(iter);
    Serial.println();
#endif

    return true;

maxVelocity:
    // maximum velocity reached
#ifdef DEBUG
    auto timeTaken2 = micros() - mPrev;
    Serial.println("max velocity");
    Serial.print("  time "); Serial.println(timeTaken2);
    Serial.print("  iter "); Serial.println(iter);
    Serial.println();
#endif

    return false;
  };

  if (segment(xpTarget / 2, ypTarget / 2)) {
    // stopped because of midpoint, reverse acceleration
    xa = -xa;
    ya = -ya;
    segment(xpTarget, ypTarget);
  } else {
    // stopped because of max velocity
    const uint32_t xd = -xa;
    const uint32_t yd = -ya;

    // constant speed segment
    // use longer axis as target to avoid rounding issues
    xv = xm;
    yv = ym;
    xa = ya = 0;
    if (yDominant) {
      segment(0, ypTarget - yp);
    } else {
      segment(xpTarget - xp, 0);
    }

    // deceleration segment
    xa = xd;
    ya = yd;
    segment(xpTarget, ypTarget);
  }

#ifdef DEBUG
  Serial.println("achieved:");
  Serial.print("  xp "); Serial.print(xp);
  Serial.print("  yp "); Serial.print(yp);
  Serial.println();
#endif

  digitalWriteFast<PIN_X_STEP>(0);
  digitalWriteFast<PIN_Y_STEP>(0);

  const int32_t sxp = xDir ? -xp : xp;
  const int32_t syp = yDir ? -yp : yp;
  g_positionX -= sxp - syp;
  g_positionY -= sxp + syp;

#ifdef DEBUG
  Serial.print("now at ");
  Serial.print(g_positionX);
  Serial.print(", ");
  Serial.println(g_positionY);
#endif
}

void linearSegmentPlotter(int32_t ySteps, uint8_t numBytes) {
  constexpr uint32_t TIME_PER_STEP = 85899; // 20 usec * 2^32

  uint8_t *pixels = buffer;
  Serial.readBytes(pixels, numBytes);

  const bool yDir = ySteps < 0; // true if negative direction
  const uint32_t ypTarget = yDir ? -ySteps : ySteps;
  
  digitalWriteFast<PIN_X_DIR>(yDir);
  digitalWriteFast<PIN_Y_DIR>(yDir);

  uint32_t yv = g_jerk * TIME_PER_STEP * STEPS_PER_MM;
  uint32_t ym = g_maxVelocity * TIME_PER_STEP * STEPS_PER_MM;
  uint32_t ya = (uint64_t(TIME_PER_STEP * STEPS_PER_MM * uint32_t(g_acceleration)) * TIME_PER_STEP) >> 32;
  
  uint32_t yFrac = 0;
  uint32_t yp = 0;

  int8_t penState = 0;

  constexpr uint8_t base = 7;
  const auto segment = [&](int32_t ypTarget) {
    while (yp < ypTarget) {
      if (yv > ym) goto maxVelocity;

      const uint16_t pixelId = yp >> base;
      const uint8_t pixel = (pixels[pixelId >> 3] >> (pixelId & 7)) & 1;
      // const uint8_t pixel = 1;

      bool zStep = (yp & ((1 << (base - 4)) - 1)) == 0;
      bool zDir = ((yp >> (base - 1)) & 1) == 1;

      digitalWriteFast<PIN_Z_DIR>(zDir);

      digitalWriteFast<PIN_X_STEP>(0);
      digitalWriteFast<PIN_Y_STEP>(0);
      digitalWriteFast<PIN_Z_STEP>(0);
      yv += ya;
      const uint32_t yFrac0 = yFrac;
      yFrac += yv;
      uint8_t yStep = yFrac < yFrac0;
      yp += yStep;
      digitalWriteFast<PIN_X_STEP>(yStep);
      digitalWriteFast<PIN_Y_STEP>(yStep);
      zStep = pixel & yStep & zStep;
      digitalWriteFast<PIN_Z_STEP>(zStep);

      penState += zStep ? (zDir ? +1 : -1) : 0;
    }

    return true;

maxVelocity:
    return false;
  };

  if (segment(ypTarget / 2)) {
    // stopped because of midpoint, reverse acceleration
    ya = -ya;
    segment(ypTarget);
  } else {
    // stopped because of max velocity
    const uint32_t yd = -ya;

    // constant speed segment
    // use longer axis as target to avoid rounding issues  
    yv = ym;
    ya = 0;
    segment(ypTarget - yp);

    // deceleration segment
    ya = yd;
    segment(ypTarget);
  }

  digitalWriteFast<PIN_X_STEP>(0);
  digitalWriteFast<PIN_Y_STEP>(0);

  const int32_t syp = yDir ? -yp : yp;
  g_positionY -= 2 * syp;

  // reset pen

  digitalWriteFast<PIN_Z_DIR>(penState < 0);
  penState = abs(penState);
  for (uint8_t i = 0; i < penState; i++) {
    digitalWriteFast<PIN_Z_STEP>(0);
    delayMicroseconds(200);
    digitalWriteFast<PIN_Z_STEP>(1);
    delayMicroseconds(200);
  }
  digitalWriteFast<PIN_Z_STEP>(0);
}

void lineTo(int32_t xsteps, int32_t ysteps) {
  xsteps = max(0, min(xsteps, g_maxPositionX));
  ysteps = max(0, min(ysteps, g_maxPositionY));
  linearSegment(
    (int32_t(g_positionY) + int32_t(g_positionX)) / 2 - (xsteps + ysteps),
    (int32_t(g_positionY) - int32_t(g_positionX)) / 2 - (ysteps - xsteps)
  );
}

void lineToPlotter(int32_t ysteps, uint8_t numBytes) {
  ysteps = max(0, min(ysteps, g_maxPositionY));
  linearSegmentPlotter(int32_t(g_positionY) / 2 - ysteps, numBytes);
}

void wiggle(int repeats, int steps, int frequency) {
  if (steps >= 128) return;
  uint32_t table[128];

  const uint32_t totalDuration = uint32_t(1000) * 1000 / (2 * frequency); // in usec

  uint32_t totalExpected = 0;
  float prevX = 0;
  for (int step = 0; step < steps; step++) {
    const float y = (2 * (step + 1) - steps) / float(steps);
    const float x = asin(y) * M_1_PI + 0.5f;
    const float dx = x - prevX;
    prevX = x;

    uint32_t wait = round(totalDuration * dx);
    table[step] = wait <= 5 ? 1 : wait - 5;
    totalExpected += wait;
  }
  Serial.print("  computed, total: ");
  Serial.println(totalExpected);
  Serial.println(totalDuration);
  Serial.println(frequency);

  auto n = micros();
  repeats *= 2;
  for (int repeat = 0; repeat < repeats; repeat++) {
    digitalWriteFast<PIN_Z_DIR>(repeat & 1);

    for (int step = 0; step < steps; step++) {
      digitalWriteFast<PIN_Z_STEP>(0);
      
      uint32_t wait = table[step];
      delayMicroseconds(2);
      digitalWriteFast<PIN_Z_STEP>(1);

      delayMicroseconds(table[step]);
    }
  }
  auto t = micros() - n;
  Serial.print("  took ");
  Serial.print(t);
  Serial.println(" usec");

  digitalWriteFast<PIN_Z_STEP>(0);
}

void dragon(int depth, float x0, float y0, float x1, float y1, int sign = 1) {
  const float xC = (x0 + x1) / 2 + sign * (y1 - y0) / 2;
  const float yC = (y0 + y1) / 2 - sign * (x1 - x0) / 2;

  if (depth == 11) {
    lineTo(xC, yC);
    lineTo(x1, y1);
  } else {
    dragon(depth + 1, x0, y0, xC, yC, -1);
    dragon(depth + 1, xC, yC, x1, y1, +1);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_X_DIR, OUTPUT);
  pinMode(PIN_Y_DIR, OUTPUT);
  pinMode(PIN_Z_DIR, OUTPUT);
  pinMode(PIN_X_STEP, OUTPUT);
  pinMode(PIN_Y_STEP, OUTPUT);
  pinMode(PIN_Z_STEP, OUTPUT);
  pinMode(PIN_STEPPER_DISABLE, OUTPUT);

  // digitalWrite(PIN_STEPPER_DISABLE, true);
  // delay(1000);
  digitalWrite(PIN_STEPPER_DISABLE, false);

  Serial.println("starting");
  // linearSegment(1200, 4000);
  // Serial.println("done");
}

char parseCommand(const char *&str) {
  return *str++;
}

int32_t parseInt(const char *&str) {
  while (*str == ' ') str++;
  
  const bool negative = *str == '-';
  if (negative) str++;

  int32_t ret = 0;
  while (*str >= '0' && *str <= '9') {
    ret = ret * 10 + (*str++ - '0');
  }
  return negative ? -ret : ret;
}

int32_t parseSteps(const char *&str) {
  while (*str == ' ') str++;
  
  const bool negative = *str == '-';
  if (negative) str++;

  int32_t ret = 0;
  while (*str >= '0' && *str <= '9') {
    ret = ret * 10 + (*str++ - '0');
  }
  
  ret *= STEPS_PER_MM;

  if (*str == '.') {
    str++;
    int32_t frac = 0;
    int32_t denom = 1;
    while (*str >= '0' && *str <= '9') {
      if (denom < 1000) {
        frac = frac * 10 + (*str - '0');
        denom *= 10;
      }
      str++;
    }
    ret += (frac * STEPS_PER_MM) / denom;
  }

  return negative ? -ret : ret;
}

void loop() {
  int len = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
  if (len == 0) return;
  buffer[len] = 0;

  Serial.print("$ ");
  Serial.println(buffer);

  const char *index = buffer;
  switch (char cmd = parseCommand(index)) {
    case 'I': {
      Serial.println();
      Serial.println("Overview:");
      Serial.print("  Jerk:   "); Serial.print(g_jerk); Serial.println(" mm/s");
      Serial.print("  Max V.: "); Serial.print(g_maxVelocity); Serial.println(" mm/s");
      Serial.print("  Accel.: "); Serial.print(g_acceleration); Serial.println(" mm/(s^2)");
      Serial.print("  Pos. X: "); Serial.print(g_positionX / float(2 * STEPS_PER_MM)); Serial.println(" mm");
      Serial.print("  Pos. Y: "); Serial.print(g_positionY / float(2 * STEPS_PER_MM)); Serial.println(" mm");
      Serial.println();
      break;
    }
    case 'E': {
      digitalWrite(PIN_STEPPER_DISABLE, false);
      Serial.println("enabled");
      break;
    }
    case 'D': {
      digitalWrite(PIN_STEPPER_DISABLE, true);
      Serial.println("disabled");
      break;
    }
    case 'J': {
      g_jerk = parseInt(index);
      Serial.print("jerk now ");
      Serial.print(g_jerk);
      Serial.println(" mm/s");
      break;
    }
    case 'M': {
      g_maxVelocity = parseInt(index);
      Serial.print("max velocity now ");
      Serial.print(g_maxVelocity);
      Serial.println(" mm/s");
      break;
    }
    case 'A': {
      g_acceleration = parseInt(index);
      Serial.print("acceleration now ");
      Serial.print(g_acceleration);
      Serial.println(" mm/(s^2)");
      break;
    }
    case 'L': {
      int x = parseSteps(index);
      int y = parseSteps(index);
      linearSegment(x, y);
      break;
    }
    case 'W': {
      int repeats = parseInt(index);
      int steps = parseInt(index);
      int frequency = parseInt(index);
      wiggle(repeats, steps, frequency);
      break;
    }
    case 'G': {
      int32_t x = parseSteps(index);
      int32_t y = parseSteps(index);
      lineTo(x, y);
      Serial.println("ok");
      break;
    }
    case 'C': {
      const float centerX = parseSteps(index);
      const float centerY = parseSteps(index);
      const float radius = parseSteps(index);
      const uint32_t numSegments = parseInt(index);
      for (uint32_t i = 0; i <= numSegments; i++) {
        const float phi = (2 * M_PI) * i / float(numSegments);
        lineTo(centerX + radius * cos(phi), centerY + radius * sin(phi));
      }
      Serial.println("ok");
      break;
    }
    case 'F': {
      dragon(0, 300 * STEPS_PER_MM, 350 * STEPS_PER_MM, 700 * STEPS_PER_MM, 350 * STEPS_PER_MM);
      Serial.println("ok");
      break;
    }
    case 'R': { // grid
      uint32_t spacing = parseSteps(index);
      uint32_t xCount = g_maxPositionX / spacing;
      uint32_t yCount = g_maxPositionY / spacing;
      for (uint32_t x = 0; x < xCount; x += 2) {
        lineTo(x * spacing, 0);
        lineTo(x * spacing, g_maxPositionY);
        lineTo((x + 1) * spacing, g_maxPositionY);
        lineTo((x + 1) * spacing, 0);
      }
      for (uint32_t y = 0; y < yCount; y += 2) {
        lineTo(0, y * spacing);
        lineTo(g_maxPositionX, y * spacing);
        lineTo(g_maxPositionX, (y + 1) * spacing);
        lineTo(0, (y + 1) * spacing);
      }
      Serial.println("ok");
      break;
    }
    case 'T': {
      uint32_t repeats = parseInt(index);
      uint32_t delay = parseInt(index) / 4;
      for (uint32_t i = 0; i < repeats; i++) {
        digitalWriteFast<PIN_Z_DIR>(0);
        digitalWriteFast<PIN_Z_STEP>(1);
        delayMicroseconds(delay);
        digitalWriteFast<PIN_Z_STEP>(0);
        delayMicroseconds(delay);

        digitalWriteFast<PIN_Z_DIR>(1);
        digitalWriteFast<PIN_Z_STEP>(1);
        delayMicroseconds(delay);
        digitalWriteFast<PIN_Z_STEP>(0);
        delayMicroseconds(delay);
      }
      Serial.println("ok");
      break;
    }
    case 'P': {
      int32_t steps = parseInt(index);
      digitalWriteFast<PIN_Z_DIR>(steps > 0);
      steps = abs(steps);
      for (uint32_t step = 0; step < steps; step++) {
        digitalWriteFast<PIN_Z_STEP>(1);
        delayMicroseconds(500);
        digitalWriteFast<PIN_Z_STEP>(0);
        delayMicroseconds(500);
      }
      Serial.println("ok");
      break;
    }
    case 'O': {
      int32_t y = parseSteps(index);
      uint8_t numBytes = parseInt(index);
      lineToPlotter(y, numBytes);
      Serial.println("ok");
      break;
    }
    default: {
      Serial.print("unknown command '");
      Serial.print(cmd);
      Serial.println("'");
      break;
    }
  }
}
