#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("Adafruit MLX90614 test");

  if (!mlx.begin()) {
    Serial.println("Error connecting to MLX sensor. Check wiring.");
    while (1);
  };

  Serial.print("Emissivity = "); Serial.println(mlx.readEmissivity());
  Serial.println("================================================");
}

void loop() {
  float ambientTotal = 0;
  float objectTotal = 0;

  for (int i = 0; i < 10; i++) {
    ambientTotal += mlx.readAmbientTempC();
    objectTotal += mlx.readObjectTempC();
    delay(100); // 임의의 딜레이 추가
  }

  float ambientAvg = ambientTotal / 10.0;
  float objectAvg = objectTotal / 10.0;

  Serial.print("Ambient (Avg) = "); Serial.print(ambientAvg);
  Serial.print("*C\tObject (Avg) = "); Serial.print(objectAvg); Serial.println("*C");
  Serial.print("Ambient (Avg) = "); Serial.print(mlx.readAmbientTempF());
  Serial.print("*F\tObject (Avg) = "); Serial.print(mlx.readObjectTempF()); Serial.println("*F");

  Serial.println();
  delay(500);
}
