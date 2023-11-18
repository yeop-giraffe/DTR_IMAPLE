#include <WiFi.h>
#include <Adafruit_MotorShield.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

const char* ssid = "ZhouLab";  // 여기에 무선 네트워크 이름 (SSID)을 입력하세요.
const char* password = "ZhouRobotics917";  // 여기에 네트워크 암호를 입력하세요.

int FRONT = 0;
int BACK = 0;
int L_TURN = 0;
int R_TURN = 0;
int UP = 0;
int DOWN = 0;

WiFiServer server(80);
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor* myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor* myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor* myMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor* myMotor4 = AFMS.getMotor(4);

AsyncWebServer webServer(80);

void setup() {
  Serial.begin(115200);
  delay(10);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  myMotor1->run(FORWARD);
  myMotor2->run(FORWARD);
  myMotor3->run(FORWARD);
  myMotor4->run(FORWARD);

  // Setup the HTTP server
  setupWebServer();

  // Start the server
  webServer.begin();
}

void loop() {
  // Handle HTTP requests
  // 이 부분은 현재 비어 있어도 됩니다.
}

void setupWebServer() {
  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    String direction = request->arg("direction");
    if (direction == "FRONT") {
      FRONT += 10;
    } else if (direction == "BACK") {
      BACK += 10;
    } else if (direction == "L_TURN") {
      L_TURN += 10;
    } else if (direction == "R_TURN") {
      R_TURN += 10;
    } else if (direction == "UP") {
      UP += 10;
    } else if (direction == "DOWN") {
      DOWN += 10;
    } else if (direction == "XROT") {
      L_TURN = 0;
      R_TURN = 0;
    }

    updateMotorSpeed();

    // HTML 코드 내에 방향키 텍스트 추가
    String html = "<html><body>";
    html += "<h1>ESP32 Web Server</h1>";
    html += "<h2>Direction Control</h2>";
    html += "<p>UP: " + String(UP) + "</p>";
    html += "<p>FRONT: " + String(FRONT) + "</p>";
    html += "<p>BACK: " + String(BACK) + "</p>";
    html += "<p>L_TURN: " + String(L_TURN) + "</p>";
    html += "<p>R_TURN: " + String(R_TURN) + "</p>";
    html += "<a href='/?direction=UP'><button>UP</button></a>";
    html += "<a href='/?direction=DOWN'><button>DOWN</button></a>";
    html += "<a href='/?direction=L_TURN'><button>L_TURN</button></a>";
    html += "<a href='/?direction=R_TURN'><button>R_TURN</button></a>";
    html += "<a href='/?direction=FRONT'><button>FRONT</button></a>";
    html += "<a href='/?direction=BACK'><button>BACK</button></a>";
    html += "<a href='/?direction=XROT'><button>XROT</button></a>";
    html += "</body></html>";

    // JavaScript 코드를 추가하여 키보드 이벤트를 감지하고 서버로 전송
    html += "<script>";
    html += "document.addEventListener('keydown', function(event) {";
    html += "  var key = event.key;";
    html += "  switch (key) {";
    html += "    case 'ArrowUp':";
    html += "      fetch('/?direction=FRONT');";
    html += "      break;";
    html += "    case 'ArrowDown':";
    html += "      fetch('/?direction=BACK');";
    html += "      break;";
    html += "    case 'ArrowLeft':";
    html += "      fetch('/?direction=L_TURN');";
    html += "      break;";
    html += "    case 'ArrowRight':";
    html += "      fetch('/?direction=R_TURN');";
    html += "      break;";
    html += "    case 'q':";
    html += "      fetch('/?direction=DOWN');";
    html += "      break;";
    html += "    case 'w':";
    html += "      fetch('/?direction=UP');";
    html += "      break;";
    html += "    case '/':";
    html += "      fetch('/?direction=XROT');";
    html += "      break;";
    html += "  }";
    html += "});";
    html += "</script>";

    request->send(200, "text/html", html);
  });
}

void updateMotorSpeed() {
  myMotor1->setSpeed(constrain(UP-DOWN-FRONT+BACK+L_TURN-R_TURN, 0, 255));
  myMotor2->setSpeed(constrain(UP-DOWN-FRONT+BACK-L_TURN+R_TURN, 0, 255));
  myMotor3->setSpeed(constrain(UP-DOWN+FRONT-BACK+L_TURN-R_TURN, 0, 255));
  myMotor4->setSpeed(constrain(UP-DOWN+FRONT-BACK-L_TURN+R_TURN, 0, 255));
}
