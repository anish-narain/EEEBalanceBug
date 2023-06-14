#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = "Rover27";
const char* password = "123456788";
const char* serverAddress = "18.134.98.192";  // Replace with the IP address or hostname of your Node.js server
const int serverPort = 3001;                  // Replace with the port number your Node.js server is listening on

// Variable declarations for traversal algorithm
int coordinates[2] = { 10, 20 };
int start_coordinates[2];
int current_coordinates[2];
String left_following;
String wall_detection = "false";
String maze_complete = "false";
String mvmt_direction;

unsigned long previousTime = 0;
const unsigned long updateInterval = 2;  // Time in milliseconds between coordinate updates

// Function declarations for traversal algorithm
void roverLeftFollow();

void setup() {
  Serial.begin(9600);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  while (maze_complete != "true") {
    // GET AND POST HTTP client setup =================================================
    HTTPClient httpPOST_roverCoordinates;
    HTTPClient httpPOST_wallDetection;
    HTTPClient httpGET_newWall;
    HTTPClient httpGET_nextDirection;
    HTTPClient httpGET_mvmtStop;
    HTTPClient httpGET_mazeComplete;

    // POST ENDPOINTS Setup ==========================================================
    current_coordinates[0] = coordinates[0];
    current_coordinates[1] = coordinates[1];

    String PostEndpoint_roverCoordinates = "http://" + String(serverAddress) + ":" + String(serverPort) + "/roverCoordinatePost";
    httpPOST_roverCoordinates.begin(PostEndpoint_roverCoordinates);  // Specify the server address, port, and endpoint
    httpPOST_roverCoordinates.addHeader("Content-Type", "application/json");
    String jsonPayload_roverCoordinates = "{\"received_coordinates\":[" + String(current_coordinates[0]) + ", " + String(current_coordinates[1]) + "]}";

    String PostEndpoint_wallDetection = "http://" + String(serverAddress) + ":" + String(serverPort) + "/wallDetection";
    httpPOST_wallDetection.begin(PostEndpoint_wallDetection);  // Specify the server address, port, and endpoint
    httpPOST_wallDetection.addHeader("Content-Type", "application/json");
    String jsonPayload_wallDetection = "{\"received_walldetection\":" + wall_detection + "}";

    // GET ENDPOINTS Setup ============================================================
    String GetEndpoint_newWall = "http://" + String(serverAddress) + ":" + String(serverPort) + "/newWall";
    httpGET_newWall.begin(GetEndpoint_newWall);  // Specify the server address and endpoint

    String GetEndpoint_nextDirection = "http://" + String(serverAddress) + ":" + String(serverPort) + "/nextDirection";
    httpGET_nextDirection.begin(GetEndpoint_nextDirection);  // Specify the server address and endpoint

    String GetEndpoint_mvmtStop = "http://" + String(serverAddress) + ":" + String(serverPort) + "/mvmtStop";
    httpGET_mvmtStop.begin(GetEndpoint_mvmtStop);  // Specify the server address and endpoint

    String GetEndpoint_mazeComplete = "http://" + String(serverAddress) + ":" + String(serverPort) + "/mazeComplete";
    httpGET_mazeComplete.begin(GetEndpoint_mazeComplete);  // Specify the server address and endpoint

    // Coordinate Update ===============================================================
    unsigned long currentTime = millis();
    if (currentTime - previousTime >= updateInterval) {
      // Update coordinates with your own coordinate calculation logic
      coordinates[0]++;  // Dummy logic, replace with your own coordinate calculation
      coordinates[1]++;  // Dummy logic, replace with your own coordinate calculation
      Serial.println(coordinates[0]);
      Serial.println(coordinates[1]);
      previousTime = currentTime;
    }

    // POST requests ==================================================================
    int httpResponseCodePost_roverCoordinates = httpPOST_roverCoordinates.POST(jsonPayload_roverCoordinates);
    if (httpResponseCodePost_roverCoordinates > 0) {
      Serial.print("Button click request sent. Response code for roverCoordinates : ");
      Serial.println(httpResponseCodePost_roverCoordinates);

      String responseBody_roverCoordinates = httpPOST_roverCoordinates.getString();
      Serial.print("Response body for roverCoordinates: ");
      Serial.println(responseBody_roverCoordinates);
    } else {
      Serial.print("Error sending request for rover coordinates. Error code: ");
      Serial.println(httpResponseCodePost_roverCoordinates);
    }

    int httpResponseCodePost_wallDetection = httpPOST_wallDetection.POST(jsonPayload_wallDetection);
    if (httpResponseCodePost_wallDetection > 0) {
      Serial.print("Button click request sent. Response code for wallDetection : ");
      Serial.println(httpResponseCodePost_wallDetection);

      String responseBody_wallDetection = httpPOST_wallDetection.getString();
      Serial.print("Response body for wallDetection: ");
      Serial.println(responseBody_wallDetection);
    } else {
      Serial.print("Error sending request for wall detection. Error code: ");
      Serial.println(httpResponseCodePost_wallDetection);
    }

    // GET requests ===================================================================
    if (wall_detection == "true") {
      int httpResponseCodeGet_newWall = httpGET_newWall.GET();
      if (httpResponseCodeGet_newWall > 0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCodeGet_newWall);
        left_following = httpGET_newWall.getString();
        Serial.println(left_following);
      } else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCodeGet_newWall);
      }
    }

    if (left_following == "true") {
      start_coordinates[0] = current_coordinates[0];
      start_coordinates[1] = current_coordinates[1];
      delay(100);  // Let the current coordinates change
      while (start_coordinates[0] != current_coordinates[0] || start_coordinates[1] != current_coordinates[1]) {
        roverLeftFollow();
      }
      left_following = "false";
    } else {
      int httpResponseCodeGet_nextDirection = httpGET_nextDirection.GET();
      if (httpResponseCodeGet_nextDirection > 0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCodeGet_nextDirection);
        mvmt_direction = httpGET_nextDirection.getString();
        Serial.println(mvmt_direction);
      } else {
        Serial.print("Error code for next direction: ");
        Serial.println(httpResponseCodeGet_nextDirection);
      }
    }

    // Free resources
    httpPOST_roverCoordinates.end();
    httpPOST_wallDetection.end();
    httpGET_newWall.end();
    httpGET_nextDirection.end();
    httpGET_mvmtStop.end();
    httpGET_mazeComplete.end();

    delay(5000);  // Wait for 5 seconds before sending the next request
  }
}

void roverLeftFollow() {
  Serial.print("Rover is doing left follow algorithm");
}
