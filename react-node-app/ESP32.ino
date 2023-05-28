#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = "Rover27";
const char* password = "123456788";
const char* serverAddress = "18.134.98.192";  // Replace with the IP address or hostname of your Node.js server
const int serverPort = 3001;  // Replace with the port number your Node.js server is listening on

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
  // Example button direction

  
  // Create a JSON payload with the button click direction
  String coordinates = "some_coordinates";
  String jsonPayload = "{\"coordinates\":\"" + coordinates + "\"}";

  //GET AND POST ENDPOINTS SETUP =================================================
  HTTPClient httpPOST;
  HTTPClient httpGet;
  
  String PostEndpoint = "http://" + String(serverAddress) + ":" + String(serverPort) + "/roverCoordinatePost"
  httpPOST.begin(PostEndpoint);  // Specify the server address, port, and endpoint
  httpPOST.addHeader("Content-Type", "application/json");

  String GetEndpoint = "http://" + String(serverAddress) + ":" + String(serverPort) + "/numericalInputESP";
  httpGet.begin(GetEndpoint);  // Specify the server address and endpoint
  
  //POST Code =====================================================================
  int httpResponseCodePost = httpPOST.POST(jsonPayload);
  
  if (httpResponseCodePost > 0) {
    Serial.print("Button click request sent. Response code: ");
    Serial.println(httpResponseCodePost);
    
    String responseBody = httpPOST.getString();
    Serial.print("Response body: ");
    Serial.println(responseBody);
  } else {
    Serial.print("Error sending request. Error code: ");
    Serial.println(httpResponseCodePost);
  }

  //GET Code =======================================================================
  int httpResponseCodeGet = httpGet.GET();
      
  if (httpResponseCodeGet>0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCodeGet);
    String payload = httpGet.getString();
    Serial.println(payload);
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCodeGet);
  }

  // Free resources
  httpPOST.end();
  httpGet.end();

  delay(5000);  // Wait for 5 seconds before sending the next request
}