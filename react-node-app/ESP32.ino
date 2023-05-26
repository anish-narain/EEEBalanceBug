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

int httpResponseCode = -5;

void loop() {
  // Example button direction
  String direction = "right";
  
  // Create a JSON payload with the button click direction
  String jsonPayload = "{\"direction\":\"" + direction + "\"}";

  HTTPClient http;
  http.begin("http://" + String(serverAddress) + ":" + String(serverPort) + "/roverCoordinatePost");  // Specify the server address, port, and endpoint
  http.addHeader("Content-Type", "application/json");
  
  httpResponseCode = http.POST(jsonPayload);
  
  if (httpResponseCode > 0) {
    Serial.print("Button click request sent. Response code: ");
    Serial.println(httpResponseCode);
    
    String responseBody = http.getString();
    Serial.print("Response body: ");
    Serial.println(responseBody);
  } else {
    Serial.print("Error sending request. Error code: ");
    Serial.println(httpResponseCode);
  }

 // int httpResponseCode2 = http.GET();
 //     
 // if (httpResponseCode2>0) {
 //   Serial.print("HTTP Response code: ");
 //   Serial.println(httpResponseCode2);
 //   String payload = http.getString();
 //   Serial.println(payload);
 // }
 // else {
 //   Serial.print("Error code: ");
 //   Serial.println(httpResponseCode2);
 // }
  // Free resources
  http.end();

  delay(5000);  // Wait for 5 seconds before sending the next request
}