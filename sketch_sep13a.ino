int relayPin = 8;  
bool relayActive = false;   
unsigned long relayTurnedOnAt = 0;  
unsigned long relayOnDuration = 5000;  


String command = "";

void setup() {
  
  pinMode(relayPin, OUTPUT);
  
  
  digitalWrite(relayPin, HIGH); 

  
  Serial.begin(9600);
  Serial.println("Sistem Siap. Menunggu perintah...");
}

void loop() {
  
  if (Serial.available() > 0) {
    command = Serial.readStringUntil('\n');  
    command.trim();  )

    
    if (command == "ON" && !relayActive) {
      digitalWrite(relayPin, LOW);           
      relayTurnedOnAt = millis();            
      relayActive = true;                   
      Serial.println("Relay menyala selama 5 detik.");  
    }
    
    
    if (command == "OFF") {
      digitalWrite(relayPin, HIGH);  
      Serial.println("Relay dimatikan secara manual.");
      relayActive = false;  
    }

    command = "";  
  }

  
  if (relayActive && (millis() - relayTurnedOnAt >= relayOnDuration)) {
    digitalWrite(relayPin, HIGH);  
    Serial.println("Relay dimatikan otomatis setelah 5 detik.");
    relayActive = false;  
  }
}
