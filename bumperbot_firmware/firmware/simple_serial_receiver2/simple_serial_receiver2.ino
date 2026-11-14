//Chương trình chạy trên Arduino, nhận dữ liệu Serial và điều khiển LED dựa trên dữ liệu nhận được.
#define LED_PIN 13


void setup() {
  // put your setup code here, to run once:
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()) // Hàm Serial.available() kiểm tra xem có bao nhiêu byte dữ liệu đang chờ được đọc trong bộ đệm Serial.
  {
    /// turn on 
    int x = Serial.readString().toInt(); // Serial.readString(): Đọc dữ liệu Serial thành một đối tượng String.
                                         // toInt(): Chuyển đổi đối tượng String thành int
    if(x == 0){
    digitalWrite(LED_PIN, LOW);
  } // turn off
  else{
    digitalWrite(LED_PIN, HIGH);
  }
}
}