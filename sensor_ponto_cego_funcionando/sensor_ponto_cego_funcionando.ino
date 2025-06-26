// TÓPICO MQTT USADO PARA PUBLICAR A MENSAGEM:
const char* mqtt_topic = "pontocego/alerta";

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RTClib.h>  // Biblioteca do RTC

#define TRIGGER_PIN 32
#define ECHO_PIN 33
#define BOTAO_SENSOR_PIN 19
#define BOTAO_SIMULADO_PIN 18
#define BUZZER_PIN 14

#define LED_VERMELHO 25
#define LED_AMARELO 26
#define LED_VERDE 27

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
RTC_DS1307 rtc;

unsigned long distanciaSimulada = 0;
unsigned long tempoInicial = 0;
bool mensagemEnviada = false;  // <- Variável para evitar envio repetido

// Aqui você deve incluir a configuração WiFi, MQTT, etc. (conexão HiveMQ)
// Exemplo básico, coloque as variáveis e funções para conectar no HiveMQ aqui
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// Configurações WiFi e HiveMQ (exemplo, troque pelos seus dados)
const char* ssid = "Nome da rede";
const char* password = "senha";

const char* mqtt_server = "d57895a0f2c543ec80dda6bc92a00625.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_user = "Pedro";
const char* mqtt_pass = "ESP322025i";

WiFiClientSecure espClient;
PubSubClient client(espClient);

void setup_wifi() {
  Serial.println();
  Serial.print("Conectando ao WiFi: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi conectado");
  Serial.println("Endereço IP: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Tentando conexão MQTT...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_pass)) {
      Serial.println("Conectado ao MQTT!");
    } else {
      Serial.print("Falhou, rc=");
      Serial.print(client.state());
      Serial.println(" Tentando novamente em 2 segundos");
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BOTAO_SENSOR_PIN, INPUT);
  pinMode(BOTAO_SIMULADO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(LED_VERMELHO, OUTPUT);
  pinMode(LED_AMARELO, OUTPUT);
  pinMode(LED_VERDE, OUTPUT);
  digitalWrite(LED_VERMELHO, LOW);
  digitalWrite(LED_AMARELO, LOW);
  digitalWrite(LED_VERDE, HIGH); // Começa aceso

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Display não encontrado!");
    while (true);
  }

  if (!rtc.begin()) {
    Serial.println("RTC não encontrado!");
    while (true);
  }

  if (!rtc.isrunning()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  tempoInicial = rtc.now().unixtime();

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  mostrarDistancia(distanciaSimulada, 60);

  // Conecta WiFi e MQTT
  setup_wifi();
  espClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
}

void mostrarDistancia(unsigned long km, int tempoRestante) {
  display.clearDisplay();

  DateTime now = rtc.now();

  char buffer[21];
  snprintf(buffer, sizeof(buffer), "%02d/%02d/%04d", now.day(), now.month(), now.year());
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(buffer);

  snprintf(buffer, sizeof(buffer), "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  display.setCursor(0, 10);
  display.print(buffer);

  display.setTextSize(2);
  display.setCursor(0, 25);
  display.print(km);
  display.println(" km");

  display.setTextSize(1);
  display.setCursor(0, 55);
  display.print("Tempo: ");
  display.print(tempoRestante);
  display.println(" s");

  display.display();
}

unsigned long medirDistanciaUltrassonico() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  long duracao = pulseIn(ECHO_PIN, HIGH, 30000);
  unsigned long distancia = duracao * 0.034 / 2;
  return distancia;
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  int tempoTotal = 60;
  unsigned long tempoAgora = rtc.now().unixtime();
  int tempoRestante = tempoTotal - (tempoAgora - tempoInicial);

  if (tempoRestante <= 0) {
    tempoInicial = tempoAgora;
    tempoRestante = tempoTotal;
  }

  if (tempoRestante <= 10) {
    digitalWrite(LED_VERMELHO, HIGH);
    digitalWrite(LED_AMARELO, LOW);
    digitalWrite(LED_VERDE, LOW);
  } else if (tempoRestante <= 20) {
    digitalWrite(LED_VERMELHO, LOW);
    digitalWrite(LED_AMARELO, HIGH);
    digitalWrite(LED_VERDE, LOW);
  } else {
    digitalWrite(LED_VERMELHO, LOW);
    digitalWrite(LED_AMARELO, LOW);
    digitalWrite(LED_VERDE, HIGH);
  }

  static bool botaoSimuladoAnt = LOW;
  bool botaoSimuladoAtual = digitalRead(BOTAO_SIMULADO_PIN);

  if (botaoSimuladoAnt == LOW && botaoSimuladoAtual == HIGH) {
    distanciaSimulada += 10;
    mostrarDistancia(distanciaSimulada, tempoRestante);
    Serial.print("Distância simulada: ");
    Serial.println(distanciaSimulada);
    delay(200);
  }
  botaoSimuladoAnt = botaoSimuladoAtual;

  bool botaoSensorPressionado = digitalRead(BOTAO_SENSOR_PIN) == HIGH;

  if (botaoSensorPressionado) {
    unsigned long distanciaReal = medirDistanciaUltrassonico();
    Serial.print("Distância real: ");
    Serial.print(distanciaReal);
    Serial.println(" cm");

    if (distanciaReal > 0 && distanciaReal <= 15) {
      digitalWrite(BUZZER_PIN, HIGH);

      if (!mensagemEnviada) {
        // Monta a mensagem JSON com alerta, data, hora, distância e tempo restante
        DateTime now = rtc.now();
        char mensagem[200];
        snprintf(mensagem, sizeof(mensagem),
          "{\"alerta\":\"Alerta de Ponto Cego\", \"data\":\"%02d/%02d/%04d\", \"hora\":\"%02d:%02d:%02d\", \"distancia\":%lu, \"tempo_descanso\":%d}",
          now.day(), now.month(), now.year(),
          now.hour(), now.minute(), now.second(),
          distanciaReal, tempoRestante);

        if (client.connected()) {
          client.publish(mqtt_topic, mensagem);
          Serial.print("Mensagem publicada no MQTT: ");
          Serial.println(mensagem);
        }

        mensagemEnviada = true;  // Evita novos envios enquanto o botão estiver pressionado
      }

    } else {
      digitalWrite(BUZZER_PIN, LOW);
    }

  } else {
    digitalWrite(BUZZER_PIN, LOW);
    mensagemEnviada = false;  // Libera envio quando o botão for solto
  }

  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 1000) {
    mostrarDistancia(distanciaSimulada, tempoRestante);
    lastUpdate = millis();
  }

  delay(50);
}

/*
  =======================
  RELAÇÃO DE PINOS - PROJETO TACÓGRAFO + PONTO CEGO
  =======================

  >> SENSOR ULTRASSÔNICO (HC-SR04)
    - TRIGGER_PIN: GPIO 32
    - ECHO_PIN:    GPIO 33
    - Alimentação: 5V
    - Observação: O pino ECHO do sensor passa por divisor de tensão feito com resistores de 1kΩ e 2.2kΩ
      para reduzir o sinal de 5V para 3.3V compatível com o ESP32.

  >> BOTÕES
    - BOTAO_SENSOR_PIN (botão que ativa o sensor):    GPIO 19
    - BOTAO_SIMULADO_PIN (botão para simular km):     GPIO 18

  >> BUZZER (Alarme sonoro)
    - BUZZER_PIN: GPIO 14

  >> LEDS DE INDICAÇÃO DE TEMPO DE DESCANSO
    - LED_VERMELHO: GPIO 25 (Alerta para parar)
    - LED_AMARELO:  GPIO 26 (Atenção - parar em 10 min)
    - LED_VERDE:    GPIO 27 (Pode dirigir)

  >> DISPLAY OLED (SSD1306 via I2C)
    - SDA: GPIO 21 (padrão I2C do ESP32)
    - SCL: GPIO 22 (padrão I2C do ESP32)
    - Alimentação: 3.3V

  >> RTC DS1307 (Relógio de tempo real - também via I2C)
    - SDA: GPIO 21 (compartilha com o display OLED)
    - SCL: GPIO 22 (compartilha com o display OLED)
    - Alimentação: 3.3V

  >> WiFi / MQTT:
    - Usa conexão WiFi e publica no tópico MQTT:
      "pontocego/alerta"
*/
