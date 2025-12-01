Russian translation available
[Russian](https://github.com/pruwait/Nice_BusT4)

Componente ESPHOME per il controllo dei camcelli/serrande automatiche Nice tramite protocollo Bus T4
Protocollo Nice Bus T4
Consente a ESP8266 di comunicare con gli apri-cancello/garage Nice utilizzando la porta Bus T4. 
La scheda di controllo Nice deve essere dotata di connettore BusT4.

# Attuali funzionalità
Invio di comandi: "Apri", "Stop", "Chiudi", "Apertura parziale", "Passo-passo (SBS)" e altri tramite pulsanti.
Invio di comandi HEX arbitrari tramite il servizio "raw_command". I separatori di byte possono essere punti o spazi. Esempio: 55 0c 00 03 00 81 01 05 86 01 82 01 64 e6 0c o 55.0D.00.FF.00.66.08.06.97.00.04.99.00.00.9D.0D
Formazione e invio di richieste GET/SET arbitrarie tramite il servizio "send_inf_command". Consente di configurare il dispositivo o ottenere il suo stato.
Visualizzazione dei pacchetti da tutti i dispositivi nella rete BusT4.

# Bus T4: Dettagli Tecnici
Si tratta di una UART modificata 19200 8n1 con una durata di uart break di 519ms-590ms prima di ogni input.
È possibile connettere diversi dispositivi; per questo, i ricetrasmettitori CAN-BUS sono aggiunti al livello fisico.
La trasmissione fisica avviene spesso tramite ricetrasmettitori CAN, ma non ci sono frame CAN.
![alt text](img/connector.jpg "BusT4 port with pinout")

BusT4 RX/TX operano a 5V.
ESP8266 RX/TX utilizzano 3.3V - è necessario utilizzare un convertitore di livello! 
Il pin VCC del BusT4 fornisce alimentazione con tensione compresa tra 24V e 28V.
Prestare estrema attenzione a seguire la corretta piedinatura - un cablaggio errato al BusT4 può immediatamente danneggiare la scheda di controllo Nice.
Verificare manualmente la tensione sulla porta VCC prima di connettersi all'unità!

![alt text](img/diagram.jpg "Example of connection with Wemos D1 mini, logic level shifter and DC-DC buck step down conventer")

# Esempi di Comandi
Il componente supporta l'invio di un comando arbitrario all'azionamento tramite il servizio ESPHome:
```
SBS:   55 0c 00 03 00 81 01 05 86 01 82 01 64 e6 0c
Open:  55 0c 00 03 05 81 01 05 83 01 82 03 64 e4 0c
Close: 55 0c 00 03 05 81 01 05 83 01 82 04 64 e3 0c
Stop:  55 0c 00 03 00 81 01 05 86 01 82 02 64 e5 0c
```
![alt text](img/IMG_20220113_160221.jpg "Внешний вид прототипа устройства")

Se sei interessato al progetto, puoi [offrire una birra o un caffè al creatore @pruwait](https://yoomoney.ru/to/4100117927279918)
La versione originale è disponibile su https://github.com/pruwait/Nice_BusT4, la maggior parte del merito va a @pruwait.

# Aggiornamenti
* Добавлены службы в интерфейс компонента для более простого запуска процедуры распознавания длины створки и процедуры распознавания устройств BlueBus не разбирая корпус привода (и даже находясь удалённо).
* Добавлен вывод в лог конфигурации считанные из устройства состояния L1, L2, L3 (Автоматическое
закрывание, Закрыть после
фотоэлемента, Всегда закрывать)
* Улучшена совместимость с приводами DPRO924
* Кнопка СТОП всегда доступна в User Interface объекта
* Улучшена совместимость с приводами Walky WL1024C
* Улучшена совместимость с приводами Spin ([@TheGoblinHero](https://github.com/TheGoblinHero))
* Добавлена функция задания произвольного положения привода ([@TheGoblinHero](https://github.com/TheGoblinHero))



