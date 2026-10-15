# Test cablaggio tastiera ESP32

Sketch diagnostico separato dal firmware della bilancia. Identifica la coppia di GPIO chiusa da ciascun tasto, senza conoscere righe, colonne o numerazione del connettore.

## Collegamenti e funzionamento

I sei fili restano collegati a **TX2 / GPIO17, GPIO5, GPIO13, GPIO14, GPIO19 e GPIO21**. TX2 è GPIO17, distinto da TX0 / GPIO1 usato dalla seriale USB. Le sei linee della tastiera non richiedono collegamenti diretti a 3V3, 5 V o GND.

La scansione tiene un solo GPIO alla volta come uscita LOW; gli altri sono ingressi con pull-up. Non pilota uscite HIGH e non presume una disposizione 4x2 dei contatti. Usa attesa non bloccante di 200 µs e debounce di 40 ms. Stampa una coppia per pressione stabile, `RELEASE` al rilascio e un avviso se rileva più contatti: premere un solo tasto alla volta.

Il test usa soltanto i sei GPIO e la seriale USB a **115200 baud**. Display, pesatura, audio, WiFi, MQTT e protezioni batteria del firmware operativo non sono attivi. Non accede a NVS e non modifica calibrazione o credenziali.

## Compilazione e caricamento

Verifica dalla root del repository:

```powershell
& "C:\Users\shado\arduino-cli\arduino-cli.exe" compile --fqbn esp32:esp32:esp32 "D:\xampp\htdocs\bench-scale\minu-bench-scale\firmware\keypad_wiring_test"
```

Caricamento USB eseguito localmente da Andrea, con monitor seriale chiuso e la porta effettiva della scheda (COM3 nell'esempio):

```powershell
& "C:\Users\shado\arduino-cli\arduino-cli.exe" compile --fqbn esp32:esp32:esp32 -u -p COM3 "D:\xampp\htdocs\bench-scale\minu-bench-scale\firmware\keypad_wiring_test"
```

Usare il normale upload senza cancellazione completa della flash. Lo sketch di test sostituisce temporaneamente il programma operativo sulla scheda; al termine va ricaricato `firmware/esp32_hx711_serial` con la mappa dei tasti verificata.

## Rilevazione degli otto tasti

1. Aprire il monitor seriale a **115200 baud**; premere RESET sulla scheda se serve rileggere il banner.
2. Premere un tasto, mantenerlo almeno mezzo secondo e rilasciarlo prima del successivo.
3. Annotare l'etichetta fisica del tasto insieme alla coppia stampata, per esempio `GPIO17 <-> GPIO19`. La coppia non ha un verso: l'ordine non identifica riga o colonna.
4. Per una replica, raccogliere tutti gli otto risultati e confrontarli con la mappa confermata sotto prima di rimappare il firmware operativo. Se un tasto non stampa nulla o compare l'avviso di contatti multipli, annotarlo insieme al suo nome.

Mappa confermata del montaggio corrente:

```text
TARE  = GPIO5  <-> GPIO21
ENTER = GPIO17 <-> GPIO14
WIFI  = GPIO17 <-> GPIO19
SLEEP = GPIO5  <-> GPIO13
SKIP  = GPIO17 <-> GPIO13
TOTAL = GPIO5  <-> GPIO19
CLEAR = GPIO5  <-> GPIO14
MODE  = GPIO17 <-> GPIO21
```

La mappa definisce il cablaggio reale già usato dal firmware operativo: righe scansione GPIO13, 14, 19, 21 e colonne GPIO17, 5. La [mappa operativa](../../docs/WIRING.md#5-tastiera-4x2--esp32) raggruppa i tasti per linea GPIO; le sigle R1–R6 del ritaglio non sono l'ordine fisico del connettore e non sono necessarie per questo test.

## Verifica locale dello scanner

L'harness `tests/firmware_keypad_wiring_test_harness.cpp` compila lo sketch reale con GPIO e seriale simulati. Copre tutte le 15 coppie di contatti, le 105 combinazioni di due contatti, debounce, mantenimento, rilascio e rollover dei timer; verifica a ogni transizione che esista al massimo un'uscita, sempre LOW.

Nel Developer Command Prompt MSVC, dalla root del repository:

```bat
if not exist "%TEMP%\minu-keypad-wiring-harness" mkdir "%TEMP%\minu-keypad-wiring-harness"
cl /nologo /std:c++17 /EHsc /W4 /WX /O2 /I tests\keypad_wiring_stubs tests\firmware_keypad_wiring_test_harness.cpp /Fe:"%TEMP%\minu-keypad-wiring-harness\firmware_keypad_wiring_test_harness.exe" /Fo:"%TEMP%\minu-keypad-wiring-harness/"
"%TEMP%\minu-keypad-wiring-harness\firmware_keypad_wiring_test_harness.exe"
```
