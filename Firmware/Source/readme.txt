/**
  @page Appunti
  
  @verbatim
  @endverbatim

@par Sensori

\todo
     Creare un task di lettura dei MEMS che accoda un messaggio al task attitude 
     con i valori dei sensori. In questo modo, il task di lettura dei MEMS � pi� 
     facilmente sostituibile dal task di telemetria che pu� inviare i valori di 
     sensori al suo posto.

     22/09/13
     Trovato dove si blocca il micro e lo scheduler di FreeRTOS.
     Riga 756 di cpal_i2c.c, chiamata alla macro:

     __CPAL_I2C_TIMEOUT(__CPAL_I2C_HAL_GET_SB(pDevInitStruct->CPAL_Dev), CPAL_I2C_TIMEOUT_SB);

     Lo stack di chiamata �:

       Attitude_Task()
       L3G4200_Init()
       SetODR()
       I2CMEMS_Read_Reg()
       CPAL_I2C_IsDeviceReady()


@par Simulazione

     Impostati i seguenti valori: 
        Roll_Kp = 1.0, Roll_Ki = 0.2, 
        Pitch_Kp = 1.0, Pitch_Ki = 1.0
        Dir_Kp = 5.0, Dir_Ki = 0.05

     Risultato: 
     raggiunge tutti i waypoint #1 - #6, non riesce a tornare al waypoint #1
     perch� non corregge subito l'errore di direzione quando l'aereo � ancora
     distante dal waypoint e poi � troppo tardi per correggerlo.

     Alzato Dir_Kp al massimo

     Risultato:
     Dopo il waypoint #6 la virata � appena accennata, raggiunta la direzione 
     del waypoint mantiene la direzione con un po' di errore.
     Avvicinandosi al waypoint #1 la correzione di direzione � in ritardo, 
     manca il waypoint e ci orbita attorno.
     Oscillazioni durante le virate di navigazione dovute al termine integrale
     del PID di rollio, Roll_Ki, non al termine integrale del PID di navigazione.

     Abbassato Roll_Kpi 

     Risultato:
     le oscillazioni spariscono.


@par Log

     01/09/13
     Abilitato task di log con funzionamento dipendente dalla posizione del 
     selettore MODE all'accensione del RC:

     MODE = MAN : salva il contenuto grezzo del buffer dell'UART del GPS
     MODE = STAB : salva latitudine, longitudine e quota leggendole mediante
                   funzioni di interfaccia

     Non presenta problemi di inconsistenza dei dati.

     Conflitti sul file system sono evitati attendendo che il task di 
     navigazione abbia terminato di leggere il file dei waypoint (10 secondi).

\todo
     Migliorare la sincronizzazione sostituendo l'attesa con un semaforo.


@par FAT FS

     Il file system viene montato sia dal task di log che dal task di navigazione.
     Spostare il montaggio del file system nel programma principale.

     25/08/13
     File system montato una sola volta nel programma principale.


@par Altimetro

     La lettura della temperatura dal sensore BMP085 restituisce 0xFFFF.
     Il problema � dovuto a componente difettoso, confermato da vari forum.
     La pressione compensata � espressa in decimi di mBar (o decimi di hPa o
     millesimi di Pa). Per calcolare l'altitudine in centimetri si puo' usare
     il polinomio del secondo ordine :

         H = [(p - 101325) * (p - 342104)] / 32768

     dove "H" � l'altezza espressa in cm e "p" � la pressione compensata 
     espressa in multipli di 0.1 mBar.
     L'approssimazione vale per pressioni comprese tra 800 mBar e 1013 mBar.
     L'errore massimo si ha con p = 800 mBar ed � pari a 15 metri.
     Al di sotto di 100 m dal ivello del mare e in atmosfera standard l'errore
     � inferiore al centimetro.

     11/09/13
     Modificato il driver BMP085: la temperatura viene letta ogni volta prima di
     leggere la pressione e non una sola volta all'avvio della macchina a stati.
     Le indicazioni di quota adesso sono consistenti.


@par Telemetria MAVLink
     
     link all'elenco dei comandi MAVLink implementati su ArduPilot Mega:
     http://code.google.com/p/ardupilot-mega/wiki/MAVLink

     link all'elenco dei parametri ArduPilot Mega modificabili tramite MAVLink:
     http://code.google.com/p/ardupilot-mega/wiki/MAVParam

     link alle specifiche del protocollo MAVLink:
     http://qgroundcontrol.org/mavlink/start
     http://qgroundcontrol.org/dev/mavlink_arduino_integration_tutorial
     http://qgroundcontrol.org/dev/mavlink_onboard_integration_tutorial
     
     29/07/12
     Creato branch MAVLINK per integrazione del protocollo Mavlink:

     - seguite istruzioni per l'integrazione contenute in:
        http://qgroundcontrol.org/dev/mavlink_onboard_integration_tutorial
     - modificati tutti gli "inline" come "__inline", 
     - incluso header mavlink.h contenuto nella cartella "common".

     Il codice generato richiede pi� RAM di quella disponibile nel STM32F100.
     Provato a includere file mavlink.h di altre cartelle:
       - "minimal",
       - "ardupilotmega", 
       - "matrixpilot"
     sempre troppa RAM.
     Controllato la mappa di memoria generata dal linker: 
       - nav.c piu' di 1K per le strutture dati dei file, 
       - log.c piu' di 1K per le strutture dati dei file, 
       - heap_1.c piu' 3K per l'heap, 
       - startup_stm32f10x_md_vl.c 1K per lo stack.
     
     Possibili soluzioni:
       1 un'unica struttura dati di file, accesso singolo
       2 ridurre le dimensioni dell'heap 
       3 ridurre lo stack in startup_stm32f10x_md_vl.c
     Risultato:
       1 da provare
       2 l'heap non pu� essere ridotto, il sistema si pianta
       3 ridotto lo stack a 512 bytes, funziona

     29/10/2012
     Log dei messaggi inviati da AqGCS
     Log dei messaggi inviati da aq-gcs2

     05/08/2013
     Implementati messaggi prinicipali di mavlink e verificati con GCS andropilot:
       - assetto
       - posizione GPS
       - download parametri
       - impostazione parametri
       - download waypoint
     Mancano ancora:
       - cancellazione waypoint
       - salvataggio waypoint su scheda SD

     12/09/13
     Aggiungere l'invio della posizione dei servi e dei joystick potrebbe essere
     utile per controllare l'escursione massima durante il volo in modo manuale.


@par GPS

     17/08/2013
     Sostituito Haicom H 203 con Royaltek bluetooth collegando dei fili all'interno.
     Impostato a 57600 baud rate UART del GPS.
     Dopo un po' il micro si blocca sulla routine di interrupt dell'UART.
     Interrupt troppo frequenti ?
     Interrupt nidificati ?
     Funzioni chiamate nell'interrupt non rientranti ?

     18/08/2013
     Abilitato DMA su UART GPS, il micro non si blocca pi�.


@par Telemetria MultiWii

     03/08/2012
     link all'implementazione Arduino del protocollo MultiWii
     https://github.com/wertarbyte/multiwii-firmware/blob/master/Serial.ino

     12/08/12
     creato branch MULTIWII per integrazione protocollo MultiWii:

     - implementati solo alcuni comandi
     - eliminato il buffer per la memorizzazione del messaggio
     - utilizzato un indice del buffer UART per l'inizio del messaggio
     - creato un progetto per il test del protocollo con target STM32F103RB
     - la struttura del file � inadatta al test di unit�.

     13/08/12
     installato il programma MultiWiiGUI:

     - dopo la connessione il programma invia per alcuni secondi le richieste:

     MSP_IDENT      multitype + multiwii version + protocol version + capability variable
     MSP_RC_TUNING  rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
     MSP_PID        16 PID
     MSP_BOX        16 checkbox
     MSP_MISC       powermeter trig + 8 free 

     - poi il programma invia con frequenze diverse le richieste:

     MSP_STATUS     cycletime & errors_count & sensor present & box activation
     MSP_RAW_IMU    raw IMU data, 9 DOF
     MSP_SERVO      8 servos
     MSP_MOTOR      8 motors
     MSP_RC         8 rc channels
     MSP_RAW_GPS    fix, numsat, lat, lon, alt, speed
     MSP_COMP_GPS   distance home, direction home
     MSP_ATTITUDE   roll, pitch, heading
     MSP_ALTITUDE   altitude
     MSP_BAT        vbat, powermetersum
     MSP_MISC       powermeter trig + 8 free
     MSP_DEBUG      debug1, debug2, debug3, debug4

     - alla pressione del pulsante "Write Settings" vengono aggiornati tutti i parametri,
       inviando i seguenti comandi:

     comando          lunghezza significato 
     -------------------------------------------------
     MSP_SET_PID           30   set PID
     MSP_SET_BOX           28   set checkboxes
     MSP_SET_RC_TUNING     7    set rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
     MSP_SET_MISC          2    set powermeter trig + 8 free 
     MSP_EEPROM_WRITE      0    save configuration to eeprom
     MSP_PID               0    request PID


@par Kalman

     2x3 Kalman Algorithm from a paper by Randal Beard and adapted to gluonpilot.

     The state x contains the roll and pitch angle
     Update x with the standard earth-to-body transformations:
         roll = roll + (p + q*sin(roll)*tan(pitch) + r*cos(roll)*tan(pitch)) * DT
         pitch = pitch + (q*cos(roll)- r*sin(roll)) * DT 
     Update the P-matrix of the extended kalman filter
     Calculate u (speed along x-axis) and w (speed along z-axis) from the GPS and pressure sensor.
     Calculate matrix h, which contains the estimated accelerometer readings:
         acceleration(x) = q*w + sin(pitch)* G
         acceleration(y) = (r*u - p*w) - cos(pitch)*sin(roll) * G
         acceleration(z) = (p*w - q*u) - cos(pitch)*cos(roll) * G 
     Calculate the Jacobian dh/dx to complete the extended kalman filter.
     Update the pitch and roll angle (state x) with the error between "h" and 
     the actual accelerometer readings with the innovation matrix.

     Pros: no gyro bias is calculated, perform surprisingly well. 
     Cons: uses euler angles, which are not gimbal-lock free, requires more processing power.

     13/11/12
     http://www.jhuapl.edu/ott/Technologies/Copyright/SuasCode.asp


@par Navigation
     
     vedi i todo in nav.c

     10/09/13
     Prima prova sul campo.

     Risultato:
     Stabilizzazione discreta. Tendenza a virare a dx (assetto delle ali ? inclinazione
     del motore ? pesi ?). 
     Abilitando la navigazione l'aereo entra in vite perch� L'escursione degli alettoni 
     � eccessiva.

     12/09/13
     Seconda prova sul campo.
     (regolato assetto ali rispetto fusoliera, massimo angolo di bank +/- 10�)

     Risultato:
     Stabilizzazione discreta, tende sempre a virare a dx.
     Navigazione non funziona bene.
     Perso il controllo a 250 m di distanza, aereo entrato in vite e precipitato.
     Dopo l'impatto la batteria era surriscaldata e rigonfia, l'elettronica era accesa, 
     danni medi.

     Analisi navigazione:
     Con angolo di bank a +/- 10� il controllo di direzione non ha autorit� sufficiente.

     Analisi crash:
     Controllata la tensione di funzionamento e la corrente erogabile dal DC/DC: 
     tensione minima 4,8 Volt, corrente 2 Ampere
     E' improbabile che la perdita di controllo sia dovuta a tensione di batteria bassa
     o a eccessivo assorbimento dai servi.
     Il log della posizione salvato nella scheda SD si interrompe poco lontano dal punto
     dell'impatto e a una quota di 150 m.
     Escludendo uno spegnimento temporaneo dell'elettronica (l'elettronica era accesa 
     quando l'aereo � stato recuperato) resta l'ipotesi di un blocco del micro.

     14/09/13
     Terza prova sul campo.
     (sostituito DC/DC, batteria Litio separata per l'elettronica, massimo bank +/- 20�)

     Risultato:
     Appena spostato il selettore su NAV, l'aereo non imposta una prua precisa.
     A volte prende una direzione che va all'esterno dell'area dei waypoint.
     Non � chiaro se e quando raggiunge il waypoint.
     In virata ha dei movimenti a scatti, come in simulazione.
     Perso di nuovo il controllo a circa 250 m di distanza, aereo precipitato.
     Dopo l'impatto l'elettronica era accesa, danni minori.

     Analisi navigazione:
     L'angolo di bank � calcolato in base alla differenza tra la prua corrente e la prua
     verso il prossimo wpt. La prua corrente � ricavata dall'IMU ed � aggiornata a 20 Hz.
     La prua verso il wpt � calcolata come arcotangente delle differenze di latitudine e 
     longitudine ed � aggiornata a 1 Hz (GPS).
     L'angolo di bank dovrebbe quindi essere aggiornato a 20 Hz, e non avere un andamento  
     a scatti con frequenza 1 Hz.
     Il movimento a scatti potrebbe essere dovuto a:
        - parametri del PID di navigazione, Kp = 5 (alto) e Ki = 0,001 (basso).
        - periodo di ricalcolo PID = 1 sec (troppo lungo)

\todo
     Aggiornare il PID di navigazione con frequenza maggiore, spostandolo nel task AHRS
     oppure chiamandolo anche se la funzione GPS_Parse() ritorna FALSE.

     Analisi crash:
     Ormai da escludere problemi di alimentazione, pi� probabile un blocco del micro.
     Il problema si verifica sempre alla stessa distanza e circa nella stessa zona in cui 
     � noto esserci dei disturbi.
     Se si tratta di un blocco del micro, la causa potrebbe essere un eccesso di interrupt
     di cattura dovuti a disturbi sul segnale PPM proveniente dalla radio.

     17/09/13
     Stress test interrupt di cattura con generatore di segnale all'ingresso PPM.
     GPS e �D Radio sconnessi, nessun altra fonte di interrupt esterno.

     Risultato:
     Il sistema non fa una piega. Provata frequenza di impulsi fino a 17 MHz.
     

@par PID
     
     08/10/12
     Creato progetto di test dei loop PID.
     Trovato e corretto un problema con la saturazione del termine integrale.
     Dal test su Xplane risultano i seguenti valori:
        Pitch Kp = 0
        Pitch Ki = 0
        Roll Kp = 1
        Roll Ki = 0
        Dir Kp = 0.75
        Dir Ki = 0.1
     Anche senza il PID di controllo beccheggio l'aereo � stabile e riesce a controllare 
     la quota solo variando i giri del motore. 
     Tutti i waypoint sono raggiunti correttamente, sia come coordinate che come quota.

     12/09/13
     il minimo e il massimo del termine integrale vanno modificati ogni volta che si
     modifica il guadagno integrale Ki in modo che il termine integrale non superi il
     valore massimo del controllo.
     Siccome il controllo � normalizzato a 1 e poi moltiplicato per un guadagno,
     dovrebbe essere sufficiente impostare minimo e massimo come:

        min = -(1 / Ki)
        MAX = 1 / Ki


@par Modifiche hardware
     
     Modifiche:
        - eliminare CD 4504 D / sostituire con componente reperibile 
        - correggere package LD1117 / cambiare tipo di stabilizzatore
        - sostituire sensore di pressione differenziale con MP3V5004DP
        - collegare il pin EOC del barometro BMP085 al micro
        - sostituire BMP085 con altro sensore barometrico
        - sostituire accelerometri e giroscopi con MPU 6000 Invensense
        - togliere pulsante di reset 

     Aggiunte :
        - batteria tampone per RTC
        - linea di reset per i sensori (accel, gyro, baro) comandata dal micro
        - altre uscite per i servocomandi
        - connessione servi ad angolo retto
        - soppressore di disturbi sull'alimentazione
        - SPI per leggere sensori (I2C d� troppi problemi)


@par How to use it ? 

@b 

@note
   
 * <h3><center>&copy; Lorentz</center></h3>
 */
