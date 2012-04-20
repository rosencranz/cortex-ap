/**
  @page Appunti
  
  @verbatim
  @endverbatim

@par Sensori

     Per semplificare il codice, creare un task di lettura dei MEMS che accoda
     un messaggio al task attitude con i valori dei sensori.
     In questo modo, il task di lettura dei MEMS � pi� facilmente sostituibile 
     dal task di telemetria che pu� inviare i valori di sensori al suo posto.

@par Simulazione

     Valori: 
     Roll_Kp = 1.0, Roll_Ki = 0.2, 
     Pitch_Kp = 1.0, Pitch_Ki = 1.0
     Dir_Kp = 5.0, Dir_Ki = 0.05

     Risultato: 
     raggiunge tutti i waypoint #1 - #6, non riesce a tornare al waypoint #1.
     Alzato Dir_Kp al massimo per correggere l'errore di direzione quando 
     l'aereo � ancora distante dal waypoint.
     Dopo il waypoint #6 la virata � appena accennata, raggiunta la direzione 
     del waypoint mantiene la direzione con un po' di errore.
     Avvicinandosi al waypoint #1 la correzione di direzione � i ritardo, 
     manca il waypoint e ci orbita attorno.
     Le oscillazioni durante le virate di navigazione sono dovute al termine
     integrale del PID di rollio, Roll_Ki, non al termine integrale del PID
     di navigazione.
     Abbassato Roll_Kpi le oscillazioni spariscono.

@par Log

     Il task di log potrebbe funzionare a tempo e salvare su SD i dati utili 
     leggendoli autonomamente mediante funzioni di interfaccia.
     Problemi di inconsistenza dei dati potrebbero essere risolti con semafori
     che "occupano" il dato quando deve essere aggiornato.

@par FAT FS

     Il file system viene montato sia dal task di log che dal task di navigazione.
     Spostare il montaggio del file system nel programma principale.

@par Altimetro

     La lettura della temperatura dal sensore BMP085 restituisce 0xFFFF.
     Il problema � dovuto a componente difettoso, confermato da vari forum.
     La pressione compensata � espressa in decimi di mBar (o decimi di hPa o
     millesimi di Pa). Per calcolare l'altitudine in centimetri si puo' usare
     il polinomio del secondo ordine :

         H = [(p - 101325) (p - 342104)] / 32768

     dove "H" � l'altezza espressa in cm e "p" � la pressione compensata 
     espressa in multipli di 0.1 mBar.
     L'approssimazione vale per pressioni comprese tra 800 mBar e 1013 mBar.
     L'errore massimo si ha con p = 800 mBar ed � pari a 15 metri.
     Al di sotto di 100 m dal ivello del mare e in atmosfera standard l'errore
     � inferiore al centimetro.


@par Telemetria
     
     link all'elenco dei comandi MAVLink implementati su ArduPilot Mega:
     http://code.google.com/p/ardupilot-mega/wiki/MAVLink

     link all'elenco dei parametri ArduPilot Mega modificabili tramite MAVLink:
     http://code.google.com/p/ardupilot-mega/wiki/MAVParam

     link alle specifiche del protocollo:
     http://qgroundcontrol.org/mavlink/start
     http://qgroundcontrol.org/dev/mavlink_arduino_integration_tutorial
     http://qgroundcontrol.org/dev/mavlink_onboard_integration_tutorial
     
@par Navigation
     
     Calcolare le differenze di longitudine e latitudine come: 

         Delta Lat = Lat2 - Lat1
         Delta Lon = (Lon2 - Lon1) * cos((Lat1 + Lat2)/2)

     Calcolare la distanza dal punto come:

         Distance = sqrt(Delta Lon ^ 2 + Delta Lat ^ 2) * 111320

@par How to use it ? 

@b 

@note
   
 * <h3><center>&copy; Lorentz</center></h3>
 */
