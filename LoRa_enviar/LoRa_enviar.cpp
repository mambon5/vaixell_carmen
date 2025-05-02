AT+MODE0     // Posar el mòdul en mode transmissió normal (normal mode)
AT+PARAMETER12,7,1,4  // Configura: potència=12, spreading factor=7, bandwidth=125kHz, coding rate=4/5
AT+FREQ=868000000   // Freqüència en Hz (868 MHz per Europa)
AT+MAC00.01        // Adreça del dispositiu (pots posar 1 i 2, per diferenciar-los)
AT+NETWORKID=6      // ID de xarxa (ha de coincidir als dos dispositius)
AT+BAUDRATE=9600,8,0,1 // UART: 9600 baudis, 8 bits, sense paritat, 1 stop bit


AT+SEND=2,5,HELLO