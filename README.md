V1.0 : Configuration du l’IOC 

Connectivity  

I2C1 sur PB9 (SDA) et PB8(SCL). F: 400 kHz. [Communication avec la carte IKS01A3] 

 SPI1 sur PA5 (SCK) PA6(MISO) PA7(MOSI) et PA8(CS).  Mode (Full-Duplex Master)[Afficheur 7 segments de la carte rouge] 

GPIO 

L0 sur PB1 en Output pour la LED sur la carte rouge 

Middleware and Software Packs 

X-CUBE-MEMS1 : Sélection de de la “Board Extension IKS01A3” sur le bus I2C1. 

 

Environnement de compilation ; 

Activation du Serial Wire Viewer pour le debug du code avec la console et le printf avec la fonction io_ptutchar(). 

 

Ajout de la bibliothèque max7219_Yncrea2.c/.h (Vers2ion modifié pour avoir le. + digit). 

 

Affichage de la température dans la console de debug. 
