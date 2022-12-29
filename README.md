# Packages clasifier system
Codigo que permite al robot revisar una serie de puntos dados en una nave industrial y comprobar si en dicha posicion hay o no un codigo QR (que representa a un paquete). Ademas almacena en un archivo de texto cuales han sido las lecturas al pasar por cada punto de forma que se puede conocer si en cada una de las iteraciones del robot habia o no habia un paquete en ese punto.

El archivo clienteModeBase.py corresponde al movimiento y tratamiento de la informacion

El archivo QRcodereader.py corresponde a la lectura de codigos QR desde la camara y el envio de su informacion a un topic

