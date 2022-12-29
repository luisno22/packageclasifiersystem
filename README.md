# Sistema clasificador de paquetes en nave industrial
Codigo que permite al robot revisar una serie de puntos dados en una nave industrial y comprobar si en dicha posicion hay o no un codigo QR (que representa a un paquete). Ademas almacena en un archivo de texto cuales han sido las lecturas al pasar por cada punto de forma que se puede conocer si en cada una de las iteraciones del robot habia o no habia un paquete en ese punto.

Su ejecución hace uso del roscore de ROS, para poder lanzar los código es necesario el roscore de ROS así como los publicadores y suscriptores necesarios con la pose del robot y las ordenes que recibe. Tanto en el robot real como en el simulado ambas cosas se lanzan de forma automática con el uso del minimal_launch.launch
Para simulación se hace uso de la libreria Turtlebot3 que añade la definición del turtlebot y los archivos necesarios para su simulación.

El archivo clienteModeBase.py corresponde al movimiento y tratamiento de la informacion

El archivo QRcodereader.py corresponde a la lectura de codigos QR desde la camara y el envio de su informacion a un topic

Dos ejemplos de funcionamiento del código
En el robot real:
https://drive.google.com/file/d/1S6KXmhCY3bYuTcwKsoc2ZnxXmYpDoB77/view?usp=share_link

En simulación:
https://drive.google.com/file/d/1KObf_zUb7mv1K7c9Vs9nVZ0heTF79699/view?usp=share_link

