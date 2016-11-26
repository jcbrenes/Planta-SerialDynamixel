# Planta-SerialDynamixel
Monitoreo de sensores de presion y Sharp con Arduino, y comunicación con un bus Dynamixel 

 * Programa que monitorea 1 sensor SHARP de proximidad y 4 sensores de presión.
 * Cuando recibe un dato serial, revisa que tenga las tramas de un mensaje Dynamixel.
 * Si además la dirección y el tipo de mensaje es correcto, devuelve en formato Dynamixel
 * el valor del sensor que se pidió o el valor del Centro de Masa
 
