# ESW-Projekt

Gruppenprojekt (zwei Pesonen) für das Modul "Embedded Software Development".
Im Rahmen dieses Projektes wurde eine digitale Wetterstation auf einem Nucleo-F030R8 Board umgesetzt.
Die Wetterstation kann durch den User über Knöpfe bedient werden, um die Uhrzeit einzustellen und zwischen verschiedenen Anzeigemodi (Temperatur, Uhrzeit/Temperatur, usw.) zu wechseln.

Das Modul "lcd.c" dient der Ansteuerung des LCD-Bildschirms.
"temp.c" ermittelt die Temperaturdaten durch Auslesen des Sensors über einen One-Wire-Bus ("one_wire.c").

Genaure Informationen sind im Designreport zu finden.
