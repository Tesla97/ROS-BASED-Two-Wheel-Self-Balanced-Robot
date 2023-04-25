## Two-Wheels Self-Balanced Robot

> Attenzione : La comprensione del seguente progetto richiede basi solide di Meccanica , Sistemi Dinamici , Teoria del Controllo

A ROS - BASED Two Wheels Self-Balanced Robot.

Definizione modello matematico robot tramite riformulazione Lagrangiana della NSL (Newton Second Law).
Implementazione di un filtro di Kalman Esteso per la stima dello stato del processo , a fronte della presenza di rumori gaussiani.
Implementazione di un controllore LQ per la stabilizzazione del pendolo.
Implementazione controllore ausiliario PID.

Il robot è dotato di un modulo di percezione dell'ambiente esterno (LIDAR) con l'obiettivo di effettuare una mappatura dell'ambiente circostante (SLAM Problem)


